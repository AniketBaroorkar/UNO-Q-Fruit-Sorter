"""High-level robot interface backed by MCU RPC calls."""

from __future__ import annotations

import copy
from enum import IntEnum
import logging
import threading
import time
from typing import Any, Protocol

from .config import AppConfig
from .exceptions import RobotCommunicationError, RobotFaultError
from .models import JointPose
from .robot_rpc import RouterRPCClient

LOG = logging.getLogger(__name__)


class FaultCode(IntEnum):
    NONE = 0
    PHYSICAL_ESTOP = 1
    WATCHDOG = 2
    SOFTWARE_ESTOP = 3
    INVALID_COMMAND = 4
    I2C_FAILURE = 5


class RobotInterface(Protocol):
    current_pose: JointPose

    def connect_and_initialize(self) -> None: ...
    def move_pose(self, pose: JointPose, duration_ms: int) -> None: ...
    def telemetry(self) -> dict[str, Any]: ...
    def emergency_stop(self) -> None: ...
    def shutdown(self) -> None: ...


class RobotClient:
    def __init__(self, config: AppConfig) -> None:
        self.config = config
        self.rpc = RouterRPCClient(
            config.safety.rpc_socket,
            config.safety.rpc_timeout_s,
        )
        self.current_pose = JointPose(tuple(config.arm.home_deg))  # type: ignore[arg-type]

        self._heartbeat_stop = threading.Event()
        self._heartbeat_thread: threading.Thread | None = None
        self._telemetry_lock = threading.RLock()
        self._telemetry: dict[str, Any] = {
            "connected": False,
            "fault": "unknown",
            "physical_estop": None,
            "outputs_enabled": False,
            "motion_busy": False,
            "pose_deg": list(self.current_pose.angles_deg),
            "communication_error": None,
        }

    def _set_telemetry(self, **values: Any) -> None:
        with self._telemetry_lock:
            self._telemetry.update(values)
            self._telemetry["pose_deg"] = list(self.current_pose.angles_deg)

    def telemetry(self) -> dict[str, Any]:
        with self._telemetry_lock:
            result = copy.deepcopy(self._telemetry)
            result["pose_deg"] = list(self.current_pose.angles_deg)
            return result

    def _fault(self) -> FaultCode:
        return FaultCode(int(self.rpc.call("get_fault")))

    def _raise_on_fault(self) -> None:
        fault = self._fault()
        self._set_telemetry(fault=fault.name.lower())
        if fault != FaultCode.NONE:
            raise RobotFaultError(f"MCU fault: {fault.name} ({int(fault)})")

    def _poll_mcu_telemetry(self) -> None:
        fault = FaultCode(int(self.rpc.call("get_fault")))
        self._set_telemetry(
            connected=True,
            fault=fault.name.lower(),
            physical_estop=bool(self.rpc.call("physical_estop")),
            outputs_enabled=bool(self.rpc.call("outputs_enabled")),
            motion_busy=bool(self.rpc.call("motion_busy")),
            communication_error=None,
        )

    def _heartbeat_loop(self) -> None:
        """
        Keep the MCU watchdog alive in both idle and motion states.

        Without this background task, a safety watchdog configured for two
        seconds would correctly disable outputs while the robot waits for fruit.
        """
        next_status_poll = 0.0
        while not self._heartbeat_stop.is_set():
            try:
                self.rpc.call("heartbeat")
                now = time.monotonic()
                if now >= next_status_poll:
                    self._poll_mcu_telemetry()
                    next_status_poll = now + 0.75
            except Exception as exc:
                self._set_telemetry(
                    connected=False,
                    communication_error=str(exc),
                )
                LOG.error("MCU heartbeat/status failure: %s", exc)

            self._heartbeat_stop.wait(self.config.safety.heartbeat_interval_s)

    def _start_heartbeat(self) -> None:
        if self._heartbeat_thread and self._heartbeat_thread.is_alive():
            return
        self._heartbeat_stop.clear()
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop,
            name="mcu-heartbeat",
            daemon=True,
        )
        self._heartbeat_thread.start()

    def _stop_heartbeat(self) -> None:
        self._heartbeat_stop.set()
        if self._heartbeat_thread:
            self._heartbeat_thread.join(timeout=2.0)

    def connect_and_initialize(self) -> None:
        deadline = time.monotonic() + self.config.safety.connect_timeout_s
        last_error: Exception | None = None

        while time.monotonic() < deadline:
            try:
                version = int(self.rpc.call("ping"))
                LOG.info("Connected to MCU motion firmware version %d", version)
                break
            except Exception as exc:
                last_error = exc
                time.sleep(0.5)
        else:
            raise RobotCommunicationError(
                f"Unable to contact MCU motion firmware: {last_error}"
            )

        self.rpc.call(
            "configure_watchdog",
            int(self.config.safety.watchdog_timeout_ms),
        )

        for logical_index, servo in enumerate(self.config.arm.servos):
            status = int(
                self.rpc.call(
                    "configure_servo",
                    logical_index,
                    servo.channel,
                    float(servo.min_deg),
                    float(servo.max_deg),
                    int(servo.min_pulse_us),
                    int(servo.max_pulse_us),
                )
            )
            if status != 0:
                raise RobotCommunicationError(
                    f"MCU rejected servo configuration for {servo.name}: {status}"
                )

        self.rpc.call("heartbeat")

        # Startup can latch a watchdog fault before Linux becomes ready.
        reset_status = int(self.rpc.call("reset_estop"))
        if reset_status != 0:
            fault = FaultCode(reset_status)
            raise RobotFaultError(
                f"Cannot reset MCU safety state: {fault.name}. "
                "Check the normally-closed E-stop circuit on D2."
            )

        enable_status = int(self.rpc.call("enable_outputs"))
        if enable_status != 0:
            raise RobotFaultError(
                f"Cannot enable PCA9685 outputs, MCU status={enable_status}"
            )

        # The MCU preloads 90-degree values while OE is disabled.
        self.current_pose = JointPose(tuple(self.config.arm.home_deg))  # type: ignore[arg-type]
        self._poll_mcu_telemetry()
        self._start_heartbeat()
        LOG.info("PCA9685 configured and six servos enabled at 90° home")

    def move_pose(self, pose: JointPose, duration_ms: int) -> None:
        self._raise_on_fault()

        status = int(
            self.rpc.call(
                "start_motion",
                *[float(value) for value in pose.angles_deg],
                int(duration_ms),
            )
        )
        if status != 0:
            try:
                fault_name = FaultCode(status).name
            except ValueError:
                fault_name = f"STATUS_{status}"
            raise RobotFaultError(f"MCU rejected motion: {fault_name}")

        self._set_telemetry(motion_busy=True)
        timeout_s = duration_ms / 1000.0 + self.config.motion.command_timeout_margin_s
        deadline = time.monotonic() + timeout_s

        while True:
            self._raise_on_fault()
            busy = bool(self.rpc.call("motion_busy"))
            self._set_telemetry(motion_busy=busy)
            if not busy:
                self.current_pose = pose
                self._set_telemetry(
                    pose_deg=list(pose.angles_deg),
                    motion_busy=False,
                )
                return

            if time.monotonic() >= deadline:
                self.emergency_stop()
                raise RobotFaultError(
                    f"Motion exceeded timeout ({timeout_s:.2f} seconds)"
                )
            time.sleep(0.03)

    def emergency_stop(self) -> None:
        try:
            self.rpc.call("software_estop")
            self._set_telemetry(
                fault=FaultCode.SOFTWARE_ESTOP.name.lower(),
                outputs_enabled=False,
                motion_busy=False,
            )
        except Exception as exc:
            self._set_telemetry(
                connected=False,
                communication_error=str(exc),
            )
            LOG.exception("Unable to send software E-stop to MCU")

    def shutdown(self) -> None:
        if self.config.safety.shutdown_disables_outputs:
            try:
                self.rpc.call("disable_outputs")
                self._set_telemetry(outputs_enabled=False, motion_busy=False)
            except Exception:
                LOG.exception("Unable to disable outputs during shutdown")
        self._stop_heartbeat()


class SimulatedRobotClient:
    """Dry-run implementation that validates the entire software pipeline."""

    def __init__(self, config: AppConfig) -> None:
        self.config = config
        self.current_pose = JointPose(tuple(config.arm.home_deg))  # type: ignore[arg-type]
        self._stopped = False
        self._connected = False

    def connect_and_initialize(self) -> None:
        LOG.warning("DRY RUN: robot hardware calls are simulated")
        self._stopped = False
        self._connected = True

    def move_pose(self, pose: JointPose, duration_ms: int) -> None:
        if self._stopped:
            raise RobotFaultError("Simulated robot is emergency-stopped")
        LOG.info(
            "DRY RUN motion duration=%dms pose=%s",
            duration_ms,
            [round(value, 1) for value in pose.angles_deg],
        )
        time.sleep(min(0.05, duration_ms / 1000.0 / 20.0))
        self.current_pose = pose

    def telemetry(self) -> dict[str, Any]:
        return {
            "connected": self._connected,
            "fault": "software_estop" if self._stopped else "none",
            "physical_estop": False,
            "outputs_enabled": self._connected and not self._stopped,
            "motion_busy": False,
            "pose_deg": list(self.current_pose.angles_deg),
            "communication_error": None,
        }

    def emergency_stop(self) -> None:
        self._stopped = True
        LOG.error("DRY RUN emergency stop")

    def shutdown(self) -> None:
        self._connected = False
        LOG.info("DRY RUN robot shutdown")
