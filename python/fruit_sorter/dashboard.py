"""Thread-safe telemetry store and FastAPI dashboard server."""

from __future__ import annotations

import copy
from datetime import datetime, timezone
import hmac
import logging
from pathlib import Path
import threading
import time
from typing import Any, Callable

import cv2
import numpy as np

from .config import AppConfig, WebConfig
from .models import ControllerState, FruitType

LOG = logging.getLogger(__name__)


def _utc_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


class DashboardStore:
    """
    Shared in-memory telemetry store.

    The robot control loop writes status while the FastAPI worker thread reads
    it. Every public method is protected by one re-entrant lock.
    """

    def __init__(self, config: AppConfig, *, dry_run: bool) -> None:
        self.config = config
        self.web = config.web
        self._lock = threading.RLock()
        self._frame_condition = threading.Condition(self._lock)
        self._jpeg: bytes | None = None
        self._jpeg_sequence = 0
        self._last_jpeg_at = 0.0
        self._started_monotonic = time.monotonic()

        self._data: dict[str, Any] = {
            "system": {
                "state": ControllerState.STARTING.value,
                "dry_run": dry_run,
                "calibration_complete": config.calibration.calibration_complete,
                "automatic_motion_enabled": config.calibration.calibration_complete,
                "camera_connected": False,
                "robot_connected": False,
                "frame_number": 0,
                "processing_fps": 0.0,
                "inference_ms": 0.0,
                "last_update": _utc_iso(),
            },
            "safety": {
                "emergency_stop": False,
                "fault": "unknown",
                "physical_estop": None,
                "outputs_enabled": False,
                "motion_busy": False,
                "last_error": None,
                "remote_estop_enabled": config.web.allow_remote_estop,
            },
            "robot": {
                "servo_names": [servo.name for servo in config.arm.servos],
                "pose_deg": list(config.arm.home_deg),
            },
            "sort_counts": {
                FruitType.APPLE.value: 0,
                FruitType.BANANA.value: 0,
                FruitType.ORANGE.value: 0,
                "total": 0,
            },
            "active_target": None,
            "last_sort": None,
            "detections": [],
            "events": [],
        }

    def _touch(self) -> None:
        self._data["system"]["last_update"] = _utc_iso()

    def add_event(self, level: str, message: str) -> None:
        with self._lock:
            events = self._data["events"]
            events.insert(
                0,
                {
                    "time": _utc_iso(),
                    "level": level.lower(),
                    "message": message,
                },
            )
            del events[self.web.event_history :]
            self._touch()

    def set_state(self, state: ControllerState) -> None:
        with self._lock:
            self._data["system"]["state"] = state.value
            self._touch()

    def set_camera_connected(self, connected: bool) -> None:
        with self._lock:
            self._data["system"]["camera_connected"] = bool(connected)
            self._touch()

    def set_robot_connected(self, connected: bool) -> None:
        with self._lock:
            self._data["system"]["robot_connected"] = bool(connected)
            self._touch()

    def set_error(self, message: str | None) -> None:
        with self._lock:
            self._data["safety"]["last_error"] = message
            self._touch()
        if message:
            self.add_event("error", message)

    def set_emergency_stop(self, asserted: bool, reason: str | None = None) -> None:
        with self._lock:
            self._data["safety"]["emergency_stop"] = bool(asserted)
            if reason:
                self._data["safety"]["last_error"] = reason
            self._touch()
        if asserted:
            self.add_event("critical", reason or "Emergency stop asserted")

    def set_active_target(self, target: dict[str, Any] | None) -> None:
        with self._lock:
            self._data["active_target"] = copy.deepcopy(target)
            self._touch()

    def update_robot(self, telemetry: dict[str, Any]) -> None:
        with self._lock:
            if "pose_deg" in telemetry:
                self._data["robot"]["pose_deg"] = [
                    round(float(value), 2) for value in telemetry["pose_deg"]
                ]
            self._data["system"]["robot_connected"] = bool(
                telemetry.get("connected", self._data["system"]["robot_connected"])
            )
            for key in (
                "fault",
                "physical_estop",
                "outputs_enabled",
                "motion_busy",
            ):
                if key in telemetry:
                    self._data["safety"][key] = telemetry[key]
            if telemetry.get("communication_error"):
                self._data["safety"]["last_error"] = telemetry["communication_error"]
            self._touch()

    def publish_cycle(
        self,
        *,
        annotated_frame: np.ndarray,
        frame_number: int,
        detections: list[dict[str, Any]],
        processing_fps: float,
        inference_ms: float,
        robot_telemetry: dict[str, Any],
    ) -> None:
        with self._lock:
            self._data["system"]["frame_number"] = int(frame_number)
            self._data["system"]["processing_fps"] = round(processing_fps, 2)
            self._data["system"]["inference_ms"] = round(inference_ms, 1)
            self._data["system"]["camera_connected"] = True
            self._data["detections"] = copy.deepcopy(detections)
            self.update_robot(robot_telemetry)
            self._touch()

            now = time.monotonic()
            minimum_interval = 1.0 / self.web.stream_fps
            if now - self._last_jpeg_at < minimum_interval:
                return

            encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.web.jpeg_quality]
            ok, encoded = cv2.imencode(".jpg", annotated_frame, encode_params)
            if not ok:
                LOG.warning("Unable to JPEG-encode dashboard frame")
                return

            self._jpeg = encoded.tobytes()
            self._jpeg_sequence += 1
            self._last_jpeg_at = now
            self._frame_condition.notify_all()

    def record_sort(
        self,
        *,
        fruit: FruitType,
        track_id: int | None,
        x_mm: float,
        y_mm: float,
        cycle_seconds: float,
    ) -> None:
        with self._lock:
            counts = self._data["sort_counts"]
            counts[fruit.value] += 1
            counts["total"] += 1
            self._data["last_sort"] = {
                "fruit": fruit.value,
                "track_id": track_id,
                "x_mm": round(x_mm, 1),
                "y_mm": round(y_mm, 1),
                "cycle_seconds": round(cycle_seconds, 2),
                "completed_at": _utc_iso(),
            }
            self._data["active_target"] = None
            self._touch()
        self.add_event(
            "success",
            f"Sorted {fruit.value} to its box in {cycle_seconds:.2f} s",
        )

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            result = copy.deepcopy(self._data)
            result["system"]["uptime_seconds"] = round(
                time.monotonic() - self._started_monotonic,
                1,
            )
            return result

    def wait_for_jpeg(
        self,
        after_sequence: int,
        timeout_s: float,
    ) -> tuple[bytes | None, int]:
        deadline = time.monotonic() + timeout_s
        with self._frame_condition:
            while self._jpeg_sequence <= after_sequence:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return self._jpeg, self._jpeg_sequence
                self._frame_condition.wait(timeout=remaining)
            return self._jpeg, self._jpeg_sequence


class DashboardServer:
    """Runs FastAPI/Uvicorn in a background thread beside the control loop."""

    def __init__(
        self,
        config: WebConfig,
        store: DashboardStore,
        emergency_stop_callback: Callable[[], None],
    ) -> None:
        self.config = config
        self.store = store
        self.emergency_stop_callback = emergency_stop_callback
        self._thread: threading.Thread | None = None
        self._server: Any = None
        self.app = self._build_app()

    def _build_app(self):
        from fastapi import FastAPI, Header, HTTPException
        from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
        from fastapi.staticfiles import StaticFiles

        static_dir = Path(__file__).with_name("web")
        app = FastAPI(
            title="UNO Q Fruit Sorter Dashboard",
            docs_url=None,
            redoc_url=None,
        )
        app.mount("/static", StaticFiles(directory=static_dir), name="static")

        @app.get("/")
        def index():
            return FileResponse(static_dir / "index.html")

        @app.get("/api/status")
        def status():
            return JSONResponse(
                self.store.snapshot(),
                headers={"Cache-Control": "no-store"},
            )

        @app.get("/api/health")
        def health():
            snapshot = self.store.snapshot()
            healthy = (
                snapshot["system"]["camera_connected"]
                and snapshot["system"]["robot_connected"]
                and not snapshot["safety"]["emergency_stop"]
            )
            return JSONResponse(
                {"healthy": healthy, "state": snapshot["system"]["state"]},
                status_code=200 if healthy else 503,
                headers={"Cache-Control": "no-store"},
            )

        @app.get("/video")
        def video():
            def generate():
                sequence = -1
                while True:
                    jpeg, sequence = self.store.wait_for_jpeg(sequence, timeout_s=2.0)
                    if jpeg is None:
                        continue
                    yield (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        b"Cache-Control: no-store\r\n\r\n"
                        + jpeg
                        + b"\r\n"
                    )

            return StreamingResponse(
                generate(),
                media_type="multipart/x-mixed-replace; boundary=frame",
                headers={
                    "Cache-Control": "no-store, no-cache, must-revalidate",
                    "Pragma": "no-cache",
                },
            )

        @app.post("/api/emergency-stop")
        def emergency_stop(x_control_token: str | None = Header(default=None)):
            if not self.config.allow_remote_estop:
                raise HTTPException(
                    status_code=403,
                    detail="Remote emergency stop is disabled",
                )
            provided = x_control_token or ""
            if not hmac.compare_digest(provided, self.config.control_token):
                raise HTTPException(status_code=401, detail="Invalid control token")

            self.emergency_stop_callback()
            return {"ok": True, "message": "Emergency stop requested"}

        return app

    def start(self) -> None:
        if not self.config.enabled:
            LOG.info("Web dashboard disabled")
            return
        if self._thread and self._thread.is_alive():
            return

        import uvicorn

        server_config = uvicorn.Config(
            self.app,
            host=self.config.host,
            port=self.config.port,
            log_level=self.config.log_level.lower(),
            access_log=False,
        )
        self._server = uvicorn.Server(server_config)

        self._thread = threading.Thread(
            target=self._server.run,
            name="web-dashboard",
            daemon=True,
        )
        self._thread.start()
        LOG.info(
            "Dashboard listening on http://%s:%d",
            self.config.host,
            self.config.port,
        )

    def stop(self) -> None:
        if self._server is not None:
            self._server.should_exit = True
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        LOG.info("Dashboard stopped")
