"""Main state machine for continuous fruit sorting."""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any

import cv2
import numpy as np

from .calibration import PixelMapper
from .camera import CameraStream
from .colour import ColourVerifier
from .config import AppConfig
from .dashboard import DashboardStore
from .detector import FruitDetector
from .exceptions import (
    CameraError,
    FruitSorterError,
    KinematicsError,
    RobotFaultError,
    SafetyError,
)
from .kinematics import ArmKinematics
from .models import (
    ColourResult,
    ControllerState,
    Detection,
    JointPose,
    Point2D,
    Point3D,
)
from .robot_client import RobotInterface
from .safety import SafetyChecker
from .tracking import PickedRegistry, StabilityTracker

LOG = logging.getLogger(__name__)


class FruitSortingController:
    def __init__(
        self,
        config: AppConfig,
        camera: CameraStream,
        detector: FruitDetector,
        robot: RobotInterface,
        dashboard: DashboardStore | None = None,
    ) -> None:
        self.config = config
        self.camera = camera
        self.detector = detector
        self.robot = robot
        self.dashboard = dashboard

        self.mapper = PixelMapper(config.calibration)
        self.colour = ColourVerifier(config.colour)
        self.stability = StabilityTracker(config.tracking)
        self.picked = PickedRegistry(config.tracking)
        self.kinematics = ArmKinematics(config.arm)
        self.safety = SafetyChecker(config.workspace)

        self.state = ControllerState.STARTING
        self._stop_event = threading.Event()
        self._frame_number = 0
        self._last_camera_sequence: int | None = None
        self._rejected_until: dict[int, float] = {}
        self._last_frame_completed_at: float | None = None
        self._smoothed_fps = 0.0

    def _set_state(self, state: ControllerState) -> None:
        self.state = state
        if self.dashboard:
            self.dashboard.set_state(state)

    def request_stop(self) -> None:
        self._stop_event.set()
        if self.dashboard:
            self.dashboard.add_event("warning", "Controlled shutdown requested")

    def emergency_stop(self) -> None:
        self._set_state(ControllerState.ESTOP)
        self.robot.emergency_stop()
        if self.dashboard:
            self.dashboard.set_emergency_stop(
                True,
                "Software emergency stop requested",
            )
            self.dashboard.update_robot(self.robot.telemetry())
        self._stop_event.set()

    def _duration_for(self, target: JointPose) -> int:
        maximum_delta = max(
            abs(target_value - current_value)
            for target_value, current_value in zip(
                target.angles_deg,
                self.robot.current_pose.angles_deg,
            )
        )
        raw_ms = maximum_delta / self.config.motion.max_joint_speed_deg_s * 1000.0
        return int(
            max(
                self.config.motion.min_move_ms,
                min(self.config.motion.max_move_ms, raw_ms),
            )
        )

    def _move_pose(self, pose: JointPose) -> None:
        self.robot.move_pose(pose, self._duration_for(pose))
        if self.dashboard:
            self.dashboard.update_robot(self.robot.telemetry())

    def _move_point(self, point: Point3D, gripper_deg: float) -> None:
        self.safety.validate_point(point)
        pose = self.kinematics.solve(point, gripper_deg)
        self._move_pose(pose)

    def _move_line(
        self,
        start: Point3D,
        end: Point3D,
        gripper_deg: float,
    ) -> None:
        for point in self.safety.line_samples(
            start,
            end,
            self.config.motion.path_step_mm,
        ):
            self._move_point(point, gripper_deg)

    def _open_gripper_at_current_pose(self) -> None:
        gripper_index = self.config.arm.servo_index("gripper")
        pose = self.robot.current_pose.with_servo(
            gripper_index,
            self.config.arm.gripper.open_deg,
        )
        self._move_pose(pose)

    def _close_gripper_at_current_pose(self) -> None:
        gripper_index = self.config.arm.servo_index("gripper")
        pose = self.robot.current_pose.with_servo(
            gripper_index,
            self.config.arm.gripper.closed_deg,
        )
        self._move_pose(pose)
        time.sleep(self.config.arm.gripper.close_dwell_s)

    def _home(self) -> None:
        self._set_state(ControllerState.HOMING)
        home = JointPose(tuple(self.config.arm.home_deg))  # type: ignore[arg-type]
        self._move_pose(home)

    def _target_robot_xy(self, detection: Detection) -> Point2D:
        mapped = self.mapper.pixel_to_robot(detection.center_px)
        offset_x, offset_y = self.config.calibration.pick_offsets_mm[detection.fruit]
        return Point2D(mapped.x + offset_x, mapped.y + offset_y)

    def _is_rejected(self, track_id: int | None) -> bool:
        if track_id is None:
            return True
        expiry = self._rejected_until.get(track_id, 0.0)
        if expiry <= time.monotonic():
            self._rejected_until.pop(track_id, None)
            return False
        return True

    def _reject_temporarily(self, detection: Detection) -> None:
        if detection.track_id is not None:
            self._rejected_until[detection.track_id] = (
                time.monotonic() + self.config.tracking.rejected_cooldown_seconds
            )

    def _select_target(
        self,
        frame: np.ndarray,
        detections: list[Detection],
    ) -> tuple[Detection, ColourResult, Point2D] | None:
        candidates: list[tuple[float, Detection, ColourResult, Point2D]] = []

        for detection in detections:
            if detection.track_id is None:
                continue
            if self._is_rejected(detection.track_id):
                continue
            if detection.area_px < self.config.tracking.min_bbox_area_px:
                continue
            if not self.mapper.inside_pick_roi(detection.center_px):
                continue
            if not self.stability.is_stable(detection):
                continue

            colour = self.colour.verify(frame, detection)
            if not colour.accepted:
                continue

            try:
                robot_xy = self._target_robot_xy(detection)
            except FruitSorterError:
                self._reject_temporarily(detection)
                continue

            if self.picked.contains(
                track_id=detection.track_id,
                fruit=detection.fruit,
                point=robot_xy,
            ):
                continue

            # Prefer high confidence and a larger visible target.
            score = detection.confidence + 0.05 * math.log1p(detection.area_px)
            candidates.append((score, detection, colour, robot_xy))

        if not candidates:
            return None
        _, detection, colour, robot_xy = max(candidates, key=lambda item: item[0])
        return detection, colour, robot_xy

    def _revalidate_target(
        self,
        original: Detection,
    ) -> tuple[Detection, Point2D] | None:
        # Obtain a genuinely newer frame rather than a buffered frame.
        frame, sequence = self.camera.read(after_sequence=self._last_camera_sequence)
        self._last_camera_sequence = sequence
        detections = self.detector.detect_and_track(frame)

        for detection in detections:
            if detection.track_id != original.track_id:
                continue
            if detection.fruit != original.fruit:
                return None
            if (
                detection.center_px.distance_to(original.center_px)
                > self.config.tracking.revalidate_max_shift_px
            ):
                return None
            colour = self.colour.verify(frame, detection)
            if not colour.accepted:
                return None
            return detection, self._target_robot_xy(detection)
        return None

    def _pick_and_place(self, detection: Detection, target_xy: Point2D) -> None:
        cycle_started = time.monotonic()
        fruit = detection.fruit
        pick_z = self.config.calibration.pick_z_mm[fruit]
        safe_z = self.config.motion.safe_travel_z_mm
        approach_z = min(
            safe_z,
            pick_z + self.config.motion.approach_clearance_mm,
        )

        pick = Point3D(target_xy.x, target_xy.y, pick_z)
        approach = Point3D(target_xy.x, target_xy.y, approach_z)
        pick_safe = Point3D(target_xy.x, target_xy.y, safe_z)

        box = self.config.boxes[fruit]
        box_safe = Point3D(box.x_mm, box.y_mm, safe_z)
        release = Point3D(box.x_mm, box.y_mm, box.release_z_mm)

        # Validate all principal waypoints before the first movement.
        for point in (pick, approach, pick_safe, box_safe, release):
            self.safety.validate_point(point)
            self.kinematics.solve(point, self.config.arm.gripper.open_deg)

        LOG.info(
            "Sorting %s track=%s at robot=(%.1f, %.1f, %.1f) mm",
            fruit.value,
            detection.track_id,
            pick.x,
            pick.y,
            pick.z,
        )
        if self.dashboard:
            self.dashboard.add_event(
                "info",
                f"Picking {fruit.value}, track {detection.track_id}",
            )

        self._set_state(ControllerState.PICKING)
        self._open_gripper_at_current_pose()
        self._move_point(pick_safe, self.config.arm.gripper.open_deg)
        self._move_line(pick_safe, approach, self.config.arm.gripper.open_deg)
        self._move_line(approach, pick, self.config.arm.gripper.open_deg)
        self._close_gripper_at_current_pose()
        self._move_line(pick, approach, self.config.arm.gripper.closed_deg)
        self._move_line(approach, pick_safe, self.config.arm.gripper.closed_deg)

        self._set_state(ControllerState.PLACING)
        self._move_line(pick_safe, box_safe, self.config.arm.gripper.closed_deg)
        self._move_line(box_safe, release, self.config.arm.gripper.closed_deg)
        self._open_gripper_at_current_pose()
        time.sleep(self.config.arm.gripper.release_dwell_s)
        self._move_line(release, box_safe, self.config.arm.gripper.open_deg)
        self._home()

        self.picked.mark(
            track_id=detection.track_id,
            fruit=fruit,
            point=target_xy,
        )
        cycle_seconds = time.monotonic() - cycle_started
        if self.dashboard:
            self.dashboard.record_sort(
                fruit=fruit,
                track_id=detection.track_id,
                x_mm=target_xy.x,
                y_mm=target_xy.y,
                cycle_seconds=cycle_seconds,
            )
        LOG.info("Completed %s sorting cycle in %.2f s", fruit.value, cycle_seconds)

    def _detection_telemetry(
        self,
        frame: np.ndarray,
        detections: list[Detection],
    ) -> list[dict[str, Any]]:
        rows: list[dict[str, Any]] = []

        for detection in detections:
            colour = self.colour.verify(frame, detection)
            stable = self.stability.is_stable(detection)
            inside_roi = self.mapper.inside_pick_roi(detection.center_px)
            sufficiently_large = (
                detection.area_px >= self.config.tracking.min_bbox_area_px
            )
            robot_xy: Point2D | None = None
            already_picked = False
            mapping_error: str | None = None

            if inside_roi:
                try:
                    robot_xy = self._target_robot_xy(detection)
                    already_picked = self.picked.contains(
                        track_id=detection.track_id,
                        fruit=detection.fruit,
                        point=robot_xy,
                    )
                except FruitSorterError as exc:
                    mapping_error = str(exc)

            if detection.track_id is None:
                status = "untracked"
            elif not sufficiently_large:
                status = "too_small"
            elif not inside_roi:
                status = "outside_roi"
            elif mapping_error:
                status = "mapping_error"
            elif not colour.accepted:
                status = "wrong_colour"
            elif already_picked:
                status = "already_picked"
            elif not stable:
                status = "moving"
            else:
                status = "ready"

            rows.append(
                {
                    "fruit": detection.fruit.value,
                    "track_id": detection.track_id,
                    "confidence": round(detection.confidence, 4),
                    "colour_ratio": round(colour.ratio, 4),
                    "colour_valid": colour.accepted,
                    "stable": stable,
                    "inside_roi": inside_roi,
                    "area_px": detection.area_px,
                    "center_x_px": round(detection.center_px.x, 1),
                    "center_y_px": round(detection.center_px.y, 1),
                    "robot_x_mm": None if robot_xy is None else round(robot_xy.x, 1),
                    "robot_y_mm": None if robot_xy is None else round(robot_xy.y, 1),
                    "status": status,
                }
            )
        return rows

    def _annotate(
        self,
        frame: np.ndarray,
        detections: list[Detection],
        telemetry_rows: list[dict[str, Any]],
    ) -> np.ndarray:
        annotated = frame.copy()
        cv2.polylines(
            annotated,
            [self.mapper.roi_polygon],
            isClosed=True,
            color=(255, 255, 255),
            thickness=2,
        )

        for detection, row in zip(detections, telemetry_rows):
            if row["status"] == "ready":
                draw_colour = (0, 200, 0)
            elif row["status"] == "moving":
                draw_colour = (0, 190, 255)
            else:
                draw_colour = (0, 0, 220)

            x1, y1, x2, y2 = detection.bbox_xyxy
            cv2.rectangle(annotated, (x1, y1), (x2, y2), draw_colour, 2)
            label = (
                f"{detection.fruit.value} id={detection.track_id} "
                f"conf={detection.confidence:.2f} "
                f"colour={row['colour_ratio']:.2f} {row['status']}"
            )
            cv2.putText(
                annotated,
                label,
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                draw_colour,
                2,
                cv2.LINE_AA,
            )

        cv2.putText(
            annotated,
            f"State: {self.state.value} | E=E-stop | Q=quit",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        if not self.config.calibration.calibration_complete:
            cv2.putText(
                annotated,
                "AUTOMATIC MOTION LOCKED: calibration_complete=false",
                (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 180, 255),
                2,
                cv2.LINE_AA,
            )
        return annotated

    def _update_fps(self) -> float:
        now = time.monotonic()
        if self._last_frame_completed_at is None:
            self._last_frame_completed_at = now
            return 0.0

        elapsed = now - self._last_frame_completed_at
        self._last_frame_completed_at = now
        instantaneous = 1.0 / elapsed if elapsed > 0 else 0.0
        if self._smoothed_fps == 0.0:
            self._smoothed_fps = instantaneous
        else:
            self._smoothed_fps = 0.85 * self._smoothed_fps + 0.15 * instantaneous
        return self._smoothed_fps

    def startup(self) -> None:
        self._set_state(ControllerState.STARTING)
        self.camera.start()
        if self.dashboard:
            self.dashboard.set_camera_connected(True)
            self.dashboard.add_event("success", "Camera initialised")

        self.robot.connect_and_initialize()
        if self.dashboard:
            self.dashboard.set_robot_connected(True)
            self.dashboard.update_robot(self.robot.telemetry())
            self.dashboard.add_event("success", "PCA9685 and robot link initialised")

        self._home()
        self.detector.load()
        if self.dashboard:
            self.dashboard.add_event("success", "YOLOv8 model loaded")
        self._set_state(ControllerState.IDLE)

        if not self.config.calibration.calibration_complete:
            warning = "Automatic picking locked: calibration_complete=false"
            LOG.warning(warning)
            if self.dashboard:
                self.dashboard.add_event("warning", warning)

    def run(self) -> None:
        self.startup()

        try:
            while not self._stop_event.is_set():
                self._set_state(ControllerState.ACQUIRING)
                frame, sequence = self.camera.read(
                    after_sequence=self._last_camera_sequence
                )
                self._last_camera_sequence = sequence
                self._frame_number += 1

                inference_started = time.perf_counter()
                detections = self.detector.detect_and_track(frame)
                inference_ms = (time.perf_counter() - inference_started) * 1000.0
                self.stability.update(detections, self._frame_number)

                telemetry_rows = self._detection_telemetry(frame, detections)
                annotated = self._annotate(frame, detections, telemetry_rows)
                processing_fps = self._update_fps()

                if self.dashboard:
                    self.dashboard.publish_cycle(
                        annotated_frame=annotated,
                        frame_number=self._frame_number,
                        detections=telemetry_rows,
                        processing_fps=processing_fps,
                        inference_ms=inference_ms,
                        robot_telemetry=self.robot.telemetry(),
                    )

                if self.config.application.display:
                    cv2.imshow(self.config.application.window_name, annotated)
                    key = cv2.waitKey(self.config.application.loop_delay_ms) & 0xFF
                    if key in (ord("e"), ord("E")):
                        LOG.critical("Keyboard emergency stop requested")
                        self.emergency_stop()
                        break
                    if key in (ord("q"), ord("Q")):
                        LOG.info("Keyboard shutdown requested")
                        self.request_stop()
                        break

                self._set_state(ControllerState.IDLE)

                if not self.config.calibration.calibration_complete:
                    continue

                selected = self._select_target(frame, detections)
                if selected is None:
                    continue

                detection, colour, target_xy = selected
                if self.dashboard:
                    self.dashboard.set_active_target(
                        {
                            "fruit": detection.fruit.value,
                            "track_id": detection.track_id,
                            "confidence": detection.confidence,
                            "colour_ratio": colour.ratio,
                            "x_mm": target_xy.x,
                            "y_mm": target_xy.y,
                        }
                    )

                LOG.info(
                    "Candidate %s id=%s confidence=%.3f colour_ratio=%.3f",
                    detection.fruit.value,
                    detection.track_id,
                    detection.confidence,
                    colour.ratio,
                )

                revalidated = self._revalidate_target(detection)
                if revalidated is None:
                    LOG.info("Target moved or failed colour revalidation")
                    self._reject_temporarily(detection)
                    if self.dashboard:
                        self.dashboard.set_active_target(None)
                        self.dashboard.add_event(
                            "warning",
                            f"Rejected moving target {detection.track_id}",
                        )
                    continue

                latest_detection, latest_xy = revalidated
                try:
                    self._pick_and_place(latest_detection, latest_xy)
                except (SafetyError, KinematicsError) as exc:
                    LOG.error("Rejected unsafe/unreachable target: %s", exc)
                    self._reject_temporarily(latest_detection)
                    if self.dashboard:
                        self.dashboard.set_active_target(None)
                        self.dashboard.set_error(str(exc))
                except RobotFaultError:
                    LOG.exception("Robot fault during pick cycle")
                    self.emergency_stop()
                    raise

        except CameraError as exc:
            LOG.exception("Camera failure")
            if self.dashboard:
                self.dashboard.set_camera_connected(False)
                self.dashboard.set_error(str(exc))
            self.emergency_stop()
            raise
        except Exception as exc:
            if self.dashboard:
                self.dashboard.set_error(str(exc))
            raise
        finally:
            self._set_state(ControllerState.STOPPED)
            self.camera.close()
            if self.dashboard:
                self.dashboard.set_camera_connected(False)
            self.robot.shutdown()
            if self.dashboard:
                self.dashboard.update_robot(self.robot.telemetry())
                self.dashboard.set_robot_connected(False)
            cv2.destroyAllWindows()
