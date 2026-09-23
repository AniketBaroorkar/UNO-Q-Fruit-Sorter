"""Typed YAML configuration with strict validation."""

from __future__ import annotations

from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .exceptions import ConfigurationError
from .models import FruitType


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid")


class ApplicationConfig(StrictModel):
    display: bool = True
    window_name: str = "UNO Q Fruit Sorter"
    loop_delay_ms: int = Field(default=1, ge=1, le=1000)


class WebConfig(StrictModel):
    enabled: bool = True
    host: str = "0.0.0.0"
    port: int = Field(default=8000, ge=1, le=65535)
    stream_fps: float = Field(default=10.0, gt=0, le=30)
    jpeg_quality: int = Field(default=75, ge=30, le=95)
    event_history: int = Field(default=30, ge=5, le=200)
    log_level: Literal["critical", "error", "warning", "info", "debug"] = "warning"
    allow_remote_estop: bool = False
    control_token: str = ""

    @model_validator(mode="after")
    def validate_remote_control(self) -> "WebConfig":
        if self.allow_remote_estop and len(self.control_token) < 12:
            raise ValueError(
                "web.control_token must contain at least 12 characters "
                "when remote E-stop is enabled"
            )
        return self


class LoggingConfig(StrictModel):
    level: str = "INFO"
    file: str = "logs/fruit_sorter.log"
    max_bytes: int = Field(default=5_000_000, ge=10_000)
    backup_count: int = Field(default=5, ge=1, le=50)


class CameraConfig(StrictModel):
    device: int | str = 0
    width: int = Field(default=1280, ge=160)
    height: int = Field(default=720, ge=120)
    fps: int = Field(default=30, ge=1, le=120)
    backend: Literal["V4L2", "ANY"] = "V4L2"
    fourcc: str = Field(default="MJPG", min_length=4, max_length=4)
    warmup_frames: int = Field(default=20, ge=0, le=300)
    frame_timeout_s: float = Field(default=2.0, gt=0)
    reconnect_delay_s: float = Field(default=1.0, gt=0)
    max_consecutive_failures: int = Field(default=10, ge=1)


class DetectorConfig(StrictModel):
    model: str = "yolov8n.pt"
    device: str = "cpu"
    image_size: int = Field(default=640, ge=160)
    confidence: float = Field(default=0.55, ge=0, le=1)
    iou: float = Field(default=0.50, ge=0, le=1)
    tracker: str = "bytetrack.yaml"
    class_ids: dict[FruitType, int]


class TrackingConfig(StrictModel):
    stable_frames: int = Field(default=8, ge=2)
    max_center_jitter_px: float = Field(default=12.0, gt=0)
    stale_after_frames: int = Field(default=30, ge=1)
    min_bbox_area_px: int = Field(default=1200, ge=1)
    revalidate_max_shift_px: float = Field(default=30.0, gt=0)
    picked_memory_seconds: float = Field(default=300.0, gt=0)
    picked_radius_mm: float = Field(default=35.0, gt=0)
    rejected_cooldown_seconds: float = Field(default=5.0, gt=0)


class HSVRange(StrictModel):
    lower: tuple[int, int, int]
    upper: tuple[int, int, int]

    @model_validator(mode="after")
    def validate_hsv(self) -> "HSVRange":
        for label, values, maxima in (
            ("lower", self.lower, (179, 255, 255)),
            ("upper", self.upper, (179, 255, 255)),
        ):
            if any(value < 0 or value > maximum for value, maximum in zip(values, maxima)):
                raise ValueError(f"{label} HSV values are outside OpenCV ranges")
        return self


class FruitColourConfig(StrictModel):
    min_ratio: float = Field(ge=0, le=1)
    ranges: list[HSVRange] = Field(min_length=1)


class ColourConfig(StrictModel):
    central_crop: float = Field(default=0.72, gt=0, le=1)
    morphology_kernel: int = Field(default=5, ge=1, le=31)
    min_pixels: int = Field(default=250, ge=1)
    fruits: dict[FruitType, FruitColourConfig]


class CalibrationConfig(StrictModel):
    calibration_complete: bool = False
    homography: list[list[float]]
    roi_polygon_px: list[tuple[int, int]] = Field(min_length=3)
    robot_reference_points_mm: list[tuple[float, float]] = Field(min_length=4)
    pick_z_mm: dict[FruitType, float]
    pick_offsets_mm: dict[FruitType, tuple[float, float]]

    @model_validator(mode="after")
    def validate_calibration(self) -> "CalibrationConfig":
        if len(self.homography) != 3 or any(len(row) != 3 for row in self.homography):
            raise ValueError("homography must be a 3x3 matrix")
        return self


class ServoConfig(StrictModel):
    name: Literal["base", "shoulder", "elbow", "wrist_pitch", "wrist_roll", "gripper"]
    channel: int = Field(ge=0, le=15)
    min_deg: float = Field(ge=0, le=180)
    max_deg: float = Field(ge=0, le=180)
    min_pulse_us: int = Field(ge=300, le=3000)
    max_pulse_us: int = Field(ge=300, le=3000)
    kinematic_zero_deg: float = Field(ge=0, le=180)
    direction: Literal[-1, 1]

    @model_validator(mode="after")
    def validate_ranges(self) -> "ServoConfig":
        if self.max_deg <= self.min_deg:
            raise ValueError("max_deg must exceed min_deg")
        if self.max_pulse_us <= self.min_pulse_us:
            raise ValueError("max_pulse_us must exceed min_pulse_us")
        return self


class ArmGeometryConfig(StrictModel):
    base_height_mm: float = Field(gt=0)
    upper_arm_mm: float = Field(gt=0)
    forearm_mm: float = Field(gt=0)
    tool_mm: float = Field(ge=0)
    tool_pitch_deg: float = Field(ge=-180, le=180)
    elbow_solution: Literal["positive", "negative"] = "negative"


class GripperConfig(StrictModel):
    open_deg: float = Field(ge=0, le=180)
    closed_deg: float = Field(ge=0, le=180)
    close_dwell_s: float = Field(default=0.5, ge=0)
    release_dwell_s: float = Field(default=0.4, ge=0)


class ArmConfig(StrictModel):
    home_deg: list[float] = Field(min_length=6, max_length=6)
    wrist_roll_joint_deg: float = Field(default=0, ge=-180, le=180)
    geometry: ArmGeometryConfig
    gripper: GripperConfig
    servos: list[ServoConfig] = Field(min_length=6, max_length=6)

    @model_validator(mode="after")
    def validate_servos(self) -> "ArmConfig":
        expected = {"base", "shoulder", "elbow", "wrist_pitch", "wrist_roll", "gripper"}
        names = {servo.name for servo in self.servos}
        if names != expected:
            raise ValueError(f"servos must contain exactly {sorted(expected)}")
        channels = [servo.channel for servo in self.servos]
        if len(channels) != len(set(channels)):
            raise ValueError("servo PCA9685 channels must be unique")
        for angle, servo in zip(self.home_deg, self.servos):
            if not servo.min_deg <= angle <= servo.max_deg:
                raise ValueError(f"home angle {angle} violates {servo.name} limits")
        return self

    def servo_index(self, name: str) -> int:
        for index, servo in enumerate(self.servos):
            if servo.name == name:
                return index
        raise KeyError(name)

    def servo(self, name: str) -> ServoConfig:
        return self.servos[self.servo_index(name)]


class MotionConfig(StrictModel):
    safe_travel_z_mm: float
    approach_clearance_mm: float = Field(gt=0)
    path_step_mm: float = Field(gt=0)
    max_joint_speed_deg_s: float = Field(gt=0)
    min_move_ms: int = Field(ge=50)
    max_move_ms: int = Field(ge=100)
    command_timeout_margin_s: float = Field(gt=0)

    @model_validator(mode="after")
    def validate_move_times(self) -> "MotionConfig":
        if self.max_move_ms < self.min_move_ms:
            raise ValueError("max_move_ms must be >= min_move_ms")
        return self


class KeepoutCylinderConfig(StrictModel):
    name: str
    x_mm: float
    y_mm: float
    radius_mm: float = Field(gt=0)
    z_min_mm: float
    z_max_mm: float


class WorkspaceConfig(StrictModel):
    x_min_mm: float
    x_max_mm: float
    y_min_mm: float
    y_max_mm: float
    z_min_mm: float
    z_max_mm: float
    keepout_cylinders: list[KeepoutCylinderConfig] = []

    @model_validator(mode="after")
    def validate_bounds(self) -> "WorkspaceConfig":
        if not (self.x_min_mm < self.x_max_mm):
            raise ValueError("x workspace bounds are invalid")
        if not (self.y_min_mm < self.y_max_mm):
            raise ValueError("y workspace bounds are invalid")
        if not (self.z_min_mm < self.z_max_mm):
            raise ValueError("z workspace bounds are invalid")
        return self


class BoxConfig(StrictModel):
    x_mm: float
    y_mm: float
    release_z_mm: float


class SafetyConfig(StrictModel):
    rpc_socket: str = "/var/run/arduino-router.sock"
    rpc_timeout_s: float = Field(default=2.0, gt=0)
    connect_timeout_s: float = Field(default=15.0, gt=0)
    heartbeat_interval_s: float = Field(default=0.25, gt=0)
    watchdog_timeout_ms: int = Field(default=2000, ge=500, le=30000)
    shutdown_disables_outputs: bool = True


class AppConfig(StrictModel):
    application: ApplicationConfig
    web: WebConfig = WebConfig()
    logging: LoggingConfig
    camera: CameraConfig
    detector: DetectorConfig
    tracking: TrackingConfig
    colour: ColourConfig
    calibration: CalibrationConfig
    arm: ArmConfig
    motion: MotionConfig
    workspace: WorkspaceConfig
    boxes: dict[FruitType, BoxConfig]
    safety: SafetyConfig

    @model_validator(mode="after")
    def validate_fruit_sections(self) -> "AppConfig":
        required = set(FruitType)
        sections = {
            "detector.class_ids": set(self.detector.class_ids),
            "colour.fruits": set(self.colour.fruits),
            "calibration.pick_z_mm": set(self.calibration.pick_z_mm),
            "calibration.pick_offsets_mm": set(self.calibration.pick_offsets_mm),
            "boxes": set(self.boxes),
        }
        for label, values in sections.items():
            if values != required:
                raise ValueError(f"{label} must define apple, banana, and orange")
        return self


def load_config(path: str | Path) -> AppConfig:
    config_path = Path(path)
    try:
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        return AppConfig.model_validate(raw)
    except Exception as exc:
        raise ConfigurationError(f"Unable to load {config_path}: {exc}") from exc
