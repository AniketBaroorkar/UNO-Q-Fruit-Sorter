"""Small immutable data models shared by the vision and motion layers."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import hypot
from typing import Optional


class FruitType(str, Enum):
    APPLE = "apple"
    BANANA = "banana"
    ORANGE = "orange"


class ControllerState(str, Enum):
    STARTING = "starting"
    IDLE = "idle"
    ACQUIRING = "acquiring"
    PICKING = "picking"
    PLACING = "placing"
    HOMING = "homing"
    ESTOP = "emergency_stop"
    STOPPED = "stopped"
    ERROR = "error"


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float

    def distance_to(self, other: "Point2D") -> float:
        return hypot(self.x - other.x, self.y - other.y)


@dataclass(frozen=True)
class Point3D:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class Detection:
    fruit: FruitType
    confidence: float
    bbox_xyxy: tuple[int, int, int, int]
    track_id: Optional[int]

    @property
    def center_px(self) -> Point2D:
        x1, y1, x2, y2 = self.bbox_xyxy
        return Point2D((x1 + x2) / 2.0, (y1 + y2) / 2.0)

    @property
    def area_px(self) -> int:
        x1, y1, x2, y2 = self.bbox_xyxy
        return max(0, x2 - x1) * max(0, y2 - y1)


@dataclass(frozen=True)
class ColourResult:
    accepted: bool
    ratio: float
    matched_pixels: int
    evaluated_pixels: int


@dataclass(frozen=True)
class JointPose:
    """Six logical servo angles in the configured servo-list order."""

    angles_deg: tuple[float, float, float, float, float, float]

    def with_servo(self, index: int, angle_deg: float) -> "JointPose":
        values = list(self.angles_deg)
        values[index] = float(angle_deg)
        return JointPose(tuple(values))  # type: ignore[arg-type]
