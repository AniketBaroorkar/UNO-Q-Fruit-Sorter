"""Track stability and duplicate-pick suppression."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math
import time

from .config import TrackingConfig
from .models import Detection, FruitType, Point2D


@dataclass
class _TrackState:
    fruit: FruitType
    centers: deque[Point2D]
    last_frame: int


class StabilityTracker:
    """Requires several consecutive, low-jitter observations before picking."""

    def __init__(self, config: TrackingConfig) -> None:
        self.config = config
        self._tracks: dict[int, _TrackState] = {}

    def update(self, detections: list[Detection], frame_number: int) -> None:
        for detection in detections:
            if detection.track_id is None:
                continue

            state = self._tracks.get(detection.track_id)
            if (
                state is None
                or state.fruit != detection.fruit
                or frame_number - state.last_frame > 1
            ):
                state = _TrackState(
                    fruit=detection.fruit,
                    centers=deque(maxlen=self.config.stable_frames),
                    last_frame=frame_number,
                )
                self._tracks[detection.track_id] = state

            state.centers.append(detection.center_px)
            state.last_frame = frame_number

        stale = [
            track_id
            for track_id, state in self._tracks.items()
            if frame_number - state.last_frame > self.config.stale_after_frames
        ]
        for track_id in stale:
            del self._tracks[track_id]

    def is_stable(self, detection: Detection) -> bool:
        if detection.track_id is None:
            return False
        state = self._tracks.get(detection.track_id)
        if state is None or len(state.centers) < self.config.stable_frames:
            return False

        mean_x = sum(point.x for point in state.centers) / len(state.centers)
        mean_y = sum(point.y for point in state.centers) / len(state.centers)
        maximum = max(
            math.hypot(point.x - mean_x, point.y - mean_y)
            for point in state.centers
        )
        return maximum <= self.config.max_center_jitter_px


@dataclass(frozen=True)
class _PickedRecord:
    fruit: FruitType
    point: Point2D
    expires_at: float


class PickedRegistry:
    """
    Remembers both tracker IDs and robot-frame positions.

    Position memory remains effective if ByteTrack allocates a new ID after the
    arm temporarily occludes the scene.
    """

    def __init__(self, config: TrackingConfig) -> None:
        self.config = config
        self._track_expiry: dict[int, float] = {}
        self._records: list[_PickedRecord] = []

    def _purge(self) -> None:
        now = time.monotonic()
        self._track_expiry = {
            track_id: expiry
            for track_id, expiry in self._track_expiry.items()
            if expiry > now
        }
        self._records = [record for record in self._records if record.expires_at > now]

    def contains(
        self,
        *,
        track_id: int | None,
        fruit: FruitType,
        point: Point2D,
    ) -> bool:
        self._purge()
        if track_id is not None and track_id in self._track_expiry:
            return True

        return any(
            record.fruit == fruit
            and record.point.distance_to(point) <= self.config.picked_radius_mm
            for record in self._records
        )

    def mark(
        self,
        *,
        track_id: int | None,
        fruit: FruitType,
        point: Point2D,
    ) -> None:
        expiry = time.monotonic() + self.config.picked_memory_seconds
        if track_id is not None:
            self._track_expiry[track_id] = expiry
        self._records.append(_PickedRecord(fruit, point, expiry))
