"""Workspace checks, keep-out zones, and sampled path generation."""

from __future__ import annotations

import math

from .config import MotionConfig, WorkspaceConfig
from .exceptions import SafetyError
from .models import Point3D


class SafetyChecker:
    def __init__(self, workspace: WorkspaceConfig) -> None:
        self.workspace = workspace

    def validate_point(self, point: Point3D) -> None:
        w = self.workspace
        if not w.x_min_mm <= point.x <= w.x_max_mm:
            raise SafetyError(f"X={point.x:.1f} mm is outside workspace")
        if not w.y_min_mm <= point.y <= w.y_max_mm:
            raise SafetyError(f"Y={point.y:.1f} mm is outside workspace")
        if not w.z_min_mm <= point.z <= w.z_max_mm:
            raise SafetyError(f"Z={point.z:.1f} mm is outside workspace")

        for zone in w.keepout_cylinders:
            radial = math.hypot(point.x - zone.x_mm, point.y - zone.y_mm)
            within_height = zone.z_min_mm <= point.z <= zone.z_max_mm
            if radial < zone.radius_mm and within_height:
                raise SafetyError(
                    f"Point enters keep-out zone {zone.name!r}: "
                    f"radius={radial:.1f} mm"
                )

    def line_samples(
        self,
        start: Point3D,
        end: Point3D,
        step_mm: float,
    ) -> list[Point3D]:
        distance = math.sqrt(
            (end.x - start.x) ** 2
            + (end.y - start.y) ** 2
            + (end.z - start.z) ** 2
        )
        segments = max(1, math.ceil(distance / step_mm))
        points = [
            Point3D(
                start.x + (end.x - start.x) * index / segments,
                start.y + (end.y - start.y) * index / segments,
                start.z + (end.z - start.z) * index / segments,
            )
            for index in range(1, segments + 1)
        ]
        for point in points:
            self.validate_point(point)
        return points
