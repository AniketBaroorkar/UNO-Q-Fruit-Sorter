"""Pixel-to-robot planar coordinate conversion."""

from __future__ import annotations

import cv2
import numpy as np

from .config import CalibrationConfig
from .exceptions import CalibrationError
from .models import Point2D


class PixelMapper:
    def __init__(self, config: CalibrationConfig) -> None:
        self.config = config
        self._homography = np.asarray(config.homography, dtype=np.float64)
        if self._homography.shape != (3, 3):
            raise CalibrationError("Homography is not 3x3")
        if abs(np.linalg.det(self._homography)) < 1e-12:
            raise CalibrationError("Homography is singular")
        self._roi = np.asarray(config.roi_polygon_px, dtype=np.int32)

    def inside_pick_roi(self, pixel: Point2D) -> bool:
        return (
            cv2.pointPolygonTest(
                self._roi,
                (float(pixel.x), float(pixel.y)),
                False,
            )
            >= 0
        )

    def pixel_to_robot(self, pixel: Point2D) -> Point2D:
        source = np.array([pixel.x, pixel.y, 1.0], dtype=np.float64)
        target = self._homography @ source
        if abs(target[2]) < 1e-12:
            raise CalibrationError("Homography mapped point to infinity")
        return Point2D(float(target[0] / target[2]), float(target[1] / target[2]))

    @property
    def roi_polygon(self) -> np.ndarray:
        return self._roi.copy()
