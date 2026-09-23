"""HSV colour validation inside a YOLO bounding box."""

from __future__ import annotations

import cv2
import numpy as np

from .config import ColourConfig
from .models import ColourResult, Detection


class ColourVerifier:
    def __init__(self, config: ColourConfig) -> None:
        self.config = config

    def verify(self, frame: np.ndarray, detection: Detection) -> ColourResult:
        frame_h, frame_w = frame.shape[:2]
        x1, y1, x2, y2 = detection.bbox_xyxy

        x1 = max(0, min(frame_w - 1, x1))
        x2 = max(0, min(frame_w, x2))
        y1 = max(0, min(frame_h - 1, y1))
        y2 = max(0, min(frame_h, y2))

        if x2 <= x1 or y2 <= y1:
            return ColourResult(False, 0.0, 0, 0)

        crop = frame[y1:y2, x1:x2]
        crop_h, crop_w = crop.shape[:2]

        # Restrict evaluation to the middle of the YOLO box. This reduces
        # background colour leakage where the box is not tightly fitted.
        factor = self.config.central_crop
        margin_x = int((1.0 - factor) * crop_w / 2.0)
        margin_y = int((1.0 - factor) * crop_h / 2.0)
        central = crop[
            margin_y : max(margin_y + 1, crop_h - margin_y),
            margin_x : max(margin_x + 1, crop_w - margin_x),
        ]
        if central.size == 0:
            return ColourResult(False, 0.0, 0, 0)

        hsv = cv2.cvtColor(central, cv2.COLOR_BGR2HSV)

        # An ellipse further suppresses the bounding-box corners, which usually
        # contain table or conveyor background rather than fruit.
        ellipse = np.zeros(hsv.shape[:2], dtype=np.uint8)
        h, w = ellipse.shape
        cv2.ellipse(
            ellipse,
            center=(w // 2, h // 2),
            axes=(max(1, int(w * 0.47)), max(1, int(h * 0.47))),
            angle=0,
            startAngle=0,
            endAngle=360,
            color=255,
            thickness=-1,
        )

        fruit_config = self.config.fruits[detection.fruit]
        combined = np.zeros(hsv.shape[:2], dtype=np.uint8)

        for hsv_range in fruit_config.ranges:
            lower = np.array(hsv_range.lower, dtype=np.uint8)
            upper = np.array(hsv_range.upper, dtype=np.uint8)
            combined = cv2.bitwise_or(combined, cv2.inRange(hsv, lower, upper))

        combined = cv2.bitwise_and(combined, ellipse)

        kernel_size = self.config.morphology_kernel
        if kernel_size > 1:
            if kernel_size % 2 == 0:
                kernel_size += 1
            kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
            combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel)
            combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)

        evaluated = int(cv2.countNonZero(ellipse))
        matched = int(cv2.countNonZero(combined))
        ratio = matched / evaluated if evaluated else 0.0
        accepted = evaluated >= self.config.min_pixels and ratio >= fruit_config.min_ratio

        return ColourResult(accepted, ratio, matched, evaluated)
