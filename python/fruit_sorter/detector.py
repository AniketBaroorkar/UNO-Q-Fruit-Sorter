"""YOLOv8 detection restricted to the three supported COCO fruit classes."""

from __future__ import annotations

import logging

import numpy as np

from .config import DetectorConfig
from .exceptions import DetectionError
from .models import Detection, FruitType

LOG = logging.getLogger(__name__)


class FruitDetector:
    def __init__(self, config: DetectorConfig) -> None:
        self.config = config
        self._model = None
        self._id_to_fruit = {class_id: fruit for fruit, class_id in config.class_ids.items()}

    def load(self) -> None:
        if self._model is not None:
            return
        try:
            from ultralytics import YOLO

            LOG.info("Loading YOLO model %s on %s", self.config.model, self.config.device)
            self._model = YOLO(self.config.model)
            LOG.info("YOLO model loaded")
        except Exception as exc:
            raise DetectionError(f"Unable to load YOLO model: {exc}") from exc

    def detect_and_track(self, frame: np.ndarray) -> list[Detection]:
        if self._model is None:
            self.load()

        try:
            results = self._model.track(
                frame,
                persist=True,
                tracker=self.config.tracker,
                classes=sorted(self._id_to_fruit),
                conf=self.config.confidence,
                iou=self.config.iou,
                imgsz=self.config.image_size,
                device=self.config.device,
                verbose=False,
            )
        except Exception as exc:
            raise DetectionError(f"YOLO tracking failed: {exc}") from exc

        if not results:
            return []

        result = results[0]
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return []

        xyxy = boxes.xyxy.detach().cpu().numpy()
        class_ids = boxes.cls.detach().cpu().numpy().astype(int)
        confidences = boxes.conf.detach().cpu().numpy()

        if boxes.id is not None:
            track_ids: list[int | None] = boxes.id.detach().cpu().numpy().astype(int).tolist()
        else:
            track_ids = [None] * len(xyxy)

        detections: list[Detection] = []
        for bounds, class_id, confidence, track_id in zip(
            xyxy, class_ids, confidences, track_ids
        ):
            fruit = self._id_to_fruit.get(int(class_id))
            if fruit is None:
                continue

            x1, y1, x2, y2 = (int(round(value)) for value in bounds)
            detections.append(
                Detection(
                    fruit=FruitType(fruit),
                    confidence=float(confidence),
                    bbox_xyxy=(x1, y1, x2, y2),
                    track_id=track_id,
                )
            )
        return detections
