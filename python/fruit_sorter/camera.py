"""Low-latency background capture for the Logitech C270."""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional

import cv2
import numpy as np

from .config import CameraConfig
from .exceptions import CameraError

LOG = logging.getLogger(__name__)


class CameraStream:
    """
    Continuously captures frames and retains only the newest frame.

    A dedicated capture thread prevents the main control loop from receiving
    buffered frames after a long robot movement.
    """

    def __init__(self, config: CameraConfig) -> None:
        self.config = config
        self._capture: Optional[cv2.VideoCapture] = None
        self._thread: Optional[threading.Thread] = None
        self._condition = threading.Condition()
        self._latest_frame: Optional[np.ndarray] = None
        self._sequence = 0
        self._stopping = False
        self._fatal_error: Optional[str] = None

    def _backend(self) -> int:
        return cv2.CAP_V4L2 if self.config.backend == "V4L2" else cv2.CAP_ANY

    def _open_capture(self) -> cv2.VideoCapture:
        capture = cv2.VideoCapture(self.config.device, self._backend())
        if not capture.isOpened():
            capture.release()
            raise CameraError(f"Unable to open camera device {self.config.device!r}")

        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        capture.set(cv2.CAP_PROP_FPS, self.config.fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        capture.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*self.config.fourcc),
        )

        actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = capture.get(cv2.CAP_PROP_FPS)
        LOG.info(
            "Camera opened: device=%r resolution=%dx%d fps=%.1f",
            self.config.device,
            actual_width,
            actual_height,
            actual_fps,
        )
        return capture

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stopping = False
        self._fatal_error = None
        self._capture = self._open_capture()

        for _ in range(self.config.warmup_frames):
            ok, _ = self._capture.read()
            if not ok:
                break

        self._thread = threading.Thread(
            target=self._capture_loop,
            name="camera-capture",
            daemon=True,
        )
        self._thread.start()

    def _capture_loop(self) -> None:
        failures = 0
        while not self._stopping:
            capture = self._capture
            if capture is None:
                break

            ok, frame = capture.read()
            if ok and frame is not None and frame.size:
                failures = 0
                with self._condition:
                    self._latest_frame = frame
                    self._sequence += 1
                    self._condition.notify_all()
                continue

            failures += 1
            LOG.warning("Camera read failure %d", failures)
            if failures < self.config.max_consecutive_failures:
                time.sleep(0.05)
                continue

            LOG.error("Camera repeatedly failed; attempting reconnection")
            try:
                capture.release()
                time.sleep(self.config.reconnect_delay_s)
                self._capture = self._open_capture()
                failures = 0
            except Exception as exc:
                self._fatal_error = str(exc)
                with self._condition:
                    self._condition.notify_all()
                time.sleep(self.config.reconnect_delay_s)

    def read(
        self,
        *,
        after_sequence: int | None = None,
        timeout_s: float | None = None,
    ) -> tuple[np.ndarray, int]:
        timeout = timeout_s if timeout_s is not None else self.config.frame_timeout_s
        deadline = time.monotonic() + timeout

        with self._condition:
            while True:
                if self._fatal_error:
                    raise CameraError(self._fatal_error)

                fresh_enough = (
                    self._latest_frame is not None
                    and (after_sequence is None or self._sequence > after_sequence)
                )
                if fresh_enough:
                    return self._latest_frame.copy(), self._sequence

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise CameraError("Timed out waiting for a fresh camera frame")
                self._condition.wait(timeout=remaining)

    def close(self) -> None:
        self._stopping = True
        with self._condition:
            self._condition.notify_all()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._capture:
            self._capture.release()
        self._capture = None
        LOG.info("Camera closed")

    def __enter__(self) -> "CameraStream":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
