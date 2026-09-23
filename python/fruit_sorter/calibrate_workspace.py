"""Interactive four-point camera-to-robot homography calibration."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml

from .config import load_config


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Calibrate camera homography")
    parser.add_argument("--config", default="config/robot.yaml")
    parser.add_argument(
        "--mark-complete",
        action="store_true",
        help="Also set calibration_complete=true after writing the homography",
    )
    return parser


def cli() -> None:
    args = build_parser().parse_args()
    config_path = Path(args.config)
    config = load_config(config_path)

    capture = cv2.VideoCapture(config.camera.device, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, config.camera.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config.camera.height)
    if not capture.isOpened():
        raise SystemExit(f"Unable to open camera {config.camera.device!r}")

    for _ in range(config.camera.warmup_frames):
        capture.read()
    ok, frame = capture.read()
    capture.release()
    if not ok or frame is None:
        raise SystemExit("Unable to capture calibration frame")

    reference_points = config.calibration.robot_reference_points_mm[:4]
    clicked: list[tuple[float, float]] = []
    window = "Click calibration marks in displayed order"

    def on_mouse(event, x, y, _flags, _userdata) -> None:
        if event == cv2.EVENT_LBUTTONDOWN and len(clicked) < 4:
            clicked.append((float(x), float(y)))

    cv2.namedWindow(window)
    cv2.setMouseCallback(window, on_mouse)

    while len(clicked) < 4:
        display = frame.copy()
        for index, point in enumerate(clicked):
            cv2.circle(display, (int(point[0]), int(point[1])), 7, (0, 255, 0), -1)
            cv2.putText(
                display,
                str(index + 1),
                (int(point[0]) + 8, int(point[1]) - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

        next_index = len(clicked)
        target = reference_points[next_index]
        instruction = (
            f"Click point {next_index + 1}/4 corresponding to "
            f"robot ({target[0]:.1f}, {target[1]:.1f}) mm | ESC cancels"
        )
        cv2.putText(
            display,
            instruction,
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )
        cv2.imshow(window, display)
        if cv2.waitKey(20) & 0xFF == 27:
            cv2.destroyAllWindows()
            raise SystemExit("Calibration cancelled")

    cv2.destroyAllWindows()

    image_points = np.asarray(clicked, dtype=np.float32)
    robot_points = np.asarray(reference_points, dtype=np.float32)
    homography = cv2.getPerspectiveTransform(image_points, robot_points)

    raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    raw["calibration"]["homography"] = homography.tolist()
    if args.mark_complete:
        raw["calibration"]["calibration_complete"] = True
    config_path.write_text(
        yaml.safe_dump(raw, sort_keys=False),
        encoding="utf-8",
    )

    print(f"Updated homography in {config_path}")
    print(np.array2string(homography, precision=8))
    if not args.mark_complete:
        print(
            "calibration_complete remains false. Validate arm geometry, servo "
            "limits, box positions, and dry-run behaviour before enabling it."
        )


if __name__ == "__main__":
    cli()
