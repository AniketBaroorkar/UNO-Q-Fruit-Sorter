"""Offline validation of YAML structure, key waypoints, and inverse kinematics."""

from __future__ import annotations

import argparse
from pathlib import Path

from .config import load_config
from .kinematics import ArmKinematics
from .models import FruitType, Point3D
from .safety import SafetyChecker


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Validate fruit sorter config")
    parser.add_argument("--config", default="config/robot.yaml")
    return parser


def cli() -> None:
    args = build_parser().parse_args()
    config = load_config(Path(args.config))
    ik = ArmKinematics(config.arm)
    safety = SafetyChecker(config.workspace)

    checked = 0
    for fruit in FruitType:
        z = config.calibration.pick_z_mm[fruit]
        # Validate a representative centre point in the configured workspace.
        centre = Point3D(
            (config.workspace.x_min_mm + config.workspace.x_max_mm) / 2,
            0.0,
            z,
        )
        safety.validate_point(centre)
        ik.solve(centre, config.arm.gripper.open_deg)
        checked += 1

        box = config.boxes[fruit]
        release = Point3D(box.x_mm, box.y_mm, box.release_z_mm)
        safe = Point3D(box.x_mm, box.y_mm, config.motion.safe_travel_z_mm)
        for point in (release, safe):
            safety.validate_point(point)
            ik.solve(point, config.arm.gripper.open_deg)
            checked += 1

    print(f"Configuration valid: {args.config}")
    print(f"Validated {checked} representative IK waypoints")
    print(
        "Automatic motion:",
        "ENABLED" if config.calibration.calibration_complete else "LOCKED",
    )


if __name__ == "__main__":
    cli()
