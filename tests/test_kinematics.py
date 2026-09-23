from pathlib import Path

from fruit_sorter.config import load_config
from fruit_sorter.kinematics import ArmKinematics
from fruit_sorter.models import Point3D


CONFIG = Path(__file__).parents[1] / "config" / "robot.yaml"


def test_representative_pick_is_reachable():
    config = load_config(CONFIG)
    pose = ArmKinematics(config.arm).solve(
        Point3D(180.0, 0.0, 32.0),
        config.arm.gripper.open_deg,
    )
    assert len(pose.angles_deg) == 6
    for value, servo in zip(pose.angles_deg, config.arm.servos):
        assert servo.min_deg <= value <= servo.max_deg
