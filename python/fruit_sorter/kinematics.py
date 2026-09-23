"""Analytic inverse kinematics for base + shoulder + elbow + wrist pitch."""

from __future__ import annotations

import math

from .config import ArmConfig
from .exceptions import KinematicsError
from .models import JointPose, Point3D


class ArmKinematics:
    """
    Solves a common educational 6-servo arm layout:

    0. Base yaw
    1. Shoulder pitch
    2. Elbow pitch
    3. Wrist pitch
    4. Wrist roll
    5. Gripper

    The position solver uses the first four joints. Wrist roll is held at a
    configured angle and the gripper is controlled independently.
    """

    def __init__(self, config: ArmConfig) -> None:
        self.config = config

    def _servo_angle(self, servo_name: str, joint_angle_deg: float) -> float:
        servo = self.config.servo(servo_name)
        angle = servo.kinematic_zero_deg + servo.direction * joint_angle_deg
        if not servo.min_deg <= angle <= servo.max_deg:
            raise KinematicsError(
                f"{servo_name} command {angle:.1f}° violates "
                f"[{servo.min_deg:.1f}, {servo.max_deg:.1f}]°"
            )
        return angle

    def solve(self, target: Point3D, gripper_deg: float) -> JointPose:
        geometry = self.config.geometry

        base_joint = math.degrees(math.atan2(target.y, target.x))
        radial = math.hypot(target.x, target.y)
        z_from_shoulder = target.z - geometry.base_height_mm

        tool_pitch = math.radians(geometry.tool_pitch_deg)

        # Remove the tool vector to obtain the wrist-centre target.
        wrist_r = radial - geometry.tool_mm * math.cos(tool_pitch)
        wrist_z = z_from_shoulder - geometry.tool_mm * math.sin(tool_pitch)

        l1 = geometry.upper_arm_mm
        l2 = geometry.forearm_mm
        distance_sq = wrist_r * wrist_r + wrist_z * wrist_z

        cosine_elbow = (distance_sq - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
        if cosine_elbow < -1.000001 or cosine_elbow > 1.000001:
            raise KinematicsError(
                f"Target ({target.x:.1f}, {target.y:.1f}, {target.z:.1f}) mm "
                "is outside the arm reach"
            )
        cosine_elbow = max(-1.0, min(1.0, cosine_elbow))

        elbow = math.acos(cosine_elbow)
        if geometry.elbow_solution == "negative":
            elbow = -elbow

        shoulder = math.atan2(wrist_z, wrist_r) - math.atan2(
            l2 * math.sin(elbow),
            l1 + l2 * math.cos(elbow),
        )
        wrist = tool_pitch - shoulder - elbow

        joint_angles = {
            "base": base_joint,
            "shoulder": math.degrees(shoulder),
            "elbow": math.degrees(elbow),
            "wrist_pitch": math.degrees(wrist),
            "wrist_roll": self.config.wrist_roll_joint_deg,
        }

        result = [0.0] * 6
        for name, joint_angle in joint_angles.items():
            result[self.config.servo_index(name)] = self._servo_angle(name, joint_angle)

        gripper = self.config.servo("gripper")
        if not gripper.min_deg <= gripper_deg <= gripper.max_deg:
            raise KinematicsError(
                f"Gripper command {gripper_deg:.1f}° violates configured limits"
            )
        result[self.config.servo_index("gripper")] = gripper_deg

        return JointPose(tuple(result))  # type: ignore[arg-type]
