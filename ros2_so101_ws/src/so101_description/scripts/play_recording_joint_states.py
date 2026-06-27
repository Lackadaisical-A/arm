#!/usr/bin/env python3
"""Publish a saved manual SO-101 JSON recording to /joint_states for RViz."""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
BODY_JOINTS = JOINTS[:-1]
URDF_LIMITS_RAD = {
    "shoulder_pan": [-1.91986, 1.91986],
    "shoulder_lift": [-1.74533, 1.74533],
    "elbow_flex": [-1.69, 1.69],
    "wrist_flex": [-1.65806, 1.65806],
    "wrist_roll": [-2.74385, 2.84121],
    "gripper": [-0.174533, 1.74533],
}
JOINT_FLIPS = {
    "shoulder_pan": False,
    "shoulder_lift": False,
    "elbow_flex": False,
    "wrist_flex": False,
    "wrist_roll": False,
    "gripper": False,
}


def clamp(value: float, low: float, high: float) -> float:
    return min(high, max(low, value))


def lerobot_deg_to_urdf_rad(motor: str, value: float, limits_deg: dict | None) -> float:
    if limits_deg and motor in limits_deg:
        lerobot_low, lerobot_high = limits_deg[motor]
    else:
        # Fallback for older recordings that do not include calibrated limits.
        lerobot_low, lerobot_high = -180.0, 180.0

    urdf_low, urdf_high = URDF_LIMITS_RAD[motor]
    if lerobot_high == lerobot_low:
        return 0.0
    ratio = clamp((value - lerobot_low) / (lerobot_high - lerobot_low), 0.0, 1.0)
    if JOINT_FLIPS[motor]:
        ratio = 1.0 - ratio
    return urdf_low + ratio * (urdf_high - urdf_low)


def gripper_percent_to_rad(value: float) -> float:
    low, high = URDF_LIMITS_RAD["gripper"]
    return low + (clamp(value, 0.0, 100.0) / 100.0) * (high - low)


class RecordingPublisher(Node):
    def __init__(self, recording_path: Path):
        super().__init__("so101_recording_joint_state_publisher")
        self.publisher = self.create_publisher(JointState, "joint_states", 10)
        payload = json.loads(recording_path.read_text(encoding="utf-8"))
        self.frames = payload["frames"]
        self.fps = float(payload.get("fps", 30))
        self.limits_deg = payload.get("limits_deg")
        self.index = 0
        self.timer = self.create_timer(1.0 / self.fps, self.publish_next)
        self.get_logger().info(f"Loaded {len(self.frames)} frames from {recording_path}")

    def publish_next(self) -> None:
        if self.index >= len(self.frames):
            self.index = 0

        positions = self.frames[self.index]["positions"]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS

        msg.position = [
            lerobot_deg_to_urdf_rad("shoulder_pan", float(positions["shoulder_pan"]), self.limits_deg),
            lerobot_deg_to_urdf_rad("shoulder_lift", float(positions["shoulder_lift"]), self.limits_deg),
            lerobot_deg_to_urdf_rad("elbow_flex", float(positions["elbow_flex"]), self.limits_deg),
            lerobot_deg_to_urdf_rad("wrist_flex", float(positions["wrist_flex"]), self.limits_deg),
            lerobot_deg_to_urdf_rad("wrist_roll", float(positions["wrist_roll"]), self.limits_deg),
            gripper_percent_to_rad(float(positions["gripper"])),
        ]
        self.publisher.publish(msg)
        self.index += 1


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: ros2 run so101_description play_recording_joint_states.py <recording.json>")
        return 2

    recording_path = Path(sys.argv[1]).expanduser().resolve()
    rclpy.init()
    node = RecordingPublisher(recording_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
