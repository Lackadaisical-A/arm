#!/usr/bin/env python3
"""Forward ROS2 slider JointState messages to a Windows LeRobot UDP bridge."""

from __future__ import annotations

import json
import math
import socket
import subprocess

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


BODY_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
MOTORS = [*BODY_JOINTS, "gripper"]
GRIPPER_LOWER_RAD = -0.174533
GRIPPER_UPPER_RAD = 1.74533


def default_windows_host_ip() -> str:
    try:
        route = subprocess.check_output(["ip", "route"], text=True)
        for line in route.splitlines():
            parts = line.split()
            if len(parts) >= 3 and parts[0] == "default" and parts[1] == "via":
                return parts[2]
    except (OSError, subprocess.SubprocessError):
        pass
    return "127.0.0.1"


def clamp(value: float, low: float, high: float) -> float:
    return min(high, max(low, value))


def gripper_rad_to_percent(value: float) -> float:
    ratio = (value - GRIPPER_LOWER_RAD) / (GRIPPER_UPPER_RAD - GRIPPER_LOWER_RAD)
    return clamp(ratio * 100.0, 0.0, 100.0)


class JointStatesUdpBridge(Node):
    def __init__(self):
        super().__init__("so101_joint_states_udp_bridge")
        self.declare_parameter("target_ip", "")
        self.declare_parameter("target_port", 50101)
        self.declare_parameter("source_topic", "/joint_states")
        self.declare_parameter("input_mode", "urdf")

        target_ip = self.get_parameter("target_ip").get_parameter_value().string_value
        if not target_ip:
            target_ip = default_windows_host_ip()
        target_port = self.get_parameter("target_port").get_parameter_value().integer_value
        source_topic = self.get_parameter("source_topic").get_parameter_value().string_value
        self.input_mode = self.get_parameter("input_mode").get_parameter_value().string_value

        self.target = (target_ip, int(target_port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0
        self.last_log_seq = -1
        self.subscription = self.create_subscription(JointState, source_topic, self.on_joint_state, 10)
        self.get_logger().info(
            f"Forwarding {source_topic} to udp://{target_ip}:{target_port} as {self.input_mode}"
        )

    def on_joint_state(self, msg: JointState) -> None:
        by_name = dict(zip(msg.name, msg.position, strict=False))
        if not all(joint in by_name for joint in MOTORS):
            return

        payload = {
            "seq": self.seq,
            "stamp_sec": int(msg.header.stamp.sec),
            "stamp_nanosec": int(msg.header.stamp.nanosec),
        }
        if self.input_mode == "lerobot":
            payload["positions"] = {joint: float(by_name[joint]) for joint in MOTORS}
        else:
            payload["urdf_positions_rad"] = {joint: float(by_name[joint]) for joint in MOTORS}

        self.sock.sendto(json.dumps(payload).encode("utf-8"), self.target)
        if self.seq == 0 or self.seq - self.last_log_seq >= 50:
            self.get_logger().info(f"Sent RViz target seq={self.seq} to udp://{self.target[0]}:{self.target[1]}")
            self.last_log_seq = self.seq
        self.seq += 1


def main() -> int:
    rclpy.init()
    node = JointStatesUdpBridge()
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
