#!/usr/bin/env python3
"""Replay MoveIt DisplayTrajectory messages as joint states and optional UDP targets."""

from __future__ import annotations

import json
import socket
import subprocess
import time

import rclpy
from moveit_msgs.msg import DisplayTrajectory
from rclpy.node import Node
from sensor_msgs.msg import JointState


MOTORS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
INITIAL_RAD = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -0.35,
    "elbow_flex": 0.75,
    "wrist_flex": -0.35,
    "wrist_roll": 0.0,
    "gripper": 0.0,
}


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


def duration_sec(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


class DisplayTrajectoryUdpBridge(Node):
    def __init__(self) -> None:
        super().__init__("so101_display_trajectory_udp_bridge")
        self.declare_parameter("display_topic", "/display_planned_path")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("publish_joint_states", True)
        self.declare_parameter("stream_udp", False)
        self.declare_parameter("target_ip", "")
        self.declare_parameter("target_port", 50101)
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("time_scale", 1.0)

        display_topic = self.get_parameter("display_topic").get_parameter_value().string_value
        joint_state_topic = self.get_parameter("joint_state_topic").get_parameter_value().string_value
        self.publish_joint_states = self.get_parameter("publish_joint_states").get_parameter_value().bool_value
        self.stream_udp = self.get_parameter("stream_udp").get_parameter_value().bool_value
        target_ip = self.get_parameter("target_ip").get_parameter_value().string_value
        if not target_ip:
            target_ip = default_windows_host_ip()
        target_port = self.get_parameter("target_port").get_parameter_value().integer_value
        self.rate_hz = max(1.0, self.get_parameter("rate_hz").get_parameter_value().double_value)
        self.time_scale = max(0.05, self.get_parameter("time_scale").get_parameter_value().double_value)

        self.current = dict(INITIAL_RAD)
        self.playback: dict | None = None
        self.seq = 0
        self.target = (target_ip, int(target_port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.publisher = self.create_publisher(JointState, joint_state_topic, 10)
        self.create_subscription(DisplayTrajectory, display_topic, self.on_display_trajectory, 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

        self.get_logger().info(f"Listening for MoveIt plans on {display_topic}")
        if self.publish_joint_states:
            self.get_logger().info(f"Publishing replayed joint states on {joint_state_topic}")
        if self.stream_udp:
            self.get_logger().warn(
                f"Streaming planned trajectory UDP to {target_ip}:{target_port}. "
                "Windows bridge must be started with --accept-urdf-targets and enabled with e."
            )

    def on_display_trajectory(self, msg: DisplayTrajectory) -> None:
        if not msg.trajectory:
            return
        trajectory = msg.trajectory[-1].joint_trajectory
        if not trajectory.joint_names or not trajectory.points:
            return

        start_state = msg.trajectory_start.joint_state
        for name, pos in zip(start_state.name, start_state.position, strict=False):
            if name in self.current:
                self.current[name] = float(pos)

        points = []
        for point in trajectory.points:
            if len(point.positions) < len(trajectory.joint_names):
                continue
            positions = dict(self.current)
            for name, position in zip(trajectory.joint_names, point.positions, strict=False):
                if name in positions:
                    positions[name] = float(position)
            points.append((duration_sec(point.time_from_start), positions))

        if not points:
            return

        self.playback = {
            "start_wall": time.perf_counter(),
            "points": points,
        }
        self.get_logger().info(f"Replaying MoveIt plan with {len(points)} points.")

    def interpolate_current(self) -> None:
        if self.playback is None:
            return

        elapsed = (time.perf_counter() - self.playback["start_wall"]) * self.time_scale
        points = self.playback["points"]
        if elapsed >= points[-1][0]:
            self.current = dict(points[-1][1])
            self.playback = None
            return

        previous_t, previous_pos = points[0]
        for next_t, next_pos in points[1:]:
            if elapsed <= next_t:
                span = max(1e-6, next_t - previous_t)
                ratio = max(0.0, min(1.0, (elapsed - previous_t) / span))
                self.current = {
                    joint: previous_pos[joint] + ratio * (next_pos[joint] - previous_pos[joint])
                    for joint in MOTORS
                }
                return
            previous_t, previous_pos = next_t, next_pos

        self.current = dict(points[-1][1])

    def publish_joint_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = MOTORS
        msg.position = [float(self.current[joint]) for joint in MOTORS]
        self.publisher.publish(msg)

    def send_udp(self) -> None:
        now = self.get_clock().now().to_msg()
        payload = {
            "seq": self.seq,
            "stamp_sec": int(now.sec),
            "stamp_nanosec": int(now.nanosec),
            "urdf_positions_rad": {joint: float(self.current[joint]) for joint in MOTORS},
        }
        self.sock.sendto(json.dumps(payload).encode("utf-8"), self.target)
        self.seq += 1

    def on_timer(self) -> None:
        self.interpolate_current()
        if self.publish_joint_states:
            self.publish_joint_state()
        if self.stream_udp and self.playback is not None:
            self.send_udp()


def main() -> int:
    rclpy.init()
    node = DisplayTrajectoryUdpBridge()
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
