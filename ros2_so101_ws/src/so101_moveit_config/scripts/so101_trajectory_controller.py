#!/usr/bin/env python3
"""Tiny FollowJointTrajectory controller for SO-101 MoveIt demos.

This is not a ROS2-control hardware driver. It gives MoveIt something concrete
to execute against: visual-only mode publishes /joint_states, and optional UDP
mode streams executed trajectory waypoints to the Windows LeRobot bridge.
"""

from __future__ import annotations

import json
import socket
import subprocess
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import DisplayTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
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
ARM_JOINTS = MOTORS[:-1]
GRIPPER_JOINTS = ["gripper"]
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


class So101TrajectoryController(Node):
    def __init__(self) -> None:
        super().__init__("so101_trajectory_controller")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("publish_joint_states", True)
        self.declare_parameter("preview_plans", True)
        self.declare_parameter("display_topic", "/display_planned_path")
        self.declare_parameter("stream_udp", False)
        self.declare_parameter("target_ip", "")
        self.declare_parameter("target_port", 50101)
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("time_scale", 1.0)

        joint_state_topic = self.get_parameter("joint_state_topic").get_parameter_value().string_value
        self.publish_joint_states = self.get_parameter("publish_joint_states").get_parameter_value().bool_value
        self.preview_plans = self.get_parameter("preview_plans").get_parameter_value().bool_value
        display_topic = self.get_parameter("display_topic").get_parameter_value().string_value
        self.stream_udp = self.get_parameter("stream_udp").get_parameter_value().bool_value
        target_ip = self.get_parameter("target_ip").get_parameter_value().string_value or default_windows_host_ip()
        target_port = self.get_parameter("target_port").get_parameter_value().integer_value
        self.rate_hz = max(1.0, self.get_parameter("rate_hz").get_parameter_value().double_value)
        self.time_scale = max(0.05, self.get_parameter("time_scale").get_parameter_value().double_value)

        self.current = dict(INITIAL_RAD)
        self.seq = 0
        self.lock = threading.Lock()
        self.target = (target_ip, int(target_port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.publisher = self.create_publisher(JointState, joint_state_topic, 10)

        self.arm_action = ActionServer(
            self,
            FollowJointTrajectory,
            "/so101_arm_controller/follow_joint_trajectory",
            execute_callback=self.execute_arm,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.gripper_action = ActionServer(
            self,
            FollowJointTrajectory,
            "/so101_gripper_controller/follow_joint_trajectory",
            execute_callback=self.execute_gripper,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        if self.preview_plans:
            self.create_subscription(DisplayTrajectory, display_topic, self.on_display_trajectory, 10)
            self.get_logger().info(f"Previewing planned paths from {display_topic}")

        self.timer = self.create_timer(1.0 / self.rate_hz, self.publish_joint_state)
        self.get_logger().info("FollowJointTrajectory action servers ready:")
        self.get_logger().info("  /so101_arm_controller/follow_joint_trajectory")
        self.get_logger().info("  /so101_gripper_controller/follow_joint_trajectory")
        if self.publish_joint_states:
            self.get_logger().info(f"Publishing SO-101 pose on {joint_state_topic}")
        if self.stream_udp:
            self.get_logger().warn(
                f"Execute will stream URDF targets to udp://{target_ip}:{target_port}. "
                "The Windows bridge must use --accept-urdf-targets and be enabled with e."
            )

    def goal_callback(self, _goal_request) -> GoalResponse:
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def publish_joint_state(self) -> None:
        if not self.publish_joint_states:
            return
        with self.lock:
            positions = [float(self.current[joint]) for joint in MOTORS]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = MOTORS
        msg.position = positions
        self.publisher.publish(msg)

    def send_udp(self) -> None:
        if not self.stream_udp:
            return
        now = self.get_clock().now().to_msg()
        with self.lock:
            positions = {joint: float(self.current[joint]) for joint in MOTORS}
        payload = {
            "seq": self.seq,
            "stamp_sec": int(now.sec),
            "stamp_nanosec": int(now.nanosec),
            "urdf_positions_rad": positions,
        }
        self.sock.sendto(json.dumps(payload).encode("utf-8"), self.target)
        self.seq += 1

    def apply_positions(self, names: list[str], positions: list[float], publish_udp: bool = True) -> None:
        with self.lock:
            for name, position in zip(names, positions, strict=False):
                if name in self.current:
                    self.current[name] = float(position)
        self.publish_joint_state()
        if publish_udp:
            self.send_udp()

    def on_display_trajectory(self, msg: DisplayTrajectory) -> None:
        if not msg.trajectory:
            return
        trajectory = msg.trajectory[-1].joint_trajectory
        if not trajectory.joint_names or not trajectory.points:
            return
        self.get_logger().info(f"Previewing planned path with {len(trajectory.points)} points.")
        thread = threading.Thread(target=self.preview_trajectory, args=(trajectory,), daemon=True)
        thread.start()

    def preview_trajectory(self, trajectory) -> None:
        start_t = time.perf_counter()
        last_t = 0.0
        for point in trajectory.points:
            target_t = duration_sec(point.time_from_start) / self.time_scale
            delay = target_t - last_t
            if delay > 0:
                time.sleep(delay)
            self.apply_positions(trajectory.joint_names, list(point.positions), publish_udp=False)
            last_t = target_t
        self.get_logger().debug(f"Preview replay finished in {time.perf_counter() - start_t:.2f}s")

    def execute_arm(self, goal_handle):
        return self.execute_trajectory(goal_handle, ARM_JOINTS)

    def execute_gripper(self, goal_handle):
        return self.execute_trajectory(goal_handle, GRIPPER_JOINTS)

    def execute_trajectory(self, goal_handle, expected_joints: list[str]):
        trajectory = goal_handle.request.trajectory
        result = FollowJointTrajectory.Result()

        if not trajectory.points:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "Trajectory has no points."
            goal_handle.abort()
            return result

        if not set(trajectory.joint_names).issubset(set(expected_joints)):
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = (
                f"Controller expected joints within {expected_joints}, "
                f"got {list(trajectory.joint_names)}."
            )
            goal_handle.abort()
            return result

        self.get_logger().info(
            f"Executing {len(trajectory.points)} points for joints {list(trajectory.joint_names)}"
        )
        last_t = 0.0
        for point in trajectory.points:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "Canceled."
                return result

            target_t = duration_sec(point.time_from_start) / self.time_scale
            delay = target_t - last_t
            if delay > 0:
                time.sleep(delay)
            self.apply_positions(trajectory.joint_names, list(point.positions), publish_udp=True)

            feedback = FollowJointTrajectory.Feedback()
            feedback.header.stamp = self.get_clock().now().to_msg()
            feedback.joint_names = list(trajectory.joint_names)
            feedback.desired = point
            feedback.actual = point
            goal_handle.publish_feedback(feedback)
            last_t = target_t

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "Executed by so101_trajectory_controller."
        self.get_logger().info("Trajectory execution finished.")
        return result


def main() -> int:
    rclpy.init()
    node = So101TrajectoryController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
