#!/usr/bin/env python3
"""Receive real SO-101 state over UDP and publish /joint_states for RViz."""

from __future__ import annotations

import json
import math
import socket
import time

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
JOINTS = [*BODY_JOINTS, "gripper"]
URDF_LIMITS_RAD = {
    "shoulder_pan": [-1.91986, 1.91986],
    "shoulder_lift": [-1.74533, 1.74533],
    "elbow_flex": [-1.69, 1.69],
    "wrist_flex": [-1.65806, 1.65806],
    "wrist_roll": [-2.74385, 2.84121],
    "gripper": [-0.174533, 1.74533],
}
DEFAULT_MAPPING = {
    "joint_flips": dict.fromkeys(JOINTS, False),
    "urdf_offsets_deg": dict.fromkeys(JOINTS, 0.0),
    "urdf_scales": dict.fromkeys(JOINTS, 1.0),
}
DEFAULT_MAPPING["joint_flips"]["elbow_flex"] = False


def clamp(value: float, low: float, high: float) -> float:
    return min(high, max(low, value))


def gripper_percent_to_rad(value: float) -> float:
    value = clamp(value, 0.0, 100.0)
    low, high = URDF_LIMITS_RAD["gripper"]
    return low + (value / 100.0) * (high - low)


def mapping_from_payload(payload: dict) -> dict:
    loaded = payload.get("mapping", {})
    mapping = {
        "joint_flips": DEFAULT_MAPPING["joint_flips"].copy(),
        "urdf_offsets_deg": DEFAULT_MAPPING["urdf_offsets_deg"].copy(),
        "urdf_scales": DEFAULT_MAPPING["urdf_scales"].copy(),
    }
    mapping["joint_flips"].update(loaded.get("joint_flips", {}))
    mapping["urdf_offsets_deg"].update(loaded.get("urdf_offsets_deg", {}))
    mapping["urdf_scales"].update(loaded.get("urdf_scales", {}))
    return mapping


def unwrap_lerobot_value(
    value: float,
    previous_value: float | None,
    lerobot_low: float,
    lerobot_high: float,
    jump_ratio: float,
) -> float:
    if previous_value is None:
        return value

    span = lerobot_high - lerobot_low
    if span <= 0:
        return value

    delta = value - previous_value
    if abs(delta) <= span * jump_ratio:
        return value

    if delta > 0:
        return value - span
    return value + span


def lerobot_deg_to_urdf_rad(
    motor: str,
    value: float,
    limits_deg: dict,
    mapping: dict,
    clip_to_urdf_limits: bool,
) -> float:
    urdf_low, urdf_high = URDF_LIMITS_RAD[motor]
    lerobot_low, lerobot_high = limits_deg[motor]
    if lerobot_high == lerobot_low:
        return 0.0
    ratio = (value - lerobot_low) / (lerobot_high - lerobot_low)
    if clip_to_urdf_limits:
        ratio = clamp(ratio, 0.0, 1.0)
    if mapping["joint_flips"][motor]:
        ratio = 1.0 - ratio
    offset_rad = math.radians(float(mapping["urdf_offsets_deg"][motor]))
    scale = float(mapping["urdf_scales"][motor])
    return urdf_low + ratio * (urdf_high - urdf_low) * scale + offset_rad


def calibrated_limit_pair(motor: str, limits_deg: dict, mapping: dict) -> tuple[float, float]:
    if motor == "gripper":
        low, high = URDF_LIMITS_RAD[motor]
        return float(low), float(high)

    lerobot_low, lerobot_high = limits_deg[motor]
    first = lerobot_deg_to_urdf_rad(motor, float(lerobot_low), limits_deg, mapping, False)
    second = lerobot_deg_to_urdf_rad(motor, float(lerobot_high), limits_deg, mapping, False)
    return min(first, second), max(first, second)


class RealStateUdpReceiver(Node):
    def __init__(self):
        super().__init__("so101_real_state_udp_receiver")
        self.declare_parameter("bind_ip", "0.0.0.0")
        self.declare_parameter("bind_port", 50102)
        self.declare_parameter("stale_timeout", 1.0)
        self.declare_parameter("unwrap_wraparound", True)
        self.declare_parameter("wrap_jump_ratio", 0.75)
        self.declare_parameter("clip_to_urdf_limits", False)

        bind_ip = self.get_parameter("bind_ip").get_parameter_value().string_value
        bind_port = self.get_parameter("bind_port").get_parameter_value().integer_value
        self.stale_timeout = self.get_parameter("stale_timeout").get_parameter_value().double_value
        self.unwrap_wraparound = self.get_parameter("unwrap_wraparound").get_parameter_value().bool_value
        self.wrap_jump_ratio = self.get_parameter("wrap_jump_ratio").get_parameter_value().double_value
        self.clip_to_urdf_limits = self.get_parameter("clip_to_urdf_limits").get_parameter_value().bool_value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((bind_ip, int(bind_port)))
        self.sock.setblocking(False)

        self.publisher = self.create_publisher(JointState, "joint_states", 10)
        self.limits_publisher = self.create_publisher(JointState, "so101_joint_limits", 10)
        self.lerobot_state_publisher = self.create_publisher(JointState, "so101_lerobot_state", 10)
        self.lerobot_limits_publisher = self.create_publisher(JointState, "so101_lerobot_limits", 10)
        self.timer = self.create_timer(1.0 / 60.0, self.poll_udp)
        self.last_rx_t = 0.0
        self.last_seq = None
        self.display_positions = {}
        self.get_logger().info(f"Listening for real SO-101 state on udp://{bind_ip}:{bind_port}")
        self.get_logger().info(
            "Visual wrap handling: "
            f"unwrap_wraparound={self.unwrap_wraparound}, "
            f"clip_to_urdf_limits={self.clip_to_urdf_limits}, "
            f"wrap_jump_ratio={self.wrap_jump_ratio:.2f}"
        )

    def poll_udp(self) -> None:
        latest = None
        while True:
            try:
                payload, _addr = self.sock.recvfrom(65535)
            except BlockingIOError:
                break

            try:
                latest = json.loads(payload.decode("utf-8"))
            except json.JSONDecodeError:
                continue

        if latest is None:
            if self.last_rx_t and time.perf_counter() - self.last_rx_t > self.stale_timeout:
                self.get_logger().warn("No fresh real-state UDP packets", throttle_duration_sec=2.0)
            return

        positions = latest.get("positions", {})
        limits_deg = latest.get("limits_deg", {})
        mapping = mapping_from_payload(latest)
        if not all(joint in positions for joint in JOINTS):
            return
        if not all(joint in limits_deg for joint in JOINTS):
            return

        now = time.perf_counter()

        lerobot_state_msg = JointState()
        lerobot_state_msg.header.stamp = self.get_clock().now().to_msg()
        lerobot_state_msg.name = JOINTS
        lerobot_state_msg.position = [float(positions[joint]) for joint in JOINTS]
        self.lerobot_state_publisher.publish(lerobot_state_msg)

        lerobot_limits_msg = JointState()
        lerobot_limits_msg.header.stamp = lerobot_state_msg.header.stamp
        lerobot_limits_msg.name = JOINTS
        lerobot_limits_msg.position = [float(limits_deg[joint][0]) for joint in JOINTS]
        lerobot_limits_msg.velocity = [float(limits_deg[joint][1]) for joint in JOINTS]
        self.lerobot_limits_publisher.publish(lerobot_limits_msg)

        if self.last_rx_t and now - self.last_rx_t > self.stale_timeout:
            self.display_positions.clear()

        for joint in BODY_JOINTS:
            value = float(positions[joint])
            if self.unwrap_wraparound:
                lerobot_low, lerobot_high = limits_deg[joint]
                value = unwrap_lerobot_value(
                    value,
                    self.display_positions.get(joint),
                    float(lerobot_low),
                    float(lerobot_high),
                    self.wrap_jump_ratio,
                )
            self.display_positions[joint] = value

        msg = JointState()
        msg.header.stamp = lerobot_state_msg.header.stamp
        msg.name = JOINTS
        msg.position = [
            lerobot_deg_to_urdf_rad(
                "shoulder_pan",
                self.display_positions["shoulder_pan"],
                limits_deg,
                mapping,
                self.clip_to_urdf_limits,
            ),
            lerobot_deg_to_urdf_rad(
                "shoulder_lift",
                self.display_positions["shoulder_lift"],
                limits_deg,
                mapping,
                self.clip_to_urdf_limits,
            ),
            lerobot_deg_to_urdf_rad(
                "elbow_flex",
                self.display_positions["elbow_flex"],
                limits_deg,
                mapping,
                self.clip_to_urdf_limits,
            ),
            lerobot_deg_to_urdf_rad(
                "wrist_flex",
                self.display_positions["wrist_flex"],
                limits_deg,
                mapping,
                self.clip_to_urdf_limits,
            ),
            lerobot_deg_to_urdf_rad(
                "wrist_roll",
                self.display_positions["wrist_roll"],
                limits_deg,
                mapping,
                self.clip_to_urdf_limits,
            ),
            gripper_percent_to_rad(float(positions["gripper"])),
        ]
        self.publisher.publish(msg)

        limits_msg = JointState()
        limits_msg.header.stamp = msg.header.stamp
        limits_msg.name = JOINTS
        lower_limits = []
        upper_limits = []
        for joint in JOINTS:
            low, high = calibrated_limit_pair(joint, limits_deg, mapping)
            lower_limits.append(low)
            upper_limits.append(high)
        limits_msg.position = lower_limits
        limits_msg.velocity = upper_limits
        self.limits_publisher.publish(limits_msg)

        self.last_rx_t = now
        seq = latest.get("seq")
        if self.last_seq is None or (seq is not None and int(seq) - int(self.last_seq) >= 50):
            self.get_logger().info(f"Displaying real SO-101 state seq={seq}")
            self.last_seq = seq


def main() -> int:
    rclpy.init()
    node = RealStateUdpReceiver()
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
