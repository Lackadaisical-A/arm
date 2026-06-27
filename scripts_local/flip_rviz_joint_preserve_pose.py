#!/usr/bin/env python
r"""Toggle an RViz visual joint flip while preserving the current displayed pose.

Use this when a joint moves the wrong direction in RViz, but the current/start
pose is already visually correct.

Example:
    python .\scripts_local\flip_rviz_joint_preserve_pose.py --port COM6 --joint elbow_flex
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


BODY_MOTORS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
MOTORS = [*BODY_MOTORS, "gripper"]
URDF_LIMITS_RAD = {
    "shoulder_pan": [-1.91986, 1.91986],
    "shoulder_lift": [-1.74533, 1.74533],
    "elbow_flex": [-1.69, 1.69],
    "wrist_flex": [-1.65806, 1.65806],
    "wrist_roll": [-2.74385, 2.84121],
}


def default_config_path() -> Path:
    return Path(__file__).resolve().parents[1] / "config" / "so101_joint_mapping.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Flip an RViz visual joint direction without moving its pose.")
    parser.add_argument("--port", required=True, help="Robot serial port, for example COM6.")
    parser.add_argument("--robot-id", default="my_so101_follower", help="LeRobot calibration id.")
    parser.add_argument("--joint", choices=BODY_MOTORS, default="elbow_flex")
    parser.add_argument("--config", type=Path, default=default_config_path())
    parser.add_argument("--set", choices=["toggle", "true", "false"], default="toggle")
    return parser.parse_args()


def load_config(path: Path) -> dict:
    config = json.loads(path.read_text(encoding="utf-8"))
    config.setdefault("joint_flips", {})
    config.setdefault("command_flips", {})
    config.setdefault("urdf_offsets_deg", {})
    config.setdefault("urdf_scales", {})
    for motor in MOTORS:
        config["joint_flips"].setdefault(motor, False)
        config["command_flips"].setdefault(motor, False)
        config["urdf_offsets_deg"].setdefault(motor, 0.0)
        config["urdf_scales"].setdefault(motor, 1.0)
    return config


def save_config(path: Path, config: dict) -> None:
    path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")


def lerobot_limits_deg(robot: SO101Follower) -> dict[str, list[float]]:
    limits = {}
    for motor in BODY_MOTORS:
        calibration = robot.calibration[motor]
        model = robot.bus.motors[motor].model
        max_res = robot.bus.model_resolution_table[model] - 1
        mid = (calibration.range_min + calibration.range_max) / 2
        low = (calibration.range_min - mid) * 360 / max_res
        high = (calibration.range_max - mid) * 360 / max_res
        limits[motor] = [float(low), float(high)]
    return limits


def display_angle_deg(motor: str, value: float, limits_deg: dict, config: dict) -> float:
    urdf_low = math.degrees(URDF_LIMITS_RAD[motor][0])
    urdf_high = math.degrees(URDF_LIMITS_RAD[motor][1])
    lerobot_low, lerobot_high = limits_deg[motor]
    if lerobot_high == lerobot_low:
        return 0.0

    ratio = (value - lerobot_low) / (lerobot_high - lerobot_low)
    if config["joint_flips"][motor]:
        ratio = 1.0 - ratio

    scale = float(config["urdf_scales"][motor])
    offset = float(config["urdf_offsets_deg"][motor])
    return urdf_low + ratio * (urdf_high - urdf_low) * scale + offset


def requested_flip(current: bool, requested: str) -> bool:
    if requested == "toggle":
        return not current
    return requested == "true"


def main() -> int:
    args = parse_args()
    config = load_config(args.config)

    robot = SO101Follower(SO101FollowerConfig(port=args.port, id=args.robot_id))
    print(f"Connecting read-only to SO-101 follower on {args.port}...")
    robot.bus.connect()
    try:
        limits_deg = lerobot_limits_deg(robot)
        positions = robot.bus.sync_read("Present_Position")
        current_value = float(positions[args.joint])
    finally:
        robot.bus.disconnect(disable_torque=False)

    old_flip = bool(config["joint_flips"][args.joint])
    new_flip = requested_flip(old_flip, args.set)
    if new_flip == old_flip:
        print(f"{args.joint} visual flip already {new_flip}; no change.")
        return 0

    old_angle = display_angle_deg(args.joint, current_value, limits_deg, config)
    old_offset = float(config["urdf_offsets_deg"][args.joint])

    config["joint_flips"][args.joint] = new_flip
    new_angle_with_old_offset = display_angle_deg(args.joint, current_value, limits_deg, config)
    config["urdf_offsets_deg"][args.joint] = old_offset + (old_angle - new_angle_with_old_offset)
    new_angle = display_angle_deg(args.joint, current_value, limits_deg, config)

    save_config(args.config, config)

    print(f"Updated {args.config}")
    print(f"{args.joint}: visual_flip {old_flip} -> {new_flip}")
    print(f"{args.joint}: offset {old_offset:+.2f} -> {config['urdf_offsets_deg'][args.joint]:+.2f} deg")
    print(f"Preserved current RViz angle: {old_angle:+.2f} -> {new_angle:+.2f} deg")
    print("Restart the Windows publisher if RViz does not reload within a second.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
