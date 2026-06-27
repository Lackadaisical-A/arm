#!/usr/bin/env python
r"""Small terminal keyboard teleop for a calibrated SO-101 follower arm.

Run from PowerShell after activating the LeRobot venv:
    python .\scripts_local\keyboard_teleop_so101.py --port COM4
"""

from __future__ import annotations

import argparse
import json
import logging
import msvcrt
import time
from pathlib import Path

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from so101_phase_utils import add_elbow_phase_arg, apply_elbow_phase


BODY_MOTORS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
MOTORS = [*BODY_MOTORS, "gripper"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Keyboard teleop for SO-101 follower.")
    parser.add_argument("--port", required=True, help="Robot serial port, for example COM4.")
    parser.add_argument("--robot-id", default="my_so101_follower", help="LeRobot calibration id.")
    parser.add_argument("--step-deg", type=float, default=1.0, help="Step for arm joints, in degrees.")
    parser.add_argument("--roll-step-deg", type=float, default=5.0, help="Step for wrist roll, in degrees.")
    parser.add_argument("--gripper-step", type=float, default=2.0, help="Step for gripper, in 0-100 units.")
    parser.add_argument(
        "--max-relative-target",
        type=float,
        default=4.0,
        help="LeRobot safety cap per command. Lower is safer, higher is snappier.",
    )
    parser.add_argument(
        "--mapping-config",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config" / "so101_joint_mapping.json",
        help="Joint mapping config containing command_flips.",
    )
    add_elbow_phase_arg(parser)
    return parser.parse_args()


def print_help() -> None:
    print(
        """
SO-101 keyboard teleop

Keep this PowerShell window focused. Each keypress sends one small jog.

  a / d  shoulder_pan    - / +
  w / s  shoulder_lift   + / -
  u / o  elbow_flex      + / -
  i / k  wrist_flex      + / -
  j / l  wrist_roll      - / +
  z / x  gripper         - / +

  r      refresh current position from robot
  p      print current position
  h/?    show this help
  q/ESC  quit and disable torque

If a direction feels backwards, use the opposite key. Start with the arm clear
of the table and your hand near the power switch.
"""
    )


def obs_to_target(obs: dict[str, float]) -> dict[str, float]:
    return {motor: float(obs[f"{motor}.pos"]) for motor in MOTORS}


def target_to_action(target: dict[str, float]) -> dict[str, float]:
    return {f"{motor}.pos": value for motor, value in target.items()}


def joint_action(motor: str, value: float) -> dict[str, float]:
    return {f"{motor}.pos": value}


def clamp_target(robot: SO101Follower, motor: str, value: float) -> float:
    if motor == "gripper":
        return min(100.0, max(0.0, value))

    # Body motors are normalized to degrees. Clamp to the calibrated physical
    # range so a typo cannot request a target beyond the recorded limits.
    calibration = robot.bus.calibration[motor]
    model = robot.bus.motors[motor].model
    max_res = robot.bus.model_resolution_table[model] - 1
    mid = (calibration.range_min + calibration.range_max) / 2
    low = (calibration.range_min - mid) * 360 / max_res
    high = (calibration.range_max - mid) * 360 / max_res
    return min(high, max(low, value))


def format_positions(target: dict[str, float]) -> str:
    return " | ".join(f"{motor}:{target[motor]:7.2f}" for motor in MOTORS)


def read_key() -> str:
    key = msvcrt.getwch()
    if key in ("\x00", "\xe0"):
        # Extended keys, such as arrows, arrive as a two-character sequence.
        _ = msvcrt.getwch()
        return ""
    return key


def load_command_flips(path: Path) -> dict[str, bool]:
    flips = dict.fromkeys(MOTORS, False)
    try:
        loaded = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return flips
    flips.update(loaded.get("command_flips", {}))
    return flips


def main() -> int:
    args = parse_args()
    logging.getLogger().setLevel(logging.ERROR)

    robot = SO101Follower(
        SO101FollowerConfig(
            port=args.port,
            id=args.robot_id,
            max_relative_target={motor: args.max_relative_target for motor in MOTORS},
        )
    )

    keymap = {
        "a": ("shoulder_pan", -args.step_deg),
        "d": ("shoulder_pan", args.step_deg),
        "w": ("shoulder_lift", args.step_deg),
        "s": ("shoulder_lift", -args.step_deg),
        "u": ("elbow_flex", args.step_deg),
        "o": ("elbow_flex", -args.step_deg),
        "i": ("wrist_flex", args.step_deg),
        "k": ("wrist_flex", -args.step_deg),
        "j": ("wrist_roll", -args.roll_step_deg),
        "l": ("wrist_roll", args.roll_step_deg),
        "z": ("gripper", -args.gripper_step),
        "x": ("gripper", args.gripper_step),
    }
    command_flips = load_command_flips(args.mapping_config)

    print(f"Connecting to SO-101 follower on {args.port} with id '{args.robot_id}'...")
    robot.connect()
    apply_elbow_phase(robot, args.elbow_phase)

    try:
        target = obs_to_target(robot.get_observation())
        print_help()
        print("Current:", format_positions(target))

        while True:
            key = read_key().lower()
            if not key:
                continue

            if key in ("q", "\x1b"):
                print("\nQuitting.")
                break

            if key in ("h", "?"):
                print_help()
                continue

            if key == "r":
                target = obs_to_target(robot.get_observation())
                print("Refreshed:", format_positions(target))
                continue

            if key == "p":
                target = obs_to_target(robot.get_observation())
                print("Current:", format_positions(target))
                continue

            if key not in keymap:
                continue

            motor, delta = keymap[key]
            if command_flips[motor]:
                delta = -delta
            before = obs_to_target(robot.get_observation())
            goal = clamp_target(robot, motor, before[motor] + delta)
            sent = robot.send_action(joint_action(motor, goal))
            time.sleep(0.20)
            after = obs_to_target(robot.get_observation())
            target.update(after)
            sent_value = float(sent[f"{motor}.pos"])
            actual_delta = after[motor] - before[motor]
            print(
                f"{motor:13s} request {delta:+6.2f} | sent {sent_value:7.2f} | "
                f"actual {actual_delta:+6.2f} -> {format_positions(target)}"
            )

    finally:
        robot.disconnect()
        print("Disconnected. Torque disabled.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
