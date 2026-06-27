#!/usr/bin/env python
r"""Detect which named SO-101 motor changes when you move each physical joint.

Run from PowerShell after activating the LeRobot venv:
    python .\scripts_local\detect_so101_motor_mapping.py --port COM4
"""

from __future__ import annotations

import argparse
import logging
import time

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


MOTORS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

PHYSICAL_JOINTS = [
    "base / shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Detect SO-101 physical-to-motor-name mapping.")
    parser.add_argument("--port", required=True, help="Robot serial port, for example COM4.")
    parser.add_argument("--robot-id", default="my_so101_follower", help="LeRobot calibration id.")
    parser.add_argument("--seconds", type=float, default=4.0, help="Seconds to watch each joint.")
    return parser.parse_args()


def read_positions(robot: SO101Follower) -> dict[str, float]:
    obs = robot.get_observation()
    return {motor: float(obs[f"{motor}.pos"]) for motor in MOTORS}


def main() -> int:
    args = parse_args()
    logging.getLogger().setLevel(logging.ERROR)

    robot = SO101Follower(SO101FollowerConfig(port=args.port, id=args.robot_id))
    print(f"Connecting to SO-101 follower on {args.port} with id '{args.robot_id}'...")
    robot.connect()

    try:
        print("Disabling torque. Hold the arm before it goes limp.")
        robot.bus.disable_torque()
        time.sleep(0.2)

        print(
            "\nFor each prompt, move only that physical joint back and forth a little.\n"
            "The script will report which LeRobot motor name changed the most.\n"
        )

        results: list[tuple[str, str, float, dict[str, float]]] = []
        for physical_joint in PHYSICAL_JOINTS:
            input(f"Press Enter, then move ONLY physical joint '{physical_joint}' for {args.seconds:.1f}s...")
            start = read_positions(robot)
            max_delta = dict.fromkeys(MOTORS, 0.0)
            end_t = time.perf_counter() + args.seconds
            while time.perf_counter() < end_t:
                current = read_positions(robot)
                for motor in MOTORS:
                    max_delta[motor] = max(max_delta[motor], abs(current[motor] - start[motor]))
                time.sleep(0.03)

            detected_motor = max(max_delta, key=max_delta.get)
            detected_delta = max_delta[detected_motor]
            results.append((physical_joint, detected_motor, detected_delta, max_delta))
            print(f"  detected: {detected_motor} changed most ({detected_delta:.2f})")
            print("  all deltas:", " | ".join(f"{m}:{max_delta[m]:.2f}" for m in MOTORS))

        print("\nSummary:")
        for physical_joint, detected_motor, detected_delta, _ in results:
            print(f"  physical {physical_joint:20s} -> LeRobot motor '{detected_motor}' ({detected_delta:.2f})")

        print(
            "\nExpected mapping:\n"
            "  base / shoulder_pan    -> shoulder_pan\n"
            "  shoulder_lift          -> shoulder_lift\n"
            "  elbow_flex             -> elbow_flex\n"
            "  wrist_flex             -> wrist_flex\n"
            "  wrist_roll             -> wrist_roll\n"
            "  gripper                -> gripper\n"
        )
        return 0
    finally:
        robot.disconnect()
        print("Disconnected. Torque disabled.")


if __name__ == "__main__":
    raise SystemExit(main())
