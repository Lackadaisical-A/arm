#!/usr/bin/env python
r"""Direct SO-101 elbow_flex test, bypassing RViz/ROS/UDP.

This reads the current calibrated elbow position, commands a small positive
step, reads back, then commands back to the original position.

Run from PowerShell:
    python .\scripts_local\test_elbow_flex_direct.py --port COM6
"""

from __future__ import annotations

import argparse
import logging
import time

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from so101_phase_utils import add_elbow_phase_arg, apply_elbow_phase


MOTORS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Direct LeRobot elbow_flex motion test.")
    parser.add_argument("--port", required=True, help="Robot serial port, for example COM6.")
    parser.add_argument("--robot-id", default="my_so101_follower", help="LeRobot calibration id.")
    parser.add_argument("--step-deg", type=float, default=8.0, help="Small elbow test step in calibrated degrees.")
    parser.add_argument("--hold-sec", type=float, default=0.75, help="Time to wait after each command.")
    parser.add_argument("--sample-sec", type=float, default=2.0, help="Time to sample raw state after the command.")
    parser.add_argument("--center", action="store_true", help="Command elbow to calibrated center 0 deg.")
    parser.add_argument("--target", type=float, default=None, help="Explicit calibrated elbow target in degrees.")
    parser.add_argument("--no-return", action="store_true", help="Do not command elbow back to the start.")
    add_elbow_phase_arg(parser)
    return parser.parse_args()


def elbow_limits_deg(robot: SO101Follower) -> tuple[float, float]:
    calibration = robot.calibration["elbow_flex"]
    model = robot.bus.motors["elbow_flex"].model
    max_res = robot.bus.model_resolution_table[model] - 1
    mid = (calibration.range_min + calibration.range_max) / 2
    low = (calibration.range_min - mid) * 360 / max_res
    high = (calibration.range_max - mid) * 360 / max_res
    return float(low), float(high)


def clamp(value: float, low: float, high: float) -> float:
    return min(high, max(low, value))


def read_elbow(robot: SO101Follower) -> float:
    return float(robot.get_observation()["elbow_flex.pos"])


def read_raw(robot: SO101Follower) -> dict[str, int]:
    names = [
        "Present_Position",
        "Goal_Position",
        "Min_Position_Limit",
        "Max_Position_Limit",
        "Torque_Enable",
        "Torque_Limit",
        "Max_Torque_Limit",
        "Present_Load",
        "Present_Current",
        "Present_Velocity",
        "Present_Voltage",
        "Present_Temperature",
        "Status",
        "Moving",
        "Operating_Mode",
        "Acceleration",
        "P_Coefficient",
    ]
    values = {}
    for name in names:
        try:
            values[name] = int(robot.bus.read(name, "elbow_flex", normalize=False))
        except Exception as exc:
            values[name] = f"<{type(exc).__name__}: {exc}>"
    return values


def print_raw(label: str, robot: SO101Follower) -> None:
    raw = read_raw(robot)
    print(
        f"{label} raw: present={raw['Present_Position']} goal={raw['Goal_Position']} "
        f"min={raw['Min_Position_Limit']} max={raw['Max_Position_Limit']} "
        f"torque={raw['Torque_Enable']} moving={raw['Moving']} status={raw['Status']} "
        f"load={raw['Present_Load']} current={raw['Present_Current']} velocity={raw['Present_Velocity']} "
        f"torque_limit={raw['Torque_Limit']} max_torque={raw['Max_Torque_Limit']} "
        f"volt={raw['Present_Voltage']} temp={raw['Present_Temperature']} "
        f"mode={raw['Operating_Mode']} accel={raw['Acceleration']} p={raw['P_Coefficient']}"
    )


def send_elbow(robot: SO101Follower, value: float) -> None:
    robot.send_action({"elbow_flex.pos": float(value)})


def main() -> int:
    args = parse_args()
    logging.getLogger().setLevel(logging.ERROR)

    robot = SO101Follower(
        SO101FollowerConfig(
            port=args.port,
            id=args.robot_id,
            max_relative_target={"elbow_flex": 220.0},
        )
    )

    print(f"Connecting to SO-101 follower on {args.port}...")
    robot.connect()
    apply_elbow_phase(
        robot,
        args.elbow_phase,
        torque_limit=args.elbow_torque_limit,
        startup_force=args.elbow_startup_force,
        p_coefficient=args.elbow_p_coefficient,
    )
    try:
        low, high = elbow_limits_deg(robot)
        start = read_elbow(robot)
        if args.target is not None:
            target = clamp(args.target, low, high)
        elif args.center:
            target = 0.0
        else:
            direction = 1.0 if start + args.step_deg <= high else -1.0
            target = clamp(start + direction * abs(args.step_deg), low, high)

        print(f"elbow_flex limits: {low:+.2f} to {high:+.2f}")
        print(f"start: {start:+.2f}")
        print(f"command: {target:+.2f}")
        print_raw("before", robot)

        robot.bus.enable_torque("elbow_flex")
        print_raw("torque on", robot)
        send_elbow(robot, target)
        sample_start = time.perf_counter()
        while time.perf_counter() - sample_start < args.sample_sec:
            time.sleep(0.25)
            now = time.perf_counter() - sample_start
            print_raw(f"t+{now:.2f}s", robot)
        time.sleep(args.hold_sec)
        after = read_elbow(robot)
        print(f"after command: {after:+.2f}  delta={after - start:+.2f}")
        print_raw("after command", robot)

        if not args.no_return:
            print(f"returning to start: {start:+.2f}")
            send_elbow(robot, start)
            time.sleep(args.hold_sec)
            returned = read_elbow(robot)
            print(f"after return: {returned:+.2f}  delta_from_start={returned - start:+.2f}")
            print_raw("after return", robot)

    finally:
        robot.disconnect()
        print("Disconnected. Torque disabled.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
