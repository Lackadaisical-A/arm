#!/usr/bin/env python
r"""Raw Feetech elbow_flex direction test, bypassing calibrated degrees.

Run from PowerShell:
    python .\scripts_local\test_elbow_raw_direction.py --port COM6
"""

from __future__ import annotations

import argparse
import logging
import time

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from so101_phase_utils import add_elbow_phase_arg, apply_elbow_phase


RAW_FIELDS = [
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
    "Phase",
    "Operating_Mode",
    "Acceleration",
    "P_Coefficient",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Direct raw elbow_flex servo test.")
    parser.add_argument("--port", required=True, help="Robot serial port, for example COM6.")
    parser.add_argument("--robot-id", default="my_so101_follower", help="LeRobot calibration id.")
    parser.add_argument("--raw-step", type=int, default=120, help="Raw tick step to test in each direction.")
    parser.add_argument("--hold-sec", type=float, default=1.25, help="Time to wait after each raw command.")
    parser.add_argument("--sample-sec", type=float, default=2.0, help="Time to sample raw state after each command.")
    parser.add_argument("--no-return", action="store_true", help="Do not command back to the starting raw position.")
    add_elbow_phase_arg(parser)
    return parser.parse_args()


def raw_read(robot: SO101Follower) -> dict[str, int | str]:
    values: dict[str, int | str] = {}
    for name in RAW_FIELDS:
        try:
            values[name] = int(robot.bus.read(name, "elbow_flex", normalize=False))
        except Exception as exc:
            values[name] = f"<{type(exc).__name__}: {exc}>"
    return values


def print_raw(label: str, robot: SO101Follower) -> dict[str, int | str]:
    raw = raw_read(robot)
    print(
        f"{label}: present={raw['Present_Position']} goal={raw['Goal_Position']} "
        f"min={raw['Min_Position_Limit']} max={raw['Max_Position_Limit']} "
        f"torque={raw['Torque_Enable']} moving={raw['Moving']} status={raw['Status']} "
        f"load={raw['Present_Load']} current={raw['Present_Current']} velocity={raw['Present_Velocity']} "
        f"torque_limit={raw['Torque_Limit']} max_torque={raw['Max_Torque_Limit']} "
        f"volt={raw['Present_Voltage']} temp={raw['Present_Temperature']} "
        f"phase={raw['Phase']} mode={raw['Operating_Mode']} accel={raw['Acceleration']} p={raw['P_Coefficient']}"
    )
    return raw


def sample(label: str, robot: SO101Follower, sample_sec: float) -> None:
    started = time.perf_counter()
    while time.perf_counter() - started < sample_sec:
        time.sleep(0.25)
        elapsed = time.perf_counter() - started
        print_raw(f"{label} t+{elapsed:.2f}s", robot)


def write_raw_goal(robot: SO101Follower, value: int) -> None:
    robot.bus.write("Goal_Position", "elbow_flex", int(value), normalize=False)


def int_or_none(value: int | str) -> int | None:
    return value if isinstance(value, int) else None


def main() -> int:
    args = parse_args()
    logging.getLogger().setLevel(logging.ERROR)

    robot = SO101Follower(
        SO101FollowerConfig(
            port=args.port,
            id=args.robot_id,
            max_relative_target={"elbow_flex": 4096.0},
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
        robot.bus.enable_torque("elbow_flex")
        raw = print_raw("start", robot)

        start = int(raw["Present_Position"])
        min_limit = int(raw["Min_Position_Limit"])
        max_limit = int(raw["Max_Position_Limit"])
        if start < min_limit or start > max_limit:
            print(
                "WARNING: elbow_flex present position is outside its calibrated raw limits "
                f"({start} not in [{min_limit}, {max_limit}])."
            )
            print("         This usually means the joint is against a stop, the range was recorded wrong, or the horn/linkage slipped.")
        down_target = max(min_limit + 10, start - abs(args.raw_step))
        up_target = min(max_limit - 10, start + abs(args.raw_step))

        print(f"raw command down: {start} -> {down_target}")
        write_raw_goal(robot, down_target)
        sample("down", robot, args.sample_sec)
        time.sleep(args.hold_sec)
        after_down = print_raw("after down", robot)
        after_down_position = int_or_none(after_down["Present_Position"])
        if after_down_position is None:
            print("Stopping: elbow_flex is reporting an error after the down command.")
            print("Power-cycle the servo bus before running more tests; the servo may still have torque enabled.")
            return 2

        print(f"raw command up: {after_down_position} -> {up_target}")
        write_raw_goal(robot, up_target)
        sample("up", robot, args.sample_sec)
        time.sleep(args.hold_sec)
        print_raw("after up", robot)

        if not args.no_return:
            print(f"returning raw goal to start: {start}")
            write_raw_goal(robot, start)
            time.sleep(args.hold_sec)
            print_raw("after return", robot)
    finally:
        try:
            robot.disconnect()
            print("Disconnected. Torque disabled.")
        except Exception as exc:
            print(f"Disconnect with torque-disable failed: {type(exc).__name__}: {exc}")
            print("Closing serial port without torque-disable. Power-cycle the arm now.")
            try:
                robot.bus.disconnect(disable_torque=False)
            except Exception:
                pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
