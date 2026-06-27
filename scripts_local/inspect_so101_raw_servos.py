#!/usr/bin/env python
r"""Inspect SO-101 Feetech servo registers without running robot.configure().

Run from PowerShell after power-cycling the arm:
    python .\scripts_local\inspect_so101_raw_servos.py --port COM6
"""

from __future__ import annotations

import argparse
import logging

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


MOTORS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

FIELDS = [
    "Present_Position",
    "Goal_Position",
    "Min_Position_Limit",
    "Max_Position_Limit",
    "Max_Torque_Limit",
    "Torque_Enable",
    "Phase",
    "Operating_Mode",
    "P_Coefficient",
    "D_Coefficient",
    "I_Coefficient",
    "Minimum_Startup_Force",
    "CW_Dead_Zone",
    "CCW_Dead_Zone",
    "Acceleration",
    "Goal_Time",
    "Goal_Velocity",
    "Torque_Limit",
    "Present_Load",
    "Present_Velocity",
    "Present_Current",
    "Present_Voltage",
    "Present_Temperature",
    "Status",
    "Moving",
    "Goal_Position_2",
    "Moving_Velocity",
    "Maximum_Velocity_Limit",
    "Maximum_Acceleration",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Inspect raw SO-101 servo registers without motion commands.")
    parser.add_argument("--port", required=True, help="Robot serial port, for example COM6.")
    parser.add_argument("--robot-id", default="my_so101_follower", help="LeRobot calibration id.")
    parser.add_argument(
        "--skip-torque-off",
        action="store_true",
        help="Only read registers. By default the script first tries to disable torque on every motor.",
    )
    return parser.parse_args()


def read_field(robot: SO101Follower, field: str, motor: str) -> int | str:
    try:
        return int(robot.bus.read(field, motor, normalize=False))
    except Exception as exc:
        return f"<{type(exc).__name__}: {exc}>"


def main() -> int:
    args = parse_args()
    logging.getLogger().setLevel(logging.ERROR)

    robot = SO101Follower(SO101FollowerConfig(port=args.port, id=args.robot_id))
    print(f"Opening raw bus on {args.port} without SO-101 configure...")
    robot.bus.connect()
    try:
        if not args.skip_torque_off:
            print("Trying torque-off on all motors...")
            for motor in MOTORS:
                try:
                    robot.bus.write("Torque_Enable", motor, 0, num_retry=3)
                    print(f"  {motor}: torque off")
                except Exception as exc:
                    print(f"  {motor}: torque-off failed: {type(exc).__name__}: {exc}")

        print()
        for motor in MOTORS:
            values = {field: read_field(robot, field, motor) for field in FIELDS}
            print(
                f"{motor:14s} "
                f"present={values['Present_Position']} goal={values['Goal_Position']} "
                f"min={values['Min_Position_Limit']} max={values['Max_Position_Limit']} "
                f"max_torque={values['Max_Torque_Limit']} torque={values['Torque_Enable']} "
                f"phase={values['Phase']} mode={values['Operating_Mode']} "
                f"pdi={values['P_Coefficient']}/{values['D_Coefficient']}/{values['I_Coefficient']} "
                f"startup={values['Minimum_Startup_Force']} dead={values['CW_Dead_Zone']}/{values['CCW_Dead_Zone']} "
                f"accel={values['Acceleration']} goal_time={values['Goal_Time']} goal_vel={values['Goal_Velocity']} "
                f"torque_limit={values['Torque_Limit']} load={values['Present_Load']} "
                f"vel={values['Present_Velocity']} current={values['Present_Current']} "
                f"volt={values['Present_Voltage']} temp={values['Present_Temperature']} "
                f"status={values['Status']} moving={values['Moving']} goal2={values['Goal_Position_2']} "
                f"moving_vel={values['Moving_Velocity']} max_vel={values['Maximum_Velocity_Limit']} "
                f"max_accel={values['Maximum_Acceleration']}"
            )
    finally:
        robot.bus.disconnect(disable_torque=False)
        print("Closed serial port.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
