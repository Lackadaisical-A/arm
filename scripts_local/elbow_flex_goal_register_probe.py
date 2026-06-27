#!/usr/bin/env python
r"""Probe elbow_flex Goal_Position writes with torque off.

This does not move the servo. It writes a few raw Goal_Position values while
torque is disabled, then reads Goal_Position back from the servo.

Run:
    python .\scripts_local\elbow_flex_goal_register_probe.py --port COM6
"""

from __future__ import annotations

import argparse
import logging

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus


MOTOR_NAME = "elbow_flex"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Probe raw elbow_flex Goal_Position writes with torque off.")
    parser.add_argument("--port", required=True, help="Serial port, for example COM6.")
    parser.add_argument("--motor-id", type=int, default=3)
    parser.add_argument("--delta", type=int, default=80)
    return parser.parse_args()


def make_bus(port: str, motor_id: int) -> FeetechMotorsBus:
    return FeetechMotorsBus(
        port=port,
        motors={MOTOR_NAME: Motor(motor_id, "sts3215", MotorNormMode.DEGREES)},
    )


def read_raw(bus: FeetechMotorsBus, field: str) -> int:
    return int(bus.read(field, MOTOR_NAME, normalize=False))


def write_raw(bus: FeetechMotorsBus, field: str, value: int) -> None:
    bus.write(field, MOTOR_NAME, int(value), normalize=False, num_retry=3)


def clamp_raw(value: int) -> int:
    return min(4095, max(0, value))


def main() -> int:
    args = parse_args()
    logging.getLogger().setLevel(logging.ERROR)

    bus = make_bus(args.port, args.motor_id)
    print(f"Opening {MOTOR_NAME} only on {args.port}, id={args.motor_id}...")
    bus.connect()
    try:
        write_raw(bus, "Torque_Enable", 0)
        present = read_raw(bus, "Present_Position")
        targets = [
            clamp_raw(present),
            clamp_raw(present - abs(args.delta)),
            clamp_raw(present + abs(args.delta)),
            clamp_raw(present),
        ]
        print(f"present={present} torque={read_raw(bus, 'Torque_Enable')}")
        for target in targets:
            write_raw(bus, "Goal_Position", target)
            readback = read_raw(bus, "Goal_Position")
            result = "OK" if readback == target else "MISMATCH"
            print(f"write_goal={target} readback_goal={readback} {result}")
    finally:
        try:
            write_raw(bus, "Torque_Enable", 0)
        except Exception:
            pass
        bus.disconnect(disable_torque=False)
        print("Closed serial port.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
