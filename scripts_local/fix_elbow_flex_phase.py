#!/usr/bin/env python
r"""Set/test Feetech Phase for only the SO-101 elbow_flex servo.

Run from PowerShell after power-cycling the arm:
    python .\scripts_local\fix_elbow_flex_phase.py --port COM6
"""

from __future__ import annotations

import argparse
import json
import logging
import time
from pathlib import Path

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus


MOTOR_NAME = "elbow_flex"
DEFAULT_MOTOR_ID = 3
DEFAULT_LIMITS_FILE = Path(__file__).resolve().parents[1] / "config" / "elbow_flex_manual_limits.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Set/test elbow_flex Feetech Phase.")
    parser.add_argument("--port", required=True, help="Serial port, for example COM6.")
    parser.add_argument("--motor-id", type=int, default=DEFAULT_MOTOR_ID, help="Elbow servo id.")
    parser.add_argument("--phase", type=int, default=12, help="Phase value to write. Use 12 first.")
    parser.add_argument("--test-step", type=int, default=-25, help="Raw tick delta for the test move.")
    parser.add_argument("--torque-limit", type=int, default=220, help="Temporary test torque limit, 0-1000.")
    parser.add_argument("--hold-sec", type=float, default=0.7, help="Time to wait after the test command.")
    parser.add_argument("--limits-file", type=Path, default=DEFAULT_LIMITS_FILE, help="Saved elbow raw limits JSON.")
    parser.add_argument("--no-test", action="store_true", help="Only write Phase, do not move.")
    return parser.parse_args()


def clamp(value: int, low: int, high: int) -> int:
    return min(high, max(low, value))


def load_limits(path: Path) -> tuple[int, int]:
    data = json.loads(path.read_text(encoding="utf-8"))
    low, high = int(data["low"]), int(data["high"])
    return (low, high) if low <= high else (high, low)


def make_bus(port: str, motor_id: int) -> FeetechMotorsBus:
    return FeetechMotorsBus(
        port=port,
        motors={MOTOR_NAME: Motor(motor_id, "sts3215", MotorNormMode.DEGREES)},
    )


def read_raw(bus: FeetechMotorsBus, field: str) -> int:
    return int(bus.read(field, MOTOR_NAME, normalize=False))


def write_raw(bus: FeetechMotorsBus, field: str, value: int, retries: int = 3) -> None:
    bus.write(field, MOTOR_NAME, int(value), normalize=False, num_retry=retries)


def torque_off(bus: FeetechMotorsBus) -> None:
    write_raw(bus, "Torque_Enable", 0)
    try:
        write_raw(bus, "Lock", 0)
    except Exception:
        pass


def print_state(bus: FeetechMotorsBus, label: str) -> int:
    present = read_raw(bus, "Present_Position")
    goal = read_raw(bus, "Goal_Position")
    phase = read_raw(bus, "Phase")
    load = read_raw(bus, "Present_Load")
    current = read_raw(bus, "Present_Current")
    status = read_raw(bus, "Status")
    print(
        f"{label}: present={present} goal={goal} phase={phase} "
        f"load={load} current={current} status={status}"
    )
    return present


def main() -> int:
    args = parse_args()
    logging.getLogger().setLevel(logging.ERROR)

    low, high = load_limits(args.limits_file)
    bus = make_bus(args.port, args.motor_id)
    print(f"Opening {MOTOR_NAME} only on {args.port}, id={args.motor_id}...")
    bus.connect()
    try:
        torque_off(bus)
        before = print_state(bus, "before")

        print(f"Writing Phase={args.phase}...")
        write_raw(bus, "Phase", int(args.phase))
        after_phase = print_state(bus, "after phase")

        if args.no_test:
            return 0

        target = clamp(after_phase + int(args.test_step), low, high)
        torque_limit = clamp(int(args.torque_limit), 0, 1000)
        print(f"Test move: {after_phase} -> {target} delta={target - after_phase:+d} torque_limit={torque_limit}")

        write_raw(bus, "Torque_Limit", torque_limit)
        write_raw(bus, "Goal_Position", after_phase)
        write_raw(bus, "Torque_Enable", 1)
        time.sleep(0.15)
        write_raw(bus, "Goal_Position", target)
        time.sleep(max(0.0, args.hold_sec))

        final = print_state(bus, "after test")
        moved = final - before
        print(f"Observed raw delta from start: {moved:+d}")
        if args.test_step < 0 and moved < 0:
            print("Result: OK, lower raw command moved lower raw.")
        elif args.test_step > 0 and moved > 0:
            print("Result: OK, higher raw command moved higher raw.")
        else:
            print("Result: still wrong direction or stalled.")
    finally:
        try:
            torque_off(bus)
            print("Torque off.")
        except Exception as exc:
            print(f"Torque-off failed: {type(exc).__name__}: {exc}")
            print("Power-cycle the arm.")
        bus.disconnect(disable_torque=False)
        print("Closed serial port.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
