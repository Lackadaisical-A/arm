#!/usr/bin/env python
r"""Absolute raw-position test for only elbow_flex.

This removes keyboard timing from the diagnosis. It commands fixed raw targets
inside the recorded elbow range and reports whether the encoder moved toward
the requested raw target.

Run from PowerShell:
    python .\scripts_local\elbow_flex_absolute_test.py --port COM6
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
    parser = argparse.ArgumentParser(description="Absolute raw target test for elbow_flex.")
    parser.add_argument("--port", required=True, help="Serial port, for example COM6.")
    parser.add_argument("--motor-id", type=int, default=DEFAULT_MOTOR_ID)
    parser.add_argument("--limits-file", type=Path, default=DEFAULT_LIMITS_FILE)
    parser.add_argument("--phase", type=int, default=12, help="Use Phase=12 unless intentionally testing.")
    parser.add_argument("--torque-limit", type=int, default=800, help="Temporary torque limit, 0-1000.")
    parser.add_argument("--margin", type=int, default=250, help="Raw ticks to stay away from recorded limits.")
    parser.add_argument("--hold-sec", type=float, default=1.2, help="Seconds to watch each target.")
    parser.add_argument("--sample-sec", type=float, default=0.15, help="Sample interval.")
    parser.add_argument(
        "--around-current",
        action="store_true",
        help="Test small +/- moves around the current hand-placed position instead of fixed range targets.",
    )
    parser.add_argument("--delta", type=int, default=80, help="Raw tick delta for --around-current.")
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


def read_raw(bus: FeetechMotorsBus, field: str = "Present_Position") -> int:
    return int(bus.read(field, MOTOR_NAME, normalize=False))


def write_raw(bus: FeetechMotorsBus, field: str, value: int, retries: int = 3) -> None:
    bus.write(field, MOTOR_NAME, int(value), normalize=False, num_retry=retries)


def torque_off(bus: FeetechMotorsBus) -> None:
    write_raw(bus, "Torque_Enable", 0)
    try:
        write_raw(bus, "Lock", 0)
    except Exception:
        pass


def torque_on(bus: FeetechMotorsBus, target: int, torque_limit: int) -> None:
    write_raw(bus, "Torque_Limit", torque_limit)
    write_raw(bus, "Goal_Position", target)
    write_raw(bus, "Torque_Enable", 1)
    try:
        write_raw(bus, "Lock", 1)
    except Exception:
        pass


def print_state(bus: FeetechMotorsBus, label: str) -> int:
    present = read_raw(bus)
    print(
        f"{label}: present={present} goal={read_raw(bus, 'Goal_Position')} "
        f"torque={read_raw(bus, 'Torque_Enable')} phase={read_raw(bus, 'Phase')} "
        f"load={read_raw(bus, 'Present_Load')} current={read_raw(bus, 'Present_Current')} "
        f"moving={read_raw(bus, 'Moving')} status={read_raw(bus, 'Status')}"
    )
    return present


def command_and_watch(bus: FeetechMotorsBus, target: int, args: argparse.Namespace) -> tuple[int, int]:
    start = print_state(bus, "before")
    expected = "higher raw" if target > start else "lower raw" if target < start else "same raw"
    print(f"COMMAND raw {start} -> {target} ({expected})")
    write_raw(bus, "Goal_Position", target)

    deadline = time.perf_counter() + max(0.0, args.hold_sec)
    last = start
    while time.perf_counter() < deadline:
        time.sleep(max(0.02, args.sample_sec))
        last = print_state(bus, "  sample")

    observed = last - start
    if target == start:
        verdict = "NOOP"
    elif (target > start and observed > 0) or (target < start and observed < 0):
        verdict = "OK"
    elif observed == 0:
        verdict = "STALLED"
    else:
        verdict = "WRONG_DIRECTION"
    print(f"RESULT target_delta={target - start:+d} observed_delta={observed:+d} {verdict}")
    return start, last


def hold_current(bus: FeetechMotorsBus, torque_limit: int, hold_sec: float, sample_sec: float) -> int:
    start = read_raw(bus)
    torque_on(bus, start, torque_limit)
    print_state(bus, "hold start")
    deadline = time.perf_counter() + max(0.0, hold_sec)
    last = start
    while time.perf_counter() < deadline:
        time.sleep(max(0.02, sample_sec))
        last = print_state(bus, "  hold")
    print(f"HOLD observed_delta={last - start:+d}")
    return last


def main() -> int:
    args = parse_args()
    logging.getLogger().setLevel(logging.ERROR)

    low, high = load_limits(args.limits_file)
    safe_low = low + abs(args.margin)
    safe_high = high - abs(args.margin)
    if safe_high <= safe_low:
        raise ValueError(f"Margin {args.margin} is too large for limits [{low}, {high}].")
    middle = int((safe_low + safe_high) / 2)
    span = safe_high - safe_low
    lower_target = int(middle - span * 0.22)
    higher_target = int(middle + span * 0.22)
    torque_limit = clamp(int(args.torque_limit), 0, 1000)

    bus = make_bus(args.port, args.motor_id)
    print(f"Opening {MOTOR_NAME} only on {args.port}, id={args.motor_id}...")
    print(f"Recorded limits raw=[{low}, {high}], test targets: low={lower_target}, mid={middle}, high={higher_target}")
    bus.connect()
    try:
        torque_off(bus)
        write_raw(bus, "Phase", int(args.phase))
        print(f"Phase set to {args.phase}")

        present = read_raw(bus)
        start_target = clamp(present, safe_low, safe_high)
        torque_on(bus, start_target, torque_limit)
        print_state(bus, "torque on")

        if args.around_current:
            print("\nHolding current hand-placed pose")
            center = hold_current(bus, torque_limit, args.hold_sec, args.sample_sec)
            center = clamp(center, safe_low, safe_high)
            lower = clamp(center - abs(args.delta), safe_low, safe_high)
            higher = clamp(center + abs(args.delta), safe_low, safe_high)

            print("\nStep A: command LOWER raw from current region")
            command_and_watch(bus, lower, args)

            print("\nTorque off. Move elbow_flex by hand back near the starting middle pose, then press ENTER.")
            torque_off(bus)
            input()
            center = read_raw(bus)
            torque_on(bus, clamp(center, safe_low, safe_high), torque_limit)
            print_state(bus, "recentered")
            higher = clamp(center + abs(args.delta), safe_low, safe_high)

            print("\nStep B: command HIGHER raw from same kind of region")
            command_and_watch(bus, higher, args)
            return 0

        print("\nStep 1: move to middle")
        command_and_watch(bus, middle, args)

        print("\nStep 2: command HIGHER raw")
        command_and_watch(bus, higher_target, args)

        print("\nStep 3: command LOWER raw")
        command_and_watch(bus, lower_target, args)
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
