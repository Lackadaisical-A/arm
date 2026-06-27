#!/usr/bin/env python
r"""Interactively test Feetech Phase values for only elbow_flex.

Run from PowerShell:
    python .\scripts_local\sweep_elbow_flex_phase.py --port COM6
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
    parser = argparse.ArgumentParser(description="Sweep/test elbow_flex Feetech Phase values.")
    parser.add_argument("--port", required=True, help="Serial port, for example COM6.")
    parser.add_argument("--motor-id", type=int, default=DEFAULT_MOTOR_ID, help="Elbow servo id.")
    parser.add_argument(
        "--phases",
        default="0,4,8,12",
        help="Comma-separated Phase values to test. Try 0,4,8,12 first.",
    )
    parser.add_argument("--test-step", type=int, default=-12, help="Raw tick delta for each test move.")
    parser.add_argument("--torque-limit", type=int, default=120, help="Temporary test torque limit, 0-1000.")
    parser.add_argument("--hold-sec", type=float, default=0.25, help="Maximum time to wait after each test command.")
    parser.add_argument("--sample-sec", type=float, default=0.04, help="Sample interval during each test move.")
    parser.add_argument("--wrong-stop-ticks", type=int, default=45, help="Torque-off when wrong-way motion exceeds this.")
    parser.add_argument("--ok-stop-ticks", type=int, default=6, help="Torque-off when correct-way motion reaches this.")
    parser.add_argument("--limits-file", type=Path, default=DEFAULT_LIMITS_FILE, help="Saved elbow raw limits JSON.")
    return parser.parse_args()


def clamp(value: int, low: int, high: int) -> int:
    return min(high, max(low, value))


def parse_phase_list(value: str) -> list[int]:
    phases = []
    for part in value.split(","):
        part = part.strip()
        if part:
            phases.append(int(part, 0))
    if not phases:
        raise ValueError("At least one phase is required.")
    return phases


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


def test_phase(bus: FeetechMotorsBus, phase: int, low: int, high: int, args: argparse.Namespace) -> int:
    torque_off(bus)
    start = print_state(bus, "before")
    target = clamp(start + int(args.test_step), low, high)
    if target == start:
        raise RuntimeError(f"Current position {start} cannot move by {args.test_step} inside [{low}, {high}].")

    print(f"Writing Phase={phase}")
    write_raw(bus, "Phase", phase)
    print_state(bus, "after phase")

    torque_limit = clamp(int(args.torque_limit), 0, 1000)
    write_raw(bus, "Torque_Limit", torque_limit)
    write_raw(bus, "Goal_Position", start)
    write_raw(bus, "Torque_Enable", 1)
    time.sleep(0.12)
    print(f"test: goal {start} -> {target} delta={target - start:+d} torque_limit={torque_limit}")
    write_raw(bus, "Goal_Position", target)
    expected_sign = -1 if args.test_step < 0 else 1
    deadline = time.perf_counter() + max(0.0, args.hold_sec)
    while time.perf_counter() < deadline:
        time.sleep(max(0.01, args.sample_sec))
        current = read_raw(bus, "Present_Position")
        delta = current - start
        wrong_way = (expected_sign < 0 and delta > 0) or (expected_sign > 0 and delta < 0)
        correct_way = (expected_sign < 0 and delta < 0) or (expected_sign > 0 and delta > 0)
        if wrong_way and abs(delta) >= abs(args.wrong_stop_ticks):
            print(f"early stop: wrong-way delta={delta:+d}")
            break
        if correct_way and abs(delta) >= abs(args.ok_stop_ticks):
            print(f"early stop: correct-way delta={delta:+d}")
            break
    torque_off(bus)
    end = print_state(bus, "after")

    observed = end - start
    observed_sign = -1 if observed < 0 else 1 if observed > 0 else 0
    if observed_sign == expected_sign:
        print(f"PHASE {phase}: OK observed_delta={observed:+d}")
    else:
        print(f"PHASE {phase}: WRONG observed_delta={observed:+d}")
    return observed


def main() -> int:
    args = parse_args()
    logging.getLogger().setLevel(logging.ERROR)

    low, high = load_limits(args.limits_file)
    phases = parse_phase_list(args.phases)

    bus = make_bus(args.port, args.motor_id)
    print(f"Opening {MOTOR_NAME} only on {args.port}, id={args.motor_id}...")
    print(f"Using saved raw limits: low={low}, high={high}")
    print(f"Testing phases: {phases}")
    bus.connect()
    try:
        original_phase = read_raw(bus, "Phase")
        print(f"Original Phase={original_phase}")
        results: list[tuple[int, int]] = []
        ok_phase: int | None = None
        for phase in phases:
            print()
            input(f"Move elbow_flex by hand to a safe middle position, then press ENTER to test Phase={phase}.")
            try:
                observed = test_phase(bus, phase, low, high, args)
                results.append((phase, observed))
                is_ok = observed < 0 if args.test_step < 0 else observed > 0
                if is_ok and ok_phase is None:
                    ok_phase = phase
            except Exception as exc:
                print(f"PHASE {phase}: ERROR {type(exc).__name__}: {exc}")
                print("Power-cycle if the servo is stuck or reporting overload.")
                break

        print()
        print("Summary:")
        for phase, observed in results:
            direction = "OK" if (observed < 0 if args.test_step < 0 else observed > 0) else "WRONG"
            print(f"  phase={phase}: observed_delta={observed:+d} {direction}")
        phase_to_keep = ok_phase if ok_phase is not None else original_phase
        torque_off(bus)
        write_raw(bus, "Phase", phase_to_keep)
        print(f"Leaving Phase={phase_to_keep}" + (" (first OK phase)" if ok_phase is not None else " (restored original)"))
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
