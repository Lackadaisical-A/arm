#!/usr/bin/env python
r"""Find a usable Feetech Phase value for only the SO-101 elbow_flex servo.

This tests each phase by:
  1. enabling torque at the current hand-placed pose,
  2. verifying the servo can hold that exact raw goal,
  3. commanding a small lower-raw move,
  4. commanding a small higher-raw move back toward the start.

Run from PowerShell:
    python .\scripts_local\find_elbow_flex_phase.py --port COM6
"""

from __future__ import annotations

import argparse
import json
import logging
import time
from datetime import datetime, timezone
from pathlib import Path

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus


MOTOR_NAME = "elbow_flex"
DEFAULT_MOTOR_ID = 3
DEFAULT_LIMITS_FILE = Path(__file__).resolve().parents[1] / "config" / "elbow_flex_manual_limits.json"
DEFAULT_PHASE_FILE = Path(__file__).resolve().parents[1] / "config" / "elbow_flex_phase.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Find a stable elbow_flex Feetech Phase.")
    parser.add_argument("--port", required=True, help="Serial port, for example COM6.")
    parser.add_argument("--motor-id", type=int, default=DEFAULT_MOTOR_ID, help="Elbow servo id.")
    parser.add_argument("--phases", default="0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15")
    parser.add_argument("--limits-file", type=Path, default=DEFAULT_LIMITS_FILE, help="Saved elbow raw limits JSON.")
    parser.add_argument("--phase-file", type=Path, default=DEFAULT_PHASE_FILE, help="Where to save the good phase.")
    parser.add_argument("--step-ticks", type=int, default=40, help="Small raw move used for each direction test.")
    parser.add_argument("--torque-limit", type=int, default=400, help="Temporary test torque limit, 0-1000.")
    parser.add_argument("--p-coefficient", type=int, default=32, help="Temporary P_Coefficient.")
    parser.add_argument("--min-startup-force", type=int, default=80, help="Temporary Minimum_Startup_Force.")
    parser.add_argument("--acceleration", type=int, default=254, help="Temporary Acceleration.")
    parser.add_argument("--hold-sec", type=float, default=4.0, help="Seconds a phase must hold before moving.")
    parser.add_argument("--post-move-hold-sec", type=float, default=0.8, help="Seconds to hold after each move.")
    parser.add_argument("--move-timeout", type=float, default=0.9, help="Seconds allowed for each small move.")
    parser.add_argument("--sample-sec", type=float, default=0.02, help="Encoder sample period.")
    parser.add_argument("--hold-drift-ticks", type=int, default=80, help="Reject phase if holding drifts this far.")
    parser.add_argument("--wrong-way-ticks", type=int, default=25, help="Reject phase if motion goes this far wrong-way.")
    parser.add_argument("--min-move-ticks", type=int, default=12, help="Minimum correct-way motion to count as moving.")
    parser.add_argument("--overshoot-ticks", type=int, default=120, help="Reject phase if it overshoots target this far.")
    parser.add_argument("--outside-margin", type=int, default=60, help="Extra raw ticks allowed outside saved limits.")
    parser.add_argument("--try-all", action="store_true", help="Test all phases instead of stopping at the first good one.")
    parser.add_argument("--no-save", action="store_true", help="Do not save the winning phase.")
    return parser.parse_args()


def clamp(value: int, low: int, high: int) -> int:
    return min(high, max(low, value))


def parse_phase_list(value: str) -> list[int]:
    phases: list[int] = []
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


def save_phase(path: Path, phase: int, args: argparse.Namespace, result: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "motor": MOTOR_NAME,
        "motor_id": args.motor_id,
        "port": args.port,
        "phase": phase,
        "saved_at": datetime.now(timezone.utc).isoformat(),
        "test": result,
    }
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


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


def configure_session(bus: FeetechMotorsBus, args: argparse.Namespace) -> None:
    write_raw(bus, "Operating_Mode", 0)
    write_raw(bus, "Torque_Limit", clamp(int(args.torque_limit), 0, 1000))
    write_raw(bus, "P_Coefficient", clamp(int(args.p_coefficient), 0, 254))
    write_raw(bus, "Acceleration", clamp(int(args.acceleration), 0, 254))
    if int(args.min_startup_force) > 0:
        write_raw(bus, "Minimum_Startup_Force", clamp(int(args.min_startup_force), 0, 1000))


def print_state(bus: FeetechMotorsBus, label: str) -> int:
    present = read_raw(bus)
    goal = read_raw(bus, "Goal_Position")
    phase = read_raw(bus, "Phase")
    load = read_raw(bus, "Present_Load")
    current = read_raw(bus, "Present_Current")
    status = read_raw(bus, "Status")
    print(f"{label}: present={present} goal={goal} phase={phase} load={load} current={current} status={status}")
    return present


def enable_hold(bus: FeetechMotorsBus, goal: int, args: argparse.Namespace) -> None:
    configure_session(bus, args)
    write_raw(bus, "Goal_Position", goal)
    write_raw(bus, "Torque_Enable", 1)
    try:
        write_raw(bus, "Lock", 1)
    except Exception:
        pass


def hold_here(bus: FeetechMotorsBus) -> int:
    present = read_raw(bus)
    write_raw(bus, "Goal_Position", present)
    return present


def watch_hold(
    bus: FeetechMotorsBus,
    goal: int,
    low: int,
    high: int,
    args: argparse.Namespace,
    seconds: float,
) -> tuple[bool, str, int]:
    deadline = time.perf_counter() + max(0.0, seconds)
    max_drift = 0
    last = read_raw(bus)
    while time.perf_counter() < deadline:
        time.sleep(max(0.005, args.sample_sec))
        last = read_raw(bus)
        drift = last - goal
        max_drift = max(max_drift, abs(drift))
        if abs(drift) >= abs(args.hold_drift_ticks):
            return False, f"drift goal={goal} present={last} drift={drift:+d}", last
        if last < low - args.outside_margin or last > high + args.outside_margin:
            return False, f"outside saved limits present={last}", last
    return True, f"held max_drift={max_drift}", last


def command_move(
    bus: FeetechMotorsBus,
    start: int,
    target: int,
    low: int,
    high: int,
    args: argparse.Namespace,
) -> tuple[bool, str, int]:
    direction = 1 if target > start else -1
    needed = max(int(args.min_move_ticks), min(abs(target - start), abs(args.step_ticks)) // 2)
    write_raw(bus, "Goal_Position", target)

    deadline = time.perf_counter() + max(0.05, args.move_timeout)
    last = start
    while time.perf_counter() < deadline:
        time.sleep(max(0.005, args.sample_sec))
        last = read_raw(bus)
        progress = direction * (last - start)
        past_target = direction * (last - target)

        if progress <= -abs(args.wrong_way_ticks):
            hold_here(bus)
            return False, f"wrong-way start={start} target={target} present={last}", last
        if past_target >= abs(args.overshoot_ticks):
            hold_here(bus)
            return False, f"overshoot start={start} target={target} present={last}", last
        if last < low - args.outside_margin or last > high + args.outside_margin:
            hold_here(bus)
            return False, f"outside saved limits present={last}", last
        if progress >= needed:
            stopped = hold_here(bus)
            return True, f"moved start={start} target={target} stopped={stopped}", stopped

    stopped = hold_here(bus)
    return False, f"timeout start={start} target={target} stopped={stopped}", stopped


def test_phase(bus: FeetechMotorsBus, phase: int, low: int, high: int, args: argparse.Namespace) -> dict[str, object]:
    torque_off(bus)
    start = read_raw(bus)
    margin = abs(args.step_ticks) + args.outside_margin
    if start <= low + margin or start >= high - margin:
        return {
            "ok": False,
            "reason": f"start {start} is too close to saved limits [{low}, {high}]; recenter by hand",
        }

    print(f"Writing Phase={phase}")
    write_raw(bus, "Phase", int(phase))
    print_state(bus, "after phase")

    enable_hold(bus, start, args)
    print_state(bus, "hold start")
    ok, reason, held_at = watch_hold(bus, start, low, high, args, args.hold_sec)
    if not ok:
        torque_off(bus)
        return {"ok": False, "phase": phase, "start": start, "reason": f"hold failed: {reason}"}
    print(f"hold OK: {reason}")

    lower_target = start - abs(args.step_ticks)
    ok, reason, lower_stop = command_move(bus, start, lower_target, low, high, args)
    print(f"lower test: {reason}")
    if not ok:
        torque_off(bus)
        return {"ok": False, "phase": phase, "start": start, "reason": f"lower failed: {reason}"}
    ok, reason, held_lower = watch_hold(bus, lower_stop, low, high, args, args.post_move_hold_sec)
    if not ok:
        torque_off(bus)
        return {"ok": False, "phase": phase, "start": start, "reason": f"lower hold failed: {reason}"}

    ok, reason, higher_stop = command_move(bus, held_lower, start, low, high, args)
    print(f"higher test: {reason}")
    if not ok:
        torque_off(bus)
        return {"ok": False, "phase": phase, "start": start, "reason": f"higher failed: {reason}"}
    ok, reason, _ = watch_hold(bus, higher_stop, low, high, args, args.post_move_hold_sec)
    torque_off(bus)
    if not ok:
        return {"ok": False, "phase": phase, "start": start, "reason": f"higher hold failed: {reason}"}

    return {
        "ok": True,
        "phase": phase,
        "start": start,
        "lower_stop": lower_stop,
        "higher_stop": higher_stop,
        "reason": "hold and both directions OK",
    }


def main() -> int:
    args = parse_args()
    logging.getLogger().setLevel(logging.ERROR)

    low, high = load_limits(args.limits_file)
    phases = parse_phase_list(args.phases)
    bus = make_bus(args.port, args.motor_id)

    print(f"Opening {MOTOR_NAME} only on {args.port}, id={args.motor_id}...")
    print(f"Using saved raw limits: low={low}, high={high}")
    print("Keep the arm supported. This will torque off after each phase.")
    bus.connect()
    good: tuple[int, dict[str, object]] | None = None
    results: list[dict[str, object]] = []
    try:
        original_phase = read_raw(bus, "Phase")
        print(f"Original Phase={original_phase}")
        for phase in phases:
            print()
            input(f"Move elbow_flex by hand to a safe middle pose, then press ENTER to test Phase={phase}.")
            try:
                result = test_phase(bus, phase, low, high, args)
            except Exception as exc:
                result = {"ok": False, "phase": phase, "reason": f"{type(exc).__name__}: {exc}"}
                try:
                    torque_off(bus)
                except Exception:
                    pass
            results.append(result)
            verdict = "OK" if result.get("ok") else "FAIL"
            print(f"PHASE {phase}: {verdict} - {result.get('reason')}")
            if result.get("ok") and good is None:
                good = (phase, result)
                if not args.try_all:
                    break

        print()
        print("Summary:")
        for result in results:
            verdict = "OK" if result.get("ok") else "FAIL"
            print(f"  phase={result.get('phase')}: {verdict} - {result.get('reason')}")

        if good is None:
            print(f"No stable phase found. Restoring original Phase={original_phase}.")
            write_raw(bus, "Phase", original_phase)
            return 2

        phase, result = good
        write_raw(bus, "Phase", phase)
        if not args.no_save:
            save_phase(args.phase_file, phase, args, result)
            print(f"Saved Phase={phase} to {args.phase_file}")
        print(f"Leaving Phase={phase}")
        return 0
    finally:
        try:
            torque_off(bus)
            print("Torque off.")
        except Exception as exc:
            print(f"Torque-off failed: {type(exc).__name__}: {exc}")
            print("Power-cycle the arm.")
        bus.disconnect(disable_torque=False)
        print("Closed serial port.")


if __name__ == "__main__":
    raise SystemExit(main())
