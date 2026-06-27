#!/usr/bin/env python
r"""Record elbow_flex raw low/high, then manually control only that servo.

This does not call SO101Follower.connect(), so it does not run the full robot
configure/calibration path. It opens a Feetech bus with only motor id 3.

Run from PowerShell:
    python .\scripts_local\elbow_flex_record_control.py --port COM6
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus


MOTOR_NAME = "elbow_flex"
DEFAULT_MOTOR_ID = 3
DEFAULT_LIMITS_FILE = Path(__file__).resolve().parents[1] / "config" / "elbow_flex_manual_limits.json"
DEFAULT_PHASE_FILE = Path(__file__).resolve().parents[1] / "config" / "elbow_flex_phase.json"


try:
    import msvcrt
except ImportError:  # pragma: no cover - Windows is the expected target here.
    msvcrt = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record and manually control only elbow_flex.")
    parser.add_argument("--port", required=True, help="Serial port, for example COM6.")
    parser.add_argument("--motor-id", type=int, default=DEFAULT_MOTOR_ID, help="Elbow servo id.")
    parser.add_argument("--low", type=int, default=None, help="Use this raw low limit instead of recording.")
    parser.add_argument("--high", type=int, default=None, help="Use this raw high limit instead of recording.")
    parser.add_argument("--use-saved", action="store_true", help="Load low/high from --limits-file.")
    parser.add_argument("--limits-file", type=Path, default=DEFAULT_LIMITS_FILE, help="JSON file for saved low/high.")
    parser.add_argument("--sample-hz", type=float, default=20.0, help="Recording sample rate.")
    parser.add_argument("--step-ticks", type=int, default=15, help="Keyboard control step in raw ticks.")
    parser.add_argument("--torque-limit", type=int, default=1000, help="Session torque limit, 0-1000.")
    parser.add_argument("--p-coefficient", type=int, default=32, help="Elbow position P coefficient for this session.")
    parser.add_argument(
        "--min-startup-force",
        type=int,
        default=800,
        help="Minimum_Startup_Force for this session. Use 0 to leave unchanged.",
    )
    parser.add_argument("--acceleration", type=int, default=254, help="Elbow acceleration for this session.")
    parser.add_argument("--settle-sec", type=float, default=0.05, help="Delay after each command.")
    parser.add_argument("--sample-sec", type=float, default=0.02, help="Encoder sample interval while moving.")
    parser.add_argument("--move-timeout", type=float, default=0.8, help="Maximum seconds for one jog.")
    parser.add_argument("--stop-tolerance", type=int, default=4, help="Raw ticks allowed around each jog target.")
    parser.add_argument("--hold-guard-sec", type=float, default=3.0, help="Seconds to verify the elbow can hold after torque-on.")
    parser.add_argument("--hold-guard-ticks", type=int, default=120, help="Torque-off if hold drifts by this many raw ticks.")
    parser.add_argument(
        "--direct-goal",
        action="store_true",
        help="Old behavior: write the raw target and let the servo PID stop itself.",
    )
    parser.add_argument("--hold-sec", type=float, default=0.5, help="Delay after --test-step command.")
    parser.add_argument(
        "--phase",
        type=int,
        default=12,
        help="Write Feetech Phase before enabling torque. Default is 12. Use -1 to skip.",
    )
    parser.add_argument("--phase-file", type=Path, default=DEFAULT_PHASE_FILE, help="Saved elbow phase JSON.")
    parser.add_argument(
        "--down-is-low",
        action="store_true",
        help="Make physical d/u keys treat raw low as down. Default assumes raw high is down.",
    )
    parser.add_argument(
        "--test-step",
        type=int,
        default=None,
        help="Command one raw delta from the current position, print state, torque off, and exit.",
    )
    parser.add_argument(
        "--write-servo-limits",
        action="store_true",
        help="Also write recorded low/high into the servo Min/Max_Position_Limit registers.",
    )
    return parser.parse_args()


def clamp(value: int, low: int, high: int) -> int:
    return min(high, max(low, value))


def load_limits(path: Path) -> tuple[int, int]:
    data = json.loads(path.read_text(encoding="utf-8"))
    return int(data["low"]), int(data["high"])


def load_saved_phase(path: Path) -> int | None:
    if not path.exists():
        return None
    data = json.loads(path.read_text(encoding="utf-8"))
    return int(data["phase"])


def save_limits(path: Path, low: int, high: int, port: str, motor_id: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "motor": MOTOR_NAME,
        "motor_id": motor_id,
        "port": port,
        "low": low,
        "high": high,
        "saved_at": datetime.now(timezone.utc).isoformat(),
    }
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def enter_pressed() -> bool:
    if msvcrt is not None:
        if not msvcrt.kbhit():
            return False
        key = msvcrt.getwch()
        return key in ("\r", "\n")
    return False


def read_raw(bus: FeetechMotorsBus, name: str = "Present_Position") -> int:
    return int(bus.read(name, MOTOR_NAME, normalize=False))


def write_raw(bus: FeetechMotorsBus, name: str, value: int, retries: int = 0) -> None:
    bus.write(name, MOTOR_NAME, int(value), normalize=False, num_retry=retries)


def print_state(bus: FeetechMotorsBus, label: str, low: int, high: int, target: int | None = None) -> None:
    present = read_raw(bus)
    goal = read_raw(bus, "Goal_Position")
    load = read_raw(bus, "Present_Load")
    current = read_raw(bus, "Present_Current")
    torque = read_raw(bus, "Torque_Enable")
    phase = read_raw(bus, "Phase")
    mode = read_raw(bus, "Operating_Mode")
    status = read_raw(bus, "Status")
    p_coeff = read_raw(bus, "P_Coefficient")
    startup_force = read_raw(bus, "Minimum_Startup_Force")
    torque_limit = read_raw(bus, "Torque_Limit")
    pct = 100.0 * (present - low) / max(1, high - low)
    target_text = "" if target is None else f" target={target}"
    print(
        f"{label}: present={present} ({pct:5.1f}%) goal={goal}{target_text} "
        f"torque={torque} load={load} current={current} phase={phase} mode={mode} status={status} "
        f"torque_limit={torque_limit} p={p_coeff} startup={startup_force}"
    )


def make_bus(port: str, motor_id: int) -> FeetechMotorsBus:
    return FeetechMotorsBus(
        port=port,
        motors={MOTOR_NAME: Motor(motor_id, "sts3215", MotorNormMode.DEGREES)},
    )


def torque_off(bus: FeetechMotorsBus) -> None:
    write_raw(bus, "Torque_Enable", 0, retries=3)
    try:
        write_raw(bus, "Lock", 0, retries=3)
    except Exception:
        pass


def torque_on(bus: FeetechMotorsBus, target: int, torque_limit: int) -> None:
    write_raw(bus, "Torque_Limit", torque_limit, retries=3)
    write_raw(bus, "Goal_Position", target, retries=3)
    write_raw(bus, "Torque_Enable", 1, retries=3)
    try:
        write_raw(bus, "Lock", 1, retries=3)
    except Exception:
        pass


def configure_elbow_session(bus: FeetechMotorsBus, args: argparse.Namespace, torque_limit: int) -> None:
    write_raw(bus, "Operating_Mode", 0, retries=3)
    write_raw(bus, "Torque_Limit", torque_limit, retries=3)
    write_raw(bus, "P_Coefficient", clamp(int(args.p_coefficient), 0, 254), retries=3)
    write_raw(bus, "Acceleration", clamp(int(args.acceleration), 0, 254), retries=3)
    if int(args.min_startup_force) > 0:
        write_raw(bus, "Minimum_Startup_Force", clamp(int(args.min_startup_force), 0, 1000), retries=3)


def torque_enabled(bus: FeetechMotorsBus) -> bool:
    return bool(int(read_raw(bus, "Torque_Enable")))


def load_is_saturated(bus: FeetechMotorsBus, torque_limit: int) -> bool:
    if torque_limit <= 0:
        return False
    return abs(read_raw(bus, "Present_Load")) >= max(20, int(torque_limit * 0.90))


def guard_hold(bus: FeetechMotorsBus, goal: int, args: argparse.Namespace) -> bool:
    deadline = time.perf_counter() + max(0.0, args.hold_guard_sec)
    while time.perf_counter() < deadline:
        time.sleep(max(0.02, args.sample_sec))
        present = read_raw(bus)
        if abs(present - goal) >= abs(args.hold_guard_ticks):
            print(
                f"HOLD FAILED: goal={goal} present={present} drift={present - goal:+d}. "
                "Torque off for safety."
            )
            torque_off(bus)
            return False
    return True


def stop_if_goal_drifted(bus: FeetechMotorsBus, args: argparse.Namespace) -> bool:
    if not torque_enabled(bus):
        return False
    present = read_raw(bus)
    goal = read_raw(bus, "Goal_Position")
    drift = present - goal
    if abs(drift) < abs(args.hold_guard_ticks):
        return False
    print(f"HOLD FAILED: goal={goal} present={present} drift={drift:+d}. Torque off for safety.")
    torque_off(bus)
    return True


def resolve_phase(args: argparse.Namespace) -> int | None:
    if args.phase is not None:
        return int(args.phase)
    return load_saved_phase(args.phase_file)


def hold_here(bus: FeetechMotorsBus) -> int:
    present = read_raw(bus)
    write_raw(bus, "Goal_Position", present, retries=3)
    return present


def controlled_move_to(
    bus: FeetechMotorsBus,
    target: int,
    low: int,
    high: int,
    args: argparse.Namespace,
) -> tuple[int, int, str]:
    start = read_raw(bus)
    target = clamp(target, low, high)
    if abs(target - start) <= args.stop_tolerance:
        stopped = hold_here(bus)
        return start, stopped, "already-there"

    direction = 1 if target > start else -1
    write_raw(bus, "Goal_Position", target, retries=3)

    status = "timeout"
    deadline = time.perf_counter() + max(0.05, args.move_timeout)
    last = start
    while time.perf_counter() < deadline:
        time.sleep(max(0.005, args.sample_sec))
        current = read_raw(bus)
        last = current
        moved = current - start

        if direction > 0 and current >= target - args.stop_tolerance:
            status = "reached"
            break
        if direction < 0 and current <= target + args.stop_tolerance:
            status = "reached"
            break
        if direction > 0 and moved < -abs(args.step_ticks):
            status = "wrong-way"
            break
        if direction < 0 and moved > abs(args.step_ticks):
            status = "wrong-way"
            break
        moving_farther_below_low = current < low and direction < 0
        moving_farther_above_high = current > high and direction > 0
        if moving_farther_below_low or moving_farther_above_high:
            status = "outside-limits"
            break

    stopped = hold_here(bus)
    return start, stopped, status


def record_limits(bus: FeetechMotorsBus, sample_hz: float) -> tuple[int, int]:
    torque_off(bus)
    input("Torque is off. Move elbow_flex to a safe middle position, then press ENTER to start recording.")
    print("Move elbow_flex through the safe physical range. Press ENTER again to stop.")

    low: int | None = None
    high: int | None = None
    samples = 0
    interval = 1.0 / max(1.0, sample_hz)

    while True:
        pos = read_raw(bus)
        low = pos if low is None else min(low, pos)
        high = pos if high is None else max(high, pos)
        samples += 1
        sys.stdout.write(f"\rpresent={pos:4d} low={low:4d} high={high:4d} samples={samples:4d}  ")
        sys.stdout.flush()
        if enter_pressed():
            print()
            break
        time.sleep(interval)

    if low is None or high is None or high - low < 30:
        raise RuntimeError(f"Recorded range is too small: low={low}, high={high}. Move the joint through more range.")
    return low, high


def get_key() -> str:
    if msvcrt is None:
        return sys.stdin.read(1)
    key = msvcrt.getwch()
    if key in ("\x00", "\xe0"):
        return ""
    return key


def manual_control(bus: FeetechMotorsBus, low: int, high: int, args: argparse.Namespace) -> None:
    present = read_raw(bus)
    target = present

    phase = resolve_phase(args)
    if phase is not None and phase >= 0:
        torque_off(bus)
        write_raw(bus, "Phase", int(phase), retries=3)
        print(f"Wrote Feetech Phase={phase}")

    if args.write_servo_limits:
        torque_off(bus)
        write_raw(bus, "Min_Position_Limit", low, retries=3)
        write_raw(bus, "Max_Position_Limit", high, retries=3)
        print(f"Wrote servo raw limits: low={low}, high={high}")

    torque_limit = clamp(int(args.torque_limit), 0, 1000)
    configure_elbow_session(bus, args, torque_limit)
    torque_on(bus, target, torque_limit)

    print()
    print(
        f"Using elbow_flex raw limits: low={low}, high={high}, start target={target}, "
        f"torque_limit={torque_limit}, p={args.p_coefficient}, startup={args.min_startup_force}"
    )
    print("Keys: u/d = labeled up/down, j/l = raw -/+, J/L = raw -/+ 5x, 1/2 = one raw -/+ test")
    print("      r = read, x = torque off, e = torque on, q = quit")
    print("Every jog is encoder-watched pulse control unless --direct-goal is used.")
    print_state(bus, "start", low, high, target)
    if not guard_hold(bus, target, args):
        print("The elbow cannot hold this pose. Move it by hand to a neutral middle pose, then press e.")

    while True:
        key = get_key()
        if not key:
            continue

        try:
            target: int | None = None
            if key == "q":
                break
            if key == "j":
                delta = -abs(args.step_ticks)
            elif key == "l":
                delta = abs(args.step_ticks)
            elif key == "J":
                delta = -abs(args.step_ticks) * 5
            elif key == "L":
                delta = abs(args.step_ticks) * 5
            elif key == "u":
                delta = abs(args.step_ticks) if args.down_is_low else -abs(args.step_ticks)
            elif key == "d":
                delta = -abs(args.step_ticks) if args.down_is_low else abs(args.step_ticks)
            elif key == "1":
                delta = -1
            elif key == "2":
                delta = 1
            elif key == "[":
                delta = None
                target = low
            elif key == "]":
                delta = None
                target = high
            elif key == "m":
                delta = None
                target = int((low + high) / 2)
            elif key == "r":
                print_state(bus, "read", low, high, target)
                stop_if_goal_drifted(bus, args)
                continue
            elif key == "x":
                write_raw(bus, "Torque_Enable", 0, retries=3)
                print("torque off")
                continue
            elif key == "e":
                present = read_raw(bus)
                target = present
                torque_on(bus, target, torque_limit)
                print_state(bus, "torque on", low, high, target)
                guard_hold(bus, target, args)
                continue
            else:
                continue

            if not torque_enabled(bus):
                print("torque is off; press e to re-enable at the current encoder position")
                continue

            before = read_raw(bus)
            if target is None:
                target = clamp(before + delta, low, high)
                if before < low and target <= before:
                    print(f"outside low limit ({before} < {low}); refusing farther-low command")
                    continue
                if before > high and target >= before:
                    print(f"outside high limit ({before} > {high}); refusing farther-high command")
                    continue

            target = clamp(target, low, high)
            if args.direct_goal:
                write_raw(bus, "Goal_Position", target, retries=3)
                time.sleep(max(0.0, args.settle_sec))
                after = read_raw(bus)
                status = "direct"
            else:
                before, after, status = controlled_move_to(bus, target, low, high, args)
                time.sleep(max(0.0, args.settle_sec))
            print(f"key={key!r} raw_target={target} observed_delta={after - before:+d} status={status}")
            print_state(bus, "cmd", low, high, target)
            if load_is_saturated(bus, torque_limit):
                print(
                    "WARNING: elbow load is at the torque limit. "
                    "The servo may drift or ignore small commands; raise --torque-limit or unload the arm."
                )
        except Exception as exc:
            print(f"\nStopping control because elbow_flex reported an error: {type(exc).__name__}: {exc}")
            print("Power-cycle the arm before running another motion command.")
            break


def run_test_step(bus: FeetechMotorsBus, low: int, high: int, args: argparse.Namespace) -> None:
    phase = resolve_phase(args)
    if phase is not None and phase >= 0:
        torque_off(bus)
        write_raw(bus, "Phase", int(phase), retries=3)
        print(f"Wrote Feetech Phase={phase}")

    present = read_raw(bus)
    target = clamp(present + int(args.test_step), low, high)
    torque_limit = clamp(int(args.torque_limit), 0, 1000)

    torque_on(bus, present, torque_limit)
    print_state(bus, "before test", low, high, present)

    print(f"test command: {present} -> {target} (delta={target - present:+d})")
    write_raw(bus, "Goal_Position", target, retries=3)
    time.sleep(max(0.0, args.hold_sec))
    print_state(bus, "after test", low, high, target)


def resolve_limits(bus: FeetechMotorsBus, args: argparse.Namespace) -> tuple[int, int]:
    if args.low is not None or args.high is not None:
        if args.low is None or args.high is None:
            raise ValueError("Use both --low and --high, or neither.")
        low, high = int(args.low), int(args.high)
    elif args.use_saved:
        low, high = load_limits(args.limits_file)
    else:
        low, high = record_limits(bus, args.sample_hz)
        save_limits(args.limits_file, low, high, args.port, args.motor_id)
        print(f"Saved limits to {args.limits_file}")

    if high < low:
        low, high = high, low
    if high - low < 30:
        raise ValueError(f"Unsafe tiny range: low={low}, high={high}")
    return low, high


def main() -> int:
    args = parse_args()
    logging.getLogger().setLevel(logging.ERROR)

    bus = make_bus(args.port, args.motor_id)
    print(f"Opening {MOTOR_NAME} only on {args.port}, id={args.motor_id}...")
    bus.connect()
    try:
        low, high = resolve_limits(bus, args)
        if args.test_step is None:
            manual_control(bus, low, high, args)
        else:
            run_test_step(bus, low, high, args)
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
