#!/usr/bin/env python
r"""Record raw low/high, then manually control one Feetech servo.

This is intentionally independent from SO101Follower. It opens one raw Feetech
motor id and uses raw Present_Position / Goal_Position values only.

Example:
    python .\scripts_local\single_servo_record_control.py --port COM6 --motor-name wrist_flex --motor-id 4
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


try:
    import msvcrt
except ImportError:  # pragma: no cover - Windows is the expected target here.
    msvcrt = None


DEFAULT_CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"
SO101_IDS = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record/control one raw Feetech servo.")
    parser.add_argument("--port", required=True, help="Serial port, for example COM6.")
    parser.add_argument("--motor-name", required=True, help="Label for this servo, for example wrist_flex.")
    parser.add_argument("--motor-id", type=int, required=True, help="Feetech servo id.")
    parser.add_argument("--model", default="sts3215", help="Feetech motor model.")
    parser.add_argument("--low", type=int, default=None, help="Use this raw low limit instead of recording.")
    parser.add_argument("--high", type=int, default=None, help="Use this raw high limit instead of recording.")
    parser.add_argument("--use-saved", action="store_true", help="Load limits from --limits-file.")
    parser.add_argument("--limits-file", type=Path, default=None, help="JSON file for saved low/high.")
    parser.add_argument("--sample-hz", type=float, default=20.0, help="Recording sample rate.")
    parser.add_argument("--step-ticks", type=int, default=20, help="Keyboard jog size in raw ticks.")
    parser.add_argument("--torque-limit", type=int, default=None, help="Session torque limit, 0-1000. Default is 1000 for elbow id 3, otherwise 300.")
    parser.add_argument("--p-coefficient", type=int, default=None, help="Position P coefficient. Default is 16 for elbow id 3, otherwise 32.")
    parser.add_argument("--min-startup-force", type=int, default=None, help="Minimum_Startup_Force. Default is 16 for elbow id 3, otherwise 80. Use 0 to leave unchanged.")
    parser.add_argument("--acceleration", type=int, default=254, help="Acceleration for this session.")
    parser.add_argument("--maximum-acceleration", type=int, default=254, help="Maximum_Acceleration for this session.")
    parser.add_argument("--maximum-velocity-limit", type=int, default=None, help="Optional Maximum_Velocity_Limit to write.")
    parser.add_argument("--max-torque-limit", type=int, default=None, help="Optional Max_Torque_Limit to write.")
    parser.add_argument("--goal-time", type=int, default=0, help="Goal_Time for this session.")
    parser.add_argument("--goal-velocity", type=int, default=0, help="Goal_Velocity for this session. 0 uses servo default/max behavior.")
    parser.add_argument("--settle-sec", type=float, default=0.05, help="Delay after each command.")
    parser.add_argument("--sample-sec", type=float, default=0.02, help="Encoder sample interval while moving.")
    parser.add_argument("--move-timeout", type=float, default=0.8, help="Maximum seconds for one jog.")
    parser.add_argument("--stop-tolerance", type=int, default=4, help="Raw ticks allowed around each jog target.")
    parser.add_argument("--hold-guard-sec", type=float, default=2.0, help="Seconds to verify the servo can hold after torque-on.")
    parser.add_argument("--hold-guard-ticks", type=int, default=120, help="Torque-off if hold drifts by this many raw ticks.")
    parser.add_argument(
        "--start-edge-margin",
        type=int,
        default=120,
        help="If starting this close to a saved limit, ask for a hand move to the middle first. Use 0 to disable.",
    )
    parser.add_argument("--phase", type=int, default=None, help="Optional Feetech Phase to write. Omit to leave unchanged.")
    parser.add_argument("--direct-goal", action="store_true", help="Write raw target and let servo PID stop itself.")
    parser.add_argument(
        "--leave-goal-on-timeout",
        action="store_true",
        help="Do not rewrite Goal_Position to current position when a jog times out.",
    )
    parser.add_argument("--debug-goal-write", action="store_true", help="Print Goal_Position immediately after each write.")
    parser.add_argument("--test-step", type=int, default=None, help="One raw delta test, then torque off and exit.")
    return parser.parse_args()


def clamp(value: int, low: int, high: int) -> int:
    return min(high, max(low, value))


def limits_file_for(args: argparse.Namespace) -> Path:
    if args.limits_file is not None:
        return args.limits_file
    safe_name = args.motor_name.replace("/", "_").replace("\\", "_")
    return DEFAULT_CONFIG_DIR / f"{safe_name}_manual_limits.json"


def load_limits(path: Path) -> tuple[int, int]:
    data = json.loads(path.read_text(encoding="utf-8"))
    low, high = int(data["low"]), int(data["high"])
    return (low, high) if low <= high else (high, low)


def save_limits(path: Path, low: int, high: int, args: argparse.Namespace) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "motor": args.motor_name,
        "motor_id": args.motor_id,
        "port": args.port,
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


def make_bus(args: argparse.Namespace) -> FeetechMotorsBus:
    return FeetechMotorsBus(
        port=args.port,
        motors={args.motor_name: Motor(args.motor_id, args.model, MotorNormMode.DEGREES)},
    )


def read_raw(bus: FeetechMotorsBus, motor: str, field: str = "Present_Position") -> int:
    return int(bus.read(field, motor, normalize=False))


def write_raw(bus: FeetechMotorsBus, motor: str, field: str, value: int, retries: int = 3) -> None:
    bus.write(field, motor, int(value), normalize=False, num_retry=retries)


def try_read(bus: FeetechMotorsBus, motor: str, field: str) -> str:
    try:
        return str(read_raw(bus, motor, field))
    except Exception as exc:
        return f"<{type(exc).__name__}: {exc}>"


def torque_off(bus: FeetechMotorsBus, motor: str) -> None:
    write_raw(bus, motor, "Torque_Enable", 0)
    try:
        write_raw(bus, motor, "Lock", 0)
    except Exception:
        pass


def torque_on(bus: FeetechMotorsBus, motor: str, target: int, torque_limit: int) -> None:
    write_raw(bus, motor, "Torque_Limit", torque_limit)
    write_raw(bus, motor, "Goal_Position", target)
    write_raw(bus, motor, "Torque_Enable", 1)
    try:
        write_raw(bus, motor, "Lock", 1)
    except Exception:
        pass


def configure_session(bus: FeetechMotorsBus, motor: str, args: argparse.Namespace, torque_limit: int) -> None:
    write_raw(bus, motor, "Operating_Mode", 0)
    if args.max_torque_limit is not None:
        write_raw(bus, motor, "Max_Torque_Limit", clamp(int(args.max_torque_limit), 0, 1000))
    write_raw(bus, motor, "Torque_Limit", torque_limit)
    write_raw(bus, motor, "P_Coefficient", clamp(int(args.p_coefficient), 0, 254))
    write_raw(bus, motor, "Goal_Time", clamp(int(args.goal_time), 0, 65535))
    write_raw(bus, motor, "Goal_Velocity", clamp(int(args.goal_velocity), 0, 32767))
    if args.maximum_velocity_limit is not None:
        write_raw(bus, motor, "Maximum_Velocity_Limit", clamp(int(args.maximum_velocity_limit), 0, 254))
    write_raw(bus, motor, "Maximum_Acceleration", clamp(int(args.maximum_acceleration), 0, 254))
    write_raw(bus, motor, "Acceleration", clamp(int(args.acceleration), 0, 254))
    if int(args.min_startup_force) > 0:
        write_raw(bus, motor, "Minimum_Startup_Force", clamp(int(args.min_startup_force), 0, 1000))


def print_state(bus: FeetechMotorsBus, motor: str, label: str, low: int, high: int, target: int | None = None) -> None:
    present = read_raw(bus, motor)
    pct = 100.0 * (present - low) / max(1, high - low)
    target_text = "" if target is None else f" target={target}"
    print(
        f"{label}: present={present} ({pct:5.1f}%) "
        f"goal={try_read(bus, motor, 'Goal_Position')}{target_text} "
        f"torque={try_read(bus, motor, 'Torque_Enable')} "
        f"load={try_read(bus, motor, 'Present_Load')} "
        f"current={try_read(bus, motor, 'Present_Current')} "
        f"phase={try_read(bus, motor, 'Phase')} "
        f"mode={try_read(bus, motor, 'Operating_Mode')} "
        f"status={try_read(bus, motor, 'Status')} "
        f"torque_limit={try_read(bus, motor, 'Torque_Limit')} "
        f"goal_time={try_read(bus, motor, 'Goal_Time')} "
        f"goal_vel={try_read(bus, motor, 'Goal_Velocity')} "
        f"max_vel={try_read(bus, motor, 'Maximum_Velocity_Limit')} "
        f"max_accel={try_read(bus, motor, 'Maximum_Acceleration')} "
        f"p={try_read(bus, motor, 'P_Coefficient')} "
        f"startup={try_read(bus, motor, 'Minimum_Startup_Force')}"
    )


def torque_enabled(bus: FeetechMotorsBus, motor: str) -> bool:
    return bool(int(read_raw(bus, motor, "Torque_Enable")))


def hold_here(bus: FeetechMotorsBus, motor: str) -> int:
    present = read_raw(bus, motor)
    write_raw(bus, motor, "Goal_Position", present)
    return present


def guard_hold(bus: FeetechMotorsBus, motor: str, goal: int, args: argparse.Namespace) -> bool:
    deadline = time.perf_counter() + max(0.0, args.hold_guard_sec)
    while time.perf_counter() < deadline:
        time.sleep(max(0.02, args.sample_sec))
        present = read_raw(bus, motor)
        if abs(present - goal) >= abs(args.hold_guard_ticks):
            print(f"HOLD FAILED: goal={goal} present={present} drift={present - goal:+d}. Torque off.")
            torque_off(bus, motor)
            return False
    return True


def ensure_middle_start(bus: FeetechMotorsBus, motor: str, low: int, high: int, args: argparse.Namespace) -> int:
    present = read_raw(bus, motor)
    if args.start_edge_margin <= 0:
        return present
    margin = max(abs(args.start_edge_margin), int((high - low) * 0.08))
    near_or_outside_low = present <= low + margin
    near_or_outside_high = present >= high - margin
    if not (near_or_outside_low or near_or_outside_high):
        return present

    torque_off(bus, motor)
    middle = int((low + high) / 2)
    print(
        f"Start pose is too close to a saved limit: present={present}, low={low}, high={high}. "
        f"Move the servo by hand near raw {middle}, then press ENTER."
    )
    input()
    return read_raw(bus, motor)


def controlled_move_to(
    bus: FeetechMotorsBus,
    motor: str,
    target: int,
    low: int,
    high: int,
    args: argparse.Namespace,
) -> tuple[int, int, str]:
    start = read_raw(bus, motor)
    target = clamp(target, low, high)
    if abs(target - start) <= args.stop_tolerance:
        stopped = hold_here(bus, motor)
        return start, stopped, "already-there"

    direction = 1 if target > start else -1
    write_raw(bus, motor, "Goal_Position", target)
    if args.debug_goal_write:
        print(f"wrote goal target={target}; servo reports goal={read_raw(bus, motor, 'Goal_Position')}")

    status = "timeout"
    deadline = time.perf_counter() + max(0.05, args.move_timeout)
    while time.perf_counter() < deadline:
        time.sleep(max(0.005, args.sample_sec))
        current = read_raw(bus, motor)
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
        if current < low and direction < 0:
            status = "outside-limits"
            break
        if current > high and direction > 0:
            status = "outside-limits"
            break

    if status == "timeout" and args.leave_goal_on_timeout:
        stopped = read_raw(bus, motor)
    else:
        stopped = hold_here(bus, motor)
    return start, stopped, status


def record_limits(bus: FeetechMotorsBus, motor: str, args: argparse.Namespace) -> tuple[int, int]:
    torque_off(bus, motor)
    input("Torque is off. Move this servo to a safe middle position, then press ENTER to start recording.")
    print("Move the servo through a safe physical range. Press ENTER again to stop.")

    low: int | None = None
    high: int | None = None
    samples = 0
    interval = 1.0 / max(1.0, args.sample_hz)
    while True:
        pos = read_raw(bus, motor)
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
        raise RuntimeError(f"Recorded range is too small: low={low}, high={high}.")
    return low, high


def resolve_limits(bus: FeetechMotorsBus, args: argparse.Namespace) -> tuple[int, int]:
    if args.low is not None or args.high is not None:
        if args.low is None or args.high is None:
            raise ValueError("Use both --low and --high, or neither.")
        low, high = int(args.low), int(args.high)
    elif args.use_saved:
        low, high = load_limits(limits_file_for(args))
    else:
        low, high = record_limits(bus, args.motor_name, args)
        path = limits_file_for(args)
        save_limits(path, low, high, args)
        print(f"Saved limits to {path}")
    return (low, high) if low <= high else (high, low)


def get_key() -> str:
    if msvcrt is None:
        return sys.stdin.read(1)
    key = msvcrt.getwch()
    if key in ("\x00", "\xe0"):
        return ""
    return key


def run_test_step(bus: FeetechMotorsBus, args: argparse.Namespace, low: int, high: int) -> None:
    motor = args.motor_name
    present = read_raw(bus, motor)
    target = clamp(present + int(args.test_step), low, high)
    torque_limit = clamp(int(args.torque_limit), 0, 1000)
    configure_session(bus, motor, args, torque_limit)
    torque_on(bus, motor, present, torque_limit)
    print_state(bus, motor, "before test", low, high, present)
    print(f"test command: {present} -> {target} (delta={target - present:+d})")
    write_raw(bus, motor, "Goal_Position", target)
    time.sleep(max(0.2, args.settle_sec))
    print_state(bus, motor, "after test", low, high, target)


def manual_control(bus: FeetechMotorsBus, args: argparse.Namespace, low: int, high: int) -> None:
    motor = args.motor_name
    if args.phase is not None:
        torque_off(bus, motor)
        write_raw(bus, motor, "Phase", int(args.phase))
        print(f"Wrote Feetech Phase={args.phase}")

    torque_limit = clamp(int(args.torque_limit), 0, 1000)
    configure_session(bus, motor, args, torque_limit)

    target = ensure_middle_start(bus, motor, low, high, args)
    torque_on(bus, motor, target, torque_limit)

    print()
    print(
        f"Using {motor} id={args.motor_id} raw limits: low={low}, high={high}, "
        f"start target={target}, torque_limit={torque_limit}, p={args.p_coefficient}, "
        f"startup={args.min_startup_force}"
    )
    print("Keys: j/l = raw -/+, J/L = raw -/+ 5x, [/] = low/high, m = middle")
    print("      1/2 = raw -/+ 1, r = read, x = torque off, e = torque on, q = quit")
    print_state(bus, motor, "start", low, high, target)
    guard_hold(bus, motor, target, args)

    while True:
        key = get_key()
        if not key:
            continue
        try:
            target = None
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
                print_state(bus, motor, "read", low, high, target)
                continue
            elif key == "x":
                torque_off(bus, motor)
                print("torque off")
                continue
            elif key == "e":
                target = read_raw(bus, motor)
                torque_on(bus, motor, target, torque_limit)
                print_state(bus, motor, "torque on", low, high, target)
                guard_hold(bus, motor, target, args)
                continue
            else:
                continue

            if not torque_enabled(bus, motor):
                print("torque is off; press e to re-enable at the current encoder position")
                continue

            before = read_raw(bus, motor)
            if target is None:
                target = clamp(before + delta, low, high)
                if before < low and target <= before:
                    print(f"outside low limit ({before} < {low}); refusing farther-low command")
                    continue
                if before > high and target >= before:
                    print(f"outside high limit ({before} > {high}); refusing farther-high command")
                    continue

            if args.direct_goal:
                write_raw(bus, motor, "Goal_Position", target)
                if args.debug_goal_write:
                    print(f"wrote goal target={target}; servo reports goal={read_raw(bus, motor, 'Goal_Position')}")
                time.sleep(max(0.0, args.settle_sec))
                after = read_raw(bus, motor)
                status = "direct"
            else:
                before, after, status = controlled_move_to(bus, motor, target, low, high, args)
                time.sleep(max(0.0, args.settle_sec))
            print(f"key={key!r} raw_target={target} observed_delta={after - before:+d} status={status}")
            print_state(bus, motor, "cmd", low, high, target)
        except Exception as exc:
            print(f"\nStopping because {motor} reported an error: {type(exc).__name__}: {exc}")
            print("Power-cycle before running another motion command.")
            break


def main() -> int:
    args = parse_args()
    logging.getLogger().setLevel(logging.ERROR)
    expected_name = SO101_IDS.get(args.motor_id)
    if expected_name is not None and args.motor_name != expected_name:
        print(
            f"WARNING: SO101 id {args.motor_id} is normally '{expected_name}', "
            f"but you passed --motor-name {args.motor_name!r}."
        )
        print("The servo id controls the hardware. The name only controls labels and the saved limits filename.")
    if args.motor_id == 3 and args.phase is None:
        args.phase = 12
        print("Defaulting SO101 id 3 / elbow_flex to Feetech Phase=12.")
    if args.torque_limit is None:
        args.torque_limit = 1000 if args.motor_id == 3 else 300
    if args.p_coefficient is None:
        args.p_coefficient = 16 if args.motor_id == 3 else 32
    if args.min_startup_force is None:
        args.min_startup_force = 16 if args.motor_id == 3 else 80

    bus = make_bus(args)
    print(f"Opening {args.motor_name} only on {args.port}, id={args.motor_id}...")
    bus.connect()
    try:
        low, high = resolve_limits(bus, args)
        if args.test_step is not None:
            run_test_step(bus, args, low, high)
        else:
            manual_control(bus, args, low, high)
    finally:
        try:
            torque_off(bus, args.motor_name)
            print("Torque off.")
        except Exception as exc:
            print(f"Torque-off failed: {type(exc).__name__}: {exc}")
            print("Power-cycle the arm.")
        bus.disconnect(disable_torque=False)
        print("Closed serial port.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
