#!/usr/bin/env python
r"""Receive RViz/ROS2 joint targets over UDP and command a LeRobot SO-101 follower.

Start this from Windows PowerShell after activating the LeRobot venv:
    python .\scripts_local\rviz_udp_lerobot_bridge.py --port COM6
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import msvcrt
import socket
import subprocess
import time
from pathlib import Path

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
BODY_MOTORS = MOTORS[:-1]
URDF_LIMITS_RAD = {
    "shoulder_pan": [-1.91986, 1.91986],
    "shoulder_lift": [-1.74533, 1.74533],
    "elbow_flex": [-1.69, 1.69],
    "wrist_flex": [-1.65806, 1.65806],
    "wrist_roll": [-2.74385, 2.84121],
    "gripper": [-0.174533, 1.74533],
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Bridge RViz joint sliders to a real SO-101 follower.")
    parser.add_argument("--port", required=True, help="Robot serial port, for example COM6.")
    parser.add_argument("--robot-id", default="my_so101_follower", help="LeRobot calibration id.")
    parser.add_argument("--udp-bind", default="0.0.0.0", help="Address to listen on.")
    parser.add_argument("--udp-port", type=int, default=50101, help="UDP port to listen on.")
    parser.add_argument("--state-target-ip", default="auto", help="WSL IP for real-state feedback, or 'auto'.")
    parser.add_argument("--state-target-port", type=int, default=50102, help="WSL UDP real-state receive port.")
    parser.add_argument("--state-rate", type=float, default=15.0, help="Real-state feedback publish rate in Hz.")
    parser.add_argument("--wsl-distro", default="Ubuntu-24.04", help="WSL distro name for auto IP detection.")
    parser.add_argument("--no-state-publish", action="store_true", help="Disable real-state feedback to RViz.")
    parser.add_argument("--max-rate", type=float, default=50.0, help="Maximum hardware command rate.")
    parser.add_argument("--max-target-age", type=float, default=0.5, help="Ignore stale UDP targets after this many seconds.")
    parser.add_argument(
        "--control-mode",
        choices=["direct", "relative"],
        default="direct",
        help="direct follows slider positions immediately; relative uses latch-and-delta control.",
    )
    parser.add_argument(
        "--command-deadband",
        type=float,
        default=0.25,
        help="Slider change required before a joint becomes active, in LeRobot deg/percent units.",
    )
    parser.add_argument(
        "--send-all-direct-joints",
        action="store_true",
        help="In direct mode, command every joint every cycle. Default sends only changed sliders to reduce servo hunting.",
    )
    parser.add_argument(
        "--send-all-joints",
        action="store_true",
        help="Old behavior: command every joint from every RViz target.",
    )
    parser.add_argument(
        "--max-relative-target",
        type=float,
        default=180.0,
        help="LeRobot safety cap per command. Lower is safer; higher follows sliders faster.",
    )
    parser.add_argument("--debug-targets", action="store_true", help="Print target/current values for moved joints.")
    add_elbow_phase_arg(parser)
    parser.add_argument(
        "--accept-urdf-targets",
        action="store_true",
        help="Accept legacy URDF/radian slider packets. Default ignores them to avoid stale RViz senders.",
    )
    parser.add_argument("--dry-run", action="store_true", help="Print targets without commanding servos.")
    parser.add_argument(
        "--mapping-config",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config" / "so101_joint_mapping.json",
        help="Joint mapping offset config.",
    )
    parser.add_argument(
        "--elbow-limits-file",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config" / "elbow_flex_manual_limits.json",
        help="Manual raw elbow_flex safe limits JSON. Missing file is ignored.",
    )
    return parser.parse_args()


def print_help() -> None:
    print(
        """
Windows LeRobot RViz bridge

Keys in this PowerShell window:
  e      enable / pause command streaming
         direct mode follows the full slider pose immediately
         relative mode latches the current RViz pose; move a slider after enabling to command that joint
         pause disables torque
  c      clear active sliders and latch the current RViz pose again
  t      disable servo torque immediately
  p      print current robot pose and latest RViz target
  h      show help
  q/ESC  quit and disable torque

Start paused. Press e first, then move one RViz slider gently.
"""
    )


def detect_wsl_ip(distro: str) -> str:
    result = subprocess.run(
        ["wsl.exe", "-d", distro, "--", "hostname", "-I"],
        check=True,
        capture_output=True,
        text=True,
    )
    ips = [part.strip() for part in result.stdout.split() if part.strip()]
    if not ips:
        raise RuntimeError(f"Could not detect IP for WSL distro '{distro}'.")
    return ips[0]


def read_positions(robot: SO101Follower) -> dict[str, float]:
    obs = robot.get_observation()
    return {motor: float(obs[f"{motor}.pos"]) for motor in MOTORS}


def lerobot_limits_deg(robot: SO101Follower) -> dict[str, list[float]]:
    limits = {}
    for motor in BODY_MOTORS:
        calibration = robot.calibration[motor]
        model = robot.bus.motors[motor].model
        max_res = robot.bus.model_resolution_table[model] - 1
        mid = (calibration.range_min + calibration.range_max) / 2
        low = (calibration.range_min - mid) * 360 / max_res
        high = (calibration.range_max - mid) * 360 / max_res
        limits[motor] = [float(low), float(high)]
    limits["gripper"] = [0.0, 100.0]
    return limits


def raw_to_lerobot_deg(robot: SO101Follower, motor: str, raw_value: int) -> float:
    calibration = robot.calibration[motor]
    model = robot.bus.motors[motor].model
    max_res = robot.bus.model_resolution_table[model] - 1
    mid = (calibration.range_min + calibration.range_max) / 2
    return float((int(raw_value) - mid) * 360 / max_res)


def apply_manual_elbow_limits(
    robot: SO101Follower,
    limits_deg: dict[str, list[float]],
    path: Path,
) -> dict[str, list[float]]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return limits_deg

    raw_low = int(data["low"])
    raw_high = int(data["high"])
    if raw_low == raw_high:
        raise ValueError(f"Invalid elbow limit file {path}: low and high are equal.")

    deg_low = raw_to_lerobot_deg(robot, "elbow_flex", raw_low)
    deg_high = raw_to_lerobot_deg(robot, "elbow_flex", raw_high)
    limits_deg["elbow_flex"] = [min(deg_low, deg_high), max(deg_low, deg_high)]
    print(
        "Using manual elbow_flex safe limits "
        f"raw=[{min(raw_low, raw_high)}, {max(raw_low, raw_high)}] "
        f"lerobot=[{limits_deg['elbow_flex'][0]:+.2f}, {limits_deg['elbow_flex'][1]:+.2f}] "
        f"from {path}"
    )
    return limits_deg


def clamp(value: float, low: float, high: float) -> float:
    return min(high, max(low, value))


class MappingConfig:
    def __init__(self, path: Path):
        self.path = path
        self.mtime = 0.0
        self.data = self._default()
        self.reload(force=True)

    def _default(self) -> dict:
        data = {
            "joint_flips": dict.fromkeys(MOTORS, False),
            "command_flips": dict.fromkeys(MOTORS, False),
            "urdf_offsets_deg": dict.fromkeys(MOTORS, 0.0),
            "urdf_scales": dict.fromkeys(MOTORS, 1.0),
        }
        data["joint_flips"]["elbow_flex"] = False
        return data

    def reload(self, force: bool = False) -> dict:
        try:
            mtime = self.path.stat().st_mtime
        except FileNotFoundError:
            return self.data

        if force or mtime != self.mtime:
            loaded = json.loads(self.path.read_text(encoding="utf-8"))
            self.data = self._default()
            self.data["joint_flips"].update(loaded.get("joint_flips", {}))
            self.data["command_flips"].update(loaded.get("command_flips", {}))
            self.data["urdf_offsets_deg"].update(loaded.get("urdf_offsets_deg", {}))
            self.data["urdf_scales"].update(loaded.get("urdf_scales", {}))
            self.mtime = mtime
            print(f"\nLoaded mapping config: {self.path}")

        return self.data


def urdf_rad_to_lerobot_target(
    urdf_positions: dict[str, float], limits_deg: dict[str, list[float]], mapping: dict
) -> dict[str, float]:
    target = {}
    offsets = mapping["urdf_offsets_deg"]
    scales = mapping["urdf_scales"]
    flips = mapping["joint_flips"]
    for motor in BODY_MOTORS:
        urdf_low, urdf_high = URDF_LIMITS_RAD[motor]
        lerobot_low, lerobot_high = limits_deg[motor]
        offset_rad = math.radians(float(offsets[motor]))
        scale = float(scales[motor])
        if scale == 0.0:
            scale = 1.0
        urdf_value = urdf_low + (float(urdf_positions[motor]) - offset_rad - urdf_low) / scale
        urdf_value = clamp(urdf_value, urdf_low, urdf_high)
        ratio = (urdf_value - urdf_low) / (urdf_high - urdf_low)
        if flips[motor]:
            ratio = 1.0 - ratio
        target[motor] = lerobot_low + ratio * (lerobot_high - lerobot_low)

    gripper_low, gripper_high = URDF_LIMITS_RAD["gripper"]
    gripper_rad = clamp(float(urdf_positions["gripper"]), gripper_low, gripper_high)
    gripper_ratio = (gripper_rad - gripper_low) / (gripper_high - gripper_low)
    target["gripper"] = gripper_ratio * 100.0
    return target


def command_target_from_latch(
    latest_target: dict[str, float],
    armed_reference: dict[str, float],
    command_base: dict[str, float],
    active_motors: list[str],
    limits_deg: dict[str, list[float]],
    command_flips: dict[str, bool],
) -> dict[str, float]:
    target = {motor: float(command_base[motor]) for motor in MOTORS if motor in command_base}
    for motor in active_motors:
        if motor not in latest_target or motor not in armed_reference or motor not in command_base:
            continue

        delta = float(latest_target[motor]) - float(armed_reference[motor])
        if command_flips[motor]:
            delta = -delta

        low, high = limits_deg[motor]
        target[motor] = clamp(float(command_base[motor]) + delta, low, high)
    return target


def send_real_state(
    sock: socket.socket,
    target: tuple[str, int],
    seq: int,
    robot: SO101Follower,
    limits_deg: dict[str, list[float]],
    mapping: dict,
) -> None:
    positions = read_positions(robot)
    payload = {
        "seq": seq,
        "stamp": time.time(),
        "positions": positions,
        "limits_deg": limits_deg,
        "mapping": mapping,
    }
    sock.sendto(json.dumps(payload).encode("utf-8"), target)


def format_positions(values: dict[str, float] | None) -> str:
    if not values:
        return "<none>"
    return " | ".join(f"{motor}:{values[motor]:7.2f}" for motor in MOTORS if motor in values)


def ordered_motors(motors: set[str]) -> list[str]:
    return [motor for motor in MOTORS if motor in motors]


def changed_motors(
    target: dict[str, float], reference: dict[str, float] | None, deadband: float
) -> set[str]:
    if reference is None:
        return set()
    return {
        motor
        for motor in MOTORS
        if motor in target and motor in reference and abs(float(target[motor]) - float(reference[motor])) >= deadband
    }


def action_from_target(target: dict[str, float], motors: list[str] | None = None) -> dict[str, float]:
    if motors is None:
        motors = MOTORS
    return {f"{motor}.pos": float(target[motor]) for motor in motors if motor in target}


def drain_udp(
    sock: socket.socket, limits_deg: dict[str, list[float]], mapping: dict, accept_urdf_targets: bool
) -> tuple[dict[str, float] | None, float | None, int | None, str | None]:
    latest_target = None
    latest_rx_t = None
    latest_seq = None
    latest_source = None
    while True:
        try:
            payload, _addr = sock.recvfrom(65535)
        except BlockingIOError:
            break

        try:
            message = json.loads(payload.decode("utf-8"))
            if "urdf_positions_rad" in message and all(
                motor in message["urdf_positions_rad"] for motor in MOTORS
            ):
                if accept_urdf_targets:
                    latest_target = urdf_rad_to_lerobot_target(message["urdf_positions_rad"], limits_deg, mapping)
                    latest_rx_t = time.perf_counter()
                    latest_seq = int(message.get("seq", -1))
                    latest_source = "urdf"
            elif "positions" in message and all(motor in message["positions"] for motor in MOTORS):
                positions = message["positions"]
                latest_target = {
                    motor: clamp(float(positions[motor]), limits_deg[motor][0], limits_deg[motor][1])
                    for motor in MOTORS
                }
                latest_rx_t = time.perf_counter()
                latest_seq = int(message.get("seq", -1))
                latest_source = "lerobot"
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            continue

    return latest_target, latest_rx_t, latest_seq, latest_source


def main() -> int:
    args = parse_args()
    logging.getLogger().setLevel(logging.ERROR)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.udp_bind, args.udp_port))
    sock.setblocking(False)

    state_sock = None
    state_target = None
    if not args.no_state_publish:
        state_target_ip = detect_wsl_ip(args.wsl_distro) if args.state_target_ip == "auto" else args.state_target_ip
        state_target = (state_target_ip, args.state_target_port)
        state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    robot = SO101Follower(
        SO101FollowerConfig(
            port=args.port,
            id=args.robot_id,
            max_relative_target=float(args.max_relative_target),
        )
    )

    print(f"Listening for RViz targets on udp://{args.udp_bind}:{args.udp_port}")
    if state_target is not None:
        print(f"Publishing real state to udp://{state_target[0]}:{state_target[1]} at {args.state_rate:.1f} Hz")
    print(f"Connecting to SO-101 follower on {args.port} with id '{args.robot_id}'...")
    robot.connect()
    apply_elbow_phase(
        robot,
        args.elbow_phase,
        torque_limit=args.elbow_torque_limit,
        startup_force=args.elbow_startup_force,
        p_coefficient=args.elbow_p_coefficient,
    )
    limits_deg = lerobot_limits_deg(robot)
    limits_deg = apply_manual_elbow_limits(robot, limits_deg, args.elbow_limits_file)
    mapping_config = MappingConfig(args.mapping_config)

    enabled = False
    latest_target: dict[str, float] | None = None
    latest_rx_t: float | None = None
    latest_seq: int | None = None
    latest_source: str | None = None
    last_send_t = 0.0
    last_debug_t = 0.0
    last_state_t = 0.0
    state_seq = 0
    last_print_t = 0.0
    armed_reference: dict[str, float] | None = None
    command_base: dict[str, float] | None = None
    active_motors: set[str] = set()
    last_direct_target: dict[str, float] | None = None
    interval = 1.0 / args.max_rate

    try:
        print_help()
        print("Current robot:", format_positions(read_positions(robot)))

        while True:
            mapping = mapping_config.reload()
            target, rx_t, seq, source = drain_udp(sock, limits_deg, mapping, args.accept_urdf_targets)
            if target is not None:
                latest_target = target
                latest_rx_t = rx_t
                latest_seq = seq
                latest_source = source
                if enabled and args.control_mode == "direct" and last_direct_target is None:
                    last_direct_target = target.copy()
                elif enabled and args.control_mode != "direct" and not args.send_all_joints:
                    if armed_reference is None:
                        armed_reference = target.copy()
                        command_base = read_positions(robot)
                        active_motors.clear()
                    else:
                        active_motors.update(changed_motors(target, armed_reference, args.command_deadband))
                elif enabled and args.control_mode != "direct" and args.send_all_joints and armed_reference is None:
                    armed_reference = target.copy()
                    command_base = read_positions(robot)

            if msvcrt.kbhit():
                key = msvcrt.getwch().lower()
                if key in ("\x00", "\xe0"):
                    _ = msvcrt.getwch()
                    continue
                if key in ("q", "\x1b"):
                    print("\nQuitting.")
                    break
                if key == "h":
                    print_help()
                elif key == "e":
                    enabled = not enabled
                    if enabled:
                        robot.bus.enable_torque()
                        armed_reference = latest_target.copy() if latest_target is not None else None
                        command_base = read_positions(robot)
                        active_motors.clear()
                        last_direct_target = latest_target.copy() if latest_target is not None else None
                        if args.control_mode == "direct":
                            if args.send_all_direct_joints:
                                print("Streaming ENABLED. Direct slider follow mode, sending all joints.")
                            else:
                                print("Streaming ENABLED. Direct slider follow mode, sending changed sliders only.")
                        elif args.send_all_joints:
                            print("Streaming ENABLED. Relative mode, sending all RViz joints.")
                        else:
                            print("Streaming ENABLED. Relative mode: RViz pose latched; move a slider to command that joint.")
                    else:
                        robot.bus.disable_torque()
                        armed_reference = None
                        command_base = None
                        active_motors.clear()
                        last_direct_target = None
                        print("Streaming PAUSED. Torque disabled.")
                elif key == "c":
                    armed_reference = latest_target.copy() if latest_target is not None else None
                    command_base = read_positions(robot)
                    active_motors.clear()
                    last_direct_target = latest_target.copy() if latest_target is not None else None
                    print("Cleared active sliders. Current RViz pose latched.")
                elif key == "t":
                    enabled = False
                    robot.bus.disable_torque()
                    armed_reference = None
                    command_base = None
                    active_motors.clear()
                    last_direct_target = None
                    print("Torque disabled. Streaming paused.")
                elif key == "p":
                    print("Current robot:", format_positions(read_positions(robot)))
                    print("Latest target:", format_positions(latest_target), f"seq={latest_seq} source={latest_source}")
                    print("Active sliders:", ", ".join(ordered_motors(active_motors)) or "<none>")

            now = time.perf_counter()
            target_is_fresh = latest_rx_t is not None and (now - latest_rx_t) <= args.max_target_age
            if args.control_mode == "direct":
                if args.send_all_direct_joints:
                    command_motors = MOTORS
                elif latest_target is not None and last_direct_target is not None:
                    command_motors = ordered_motors(
                        changed_motors(latest_target, last_direct_target, args.command_deadband)
                    )
                else:
                    command_motors = []
                should_send = (
                    enabled
                    and latest_target
                    and target_is_fresh
                    and bool(command_motors)
                    and (now - last_send_t) >= interval
                )
            else:
                command_motors = MOTORS if args.send_all_joints else ordered_motors(active_motors)
                should_send = (
                    enabled
                    and latest_target
                    and armed_reference
                    and command_base
                    and target_is_fresh
                    and bool(command_motors)
                    and (now - last_send_t) >= interval
                )

            if should_send:
                if args.control_mode == "direct":
                    command_target = latest_target
                else:
                    command_target = command_target_from_latch(
                        latest_target,
                        armed_reference,
                        command_base,
                        command_motors,
                        limits_deg,
                        mapping["command_flips"],
                    )
                if args.dry_run:
                    pass
                else:
                    robot.send_action(action_from_target(command_target, command_motors))
                if args.control_mode == "direct" and latest_target is not None:
                    if last_direct_target is None:
                        last_direct_target = latest_target.copy()
                    for motor in command_motors:
                        last_direct_target[motor] = float(latest_target[motor])
                if args.debug_targets and now - last_debug_t >= 0.2 and "elbow_flex" in command_motors:
                    current = read_positions(robot)
                    print(
                        "\nTarget elbow/current elbow: "
                        f"{command_target['elbow_flex']:+.2f} / {current['elbow_flex']:+.2f} "
                        f"(source={latest_source})"
                    )
                    last_debug_t = now
                last_send_t = now

            if (
                state_sock is not None
                and state_target is not None
                and args.state_rate > 0
                and (now - last_state_t) >= (1.0 / args.state_rate)
            ):
                send_real_state(state_sock, state_target, state_seq, robot, limits_deg, mapping)
                state_seq += 1
                last_state_t = now

            if now - last_print_t >= 1.0:
                state = "ENABLED" if enabled else "PAUSED"
                freshness = "fresh" if target_is_fresh else "stale/no target"
                if args.control_mode == "direct":
                    if args.send_all_direct_joints:
                        detail = "direct=all"
                    else:
                        active = ",".join(command_motors) if "command_motors" in locals() and command_motors else "none"
                        detail = f"direct changed={active}"
                else:
                    active = ",".join(ordered_motors(active_motors)) if active_motors else "none"
                    detail = f"active={active}"
                print(
                    f"\r{state} | target {freshness} | {detail} | seq={latest_seq} source={latest_source}",
                    end="",
                    flush=True,
                )
                last_print_t = now

            time.sleep(0.005)

    finally:
        print()
        robot.disconnect()
        sock.close()
        if state_sock is not None:
            state_sock.close()
        print("Disconnected. Torque disabled.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
