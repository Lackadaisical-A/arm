#!/usr/bin/env python
r"""Publish real SO-101 follower joint state to WSL/RViz over UDP.

Run from Windows PowerShell after activating the LeRobot venv:
    python .\scripts_local\publish_lerobot_state_udp.py --port COM6

This is display-only. It does not send goal positions and does not enable torque.
"""

from __future__ import annotations

import argparse
import json
import logging
import socket
import subprocess
import time
from pathlib import Path

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


MOTORS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
BODY_MOTORS = MOTORS[:-1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish real SO-101 state to RViz over UDP.")
    parser.add_argument("--port", required=True, help="Robot serial port, for example COM6.")
    parser.add_argument("--robot-id", default="my_so101_follower", help="LeRobot calibration id.")
    parser.add_argument("--target-ip", default="auto", help="WSL IP address, or 'auto'.")
    parser.add_argument("--target-port", type=int, default=50102, help="WSL UDP receive port.")
    parser.add_argument("--rate", type=float, default=20.0, help="Publish rate in Hz.")
    parser.add_argument("--wsl-distro", default="Ubuntu-24.04", help="WSL distro name for auto IP detection.")
    parser.add_argument(
        "--mapping-config",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config" / "so101_joint_mapping.json",
        help="Joint mapping offset config.",
    )
    return parser.parse_args()


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
    positions = robot.bus.sync_read("Present_Position")
    return {motor: float(positions[motor]) for motor in MOTORS}


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


def format_positions(positions: dict[str, float]) -> str:
    return " | ".join(f"{motor}:{positions[motor]:7.2f}" for motor in MOTORS)


class MappingConfig:
    def __init__(self, path: Path):
        self.path = path
        self.mtime = 0.0
        self.data = self._default()
        self.reload(force=True)

    def _default(self) -> dict:
        data = {
            "joint_flips": dict.fromkeys(MOTORS, False),
            "urdf_offsets_deg": dict.fromkeys(MOTORS, 0.0),
            "urdf_scales": dict.fromkeys(MOTORS, 1.0),
        }
        data["joint_flips"]["elbow_flex"] = True
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
            self.data["urdf_offsets_deg"].update(loaded.get("urdf_offsets_deg", {}))
            self.data["urdf_scales"].update(loaded.get("urdf_scales", {}))
            self.mtime = mtime
            print(f"\nLoaded mapping config: {self.path}")

        return self.data


def main() -> int:
    args = parse_args()
    logging.getLogger().setLevel(logging.ERROR)

    target_ip = detect_wsl_ip(args.wsl_distro) if args.target_ip == "auto" else args.target_ip
    target = (target_ip, args.target_port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    robot = SO101Follower(SO101FollowerConfig(port=args.port, id=args.robot_id))
    mapping_config = MappingConfig(args.mapping_config)

    print(f"Connecting read-only to SO-101 follower on {args.port} with id '{args.robot_id}'...")
    robot.bus.connect()
    limits_deg = lerobot_limits_deg(robot)
    print(f"Publishing real state to udp://{target_ip}:{args.target_port} at {args.rate:.1f} Hz")
    print("This is display-only: no goal positions are sent.")

    interval = 1.0 / args.rate
    seq = 0
    last_print_t = 0.0

    try:
        while True:
            loop_t = time.perf_counter()
            positions = read_positions(robot)
            mapping = mapping_config.reload()
            payload = {
                "seq": seq,
                "stamp": time.time(),
                "positions": positions,
                "limits_deg": limits_deg,
                "mapping": mapping,
            }
            sock.sendto(json.dumps(payload).encode("utf-8"), target)

            if loop_t - last_print_t >= 1.0:
                print(f"\rseq={seq} | {format_positions(positions)}", end="", flush=True)
                last_print_t = loop_t

            seq += 1
            sleep_s = interval - (time.perf_counter() - loop_t)
            if sleep_s > 0:
                time.sleep(sleep_s)
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        robot.bus.disconnect(disable_torque=False)
        sock.close()
        print("Disconnected read-only bus.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
