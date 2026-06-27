#!/usr/bin/env python
r"""Interactive tuner for SO-101 RViz joint offsets and scales.

Run while display_real_state + publish_lerobot_state_udp are running:
    python .\scripts_local\tune_so101_rviz_offsets.py
"""

from __future__ import annotations

import argparse
import json
import msvcrt
from pathlib import Path


MOTORS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
DEFAULT_FLIPS = dict.fromkeys(MOTORS, False)
DEFAULT_FLIPS["elbow_flex"] = True


def default_config_path() -> Path:
    return Path(__file__).resolve().parents[1] / "config" / "so101_joint_mapping.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Tune SO-101 RViz display offsets and scales.")
    parser.add_argument("--config", type=Path, default=default_config_path())
    parser.add_argument("--small-step", type=float, default=1.0, help="Small offset step in degrees.")
    parser.add_argument("--big-step", type=float, default=5.0, help="Big offset step in degrees.")
    parser.add_argument("--scale-small-step", type=float, default=0.01, help="Small scale step.")
    parser.add_argument("--scale-big-step", type=float, default=0.05, help="Big scale step.")
    return parser.parse_args()


def load_config(path: Path) -> dict:
    config = json.loads(path.read_text(encoding="utf-8"))
    config.setdefault("joint_flips", {})
    config.setdefault("command_flips", {})
    config.setdefault("urdf_offsets_deg", {})
    config.setdefault("urdf_scales", {})
    for motor in MOTORS:
        config["joint_flips"].setdefault(motor, DEFAULT_FLIPS[motor])
        config["command_flips"].setdefault(motor, False)
        config["urdf_offsets_deg"].setdefault(motor, 0.0)
        config["urdf_scales"].setdefault(motor, 1.0)
    return config


def save_config(path: Path, config: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")


def print_help(selected: str, config: dict) -> None:
    print(
        """
SO-101 RViz offset tuner

Run this while the display-only RViz mirror is active. Select a joint, then
adjust its URDF display offset and scale until RViz matches the real arm.

Keys:
  1..6   select joint
  a/d    offset -/+ small step
  A/D    offset -/+ big step
  j/l    scale -/+ small step
  J/L    scale -/+ big step
  f      toggle sign flip for selected joint
  g      toggle command direction for selected joint
  p      print all offsets
  r      reset selected joint offset to 0, scale to 1, and flip default
  q/ESC  quit
"""
    )
    print_status(selected, config)


def print_status(selected: str, config: dict) -> None:
    offsets = config["urdf_offsets_deg"]
    scales = config["urdf_scales"]
    flips = config["joint_flips"]
    command_flips = config["command_flips"]
    print(
        f"Selected: {selected} | offset={offsets[selected]:+.2f} deg | "
        f"scale={scales[selected]:.4f} | visual_flip={flips[selected]} | "
        f"command_flip={command_flips[selected]}"
    )


def print_all(config: dict) -> None:
    print("\nCurrent mapping:")
    for motor in MOTORS:
        print(
            f"  {motor:14s} offset={config['urdf_offsets_deg'][motor]:+7.2f} deg "
            f"scale={config['urdf_scales'][motor]:7.4f} "
            f"visual_flip={config['joint_flips'][motor]} "
            f"command_flip={config['command_flips'][motor]}"
        )


def main() -> int:
    args = parse_args()
    config = load_config(args.config)
    selected = "elbow_flex"
    print(f"Editing {args.config}")
    print_help(selected, config)

    while True:
        key = msvcrt.getwch()
        if key in ("\x00", "\xe0"):
            _ = msvcrt.getwch()
            continue

        if key in ("q", "\x1b"):
            print("\nDone.")
            return 0

        if key in "123456":
            selected = MOTORS[int(key) - 1]
            print_status(selected, config)
            continue

        if key in ("a", "d", "A", "D"):
            step = args.big_step if key in ("A", "D") else args.small_step
            direction = -1.0 if key in ("a", "A") else 1.0
            config["urdf_offsets_deg"][selected] += direction * step
            save_config(args.config, config)
            print_status(selected, config)
            continue

        if key in ("j", "l", "J", "L"):
            step = args.scale_big_step if key in ("J", "L") else args.scale_small_step
            direction = -1.0 if key in ("j", "J") else 1.0
            config["urdf_scales"][selected] = max(0.05, config["urdf_scales"][selected] + direction * step)
            save_config(args.config, config)
            print_status(selected, config)
            continue

        if key == "f":
            config["joint_flips"][selected] = not config["joint_flips"][selected]
            save_config(args.config, config)
            print_status(selected, config)
            continue

        if key == "g":
            config["command_flips"][selected] = not config["command_flips"][selected]
            save_config(args.config, config)
            print_status(selected, config)
            continue

        if key == "r":
            config["urdf_offsets_deg"][selected] = 0.0
            config["urdf_scales"][selected] = 1.0
            config["joint_flips"][selected] = DEFAULT_FLIPS[selected]
            config["command_flips"][selected] = False
            save_config(args.config, config)
            print_status(selected, config)
            continue

        if key == "p":
            print_all(config)
            continue

        if key == "h" or key == "?":
            print_help(selected, config)


if __name__ == "__main__":
    raise SystemExit(main())
