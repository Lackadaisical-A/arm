#!/usr/bin/env python
r"""Manual teach-and-replay for a calibrated SO-101 follower.

Run from PowerShell after activating the LeRobot venv:
    python .\scripts_local\manual_record_replay_so101.py --port COM4
"""

from __future__ import annotations

import argparse
import json
import logging
import msvcrt
import time
from datetime import datetime
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record SO-101 by hand, then replay the movement.")
    parser.add_argument("--port", required=True, help="Robot serial port, for example COM4.")
    parser.add_argument("--robot-id", default="my_so101_follower", help="LeRobot calibration id.")
    parser.add_argument("--fps", type=float, default=30.0, help="Recording and replay rate.")
    parser.add_argument(
        "--max-relative-target",
        type=float,
        default=12.0,
        help="LeRobot safety cap per replay command. Lower is safer, higher follows faster motions.",
    )
    parser.add_argument(
        "--start-tolerance",
        type=float,
        default=8.0,
        help="Warn if the arm is this far from the recorded start before playback.",
    )
    parser.add_argument(
        "--save-dir",
        default="recordings",
        help="Folder, relative to the LeRobot checkout, where JSON recordings are saved.",
    )
    add_elbow_phase_arg(parser)
    return parser.parse_args()


def print_controls() -> None:
    print(
        """
Manual record/replay

  1. Torque is disabled during recording.
  2. Move the arm by hand to teach the motion.
  3. Press p to stop recording.
  4. Move the arm back to the starting pose by hand.
  5. Press Enter to replay with torque enabled.

Keys while recording:
  p      stop recording and prepare playback
  q/ESC  quit without playback
"""
    )


def read_positions(robot: SO101Follower) -> dict[str, float]:
    obs = robot.get_observation()
    return {motor: float(obs[f"{motor}.pos"]) for motor in MOTORS}


def action_from_positions(positions: dict[str, float]) -> dict[str, float]:
    return {f"{motor}.pos": value for motor, value in positions.items()}


def format_positions(positions: dict[str, float]) -> str:
    return " | ".join(f"{motor}:{positions[motor]:7.2f}" for motor in MOTORS)


def max_abs_delta(a: dict[str, float], b: dict[str, float]) -> float:
    return max(abs(a[motor] - b[motor]) for motor in MOTORS)


def save_recording(save_dir: Path, port: str, robot_id: str, fps: float, trajectory: list[dict]) -> Path:
    save_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = save_dir / f"so101_manual_{stamp}.json"
    payload = {
        "robot": "so101_follower",
        "port": port,
        "robot_id": robot_id,
        "fps": fps,
        "motors": MOTORS,
        "frames": trajectory,
    }
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return path


def record_trajectory(robot: SO101Follower, fps: float) -> list[dict]:
    interval = 1.0 / fps
    start_t = time.perf_counter()
    next_t = start_t
    frames: list[dict] = []

    print("Recording now. Move the arm by hand. Press p to stop, q to quit.")
    while True:
        now = time.perf_counter()
        if now >= next_t:
            positions = read_positions(robot)
            frames.append({"t": now - start_t, "positions": positions})
            if len(frames) % max(1, int(fps)) == 0:
                print(f"\rRecorded {len(frames)} frames / {frames[-1]['t']:.1f}s", end="", flush=True)
            next_t += interval

        if msvcrt.kbhit():
            key = msvcrt.getwch().lower()
            if key in ("\x00", "\xe0"):
                _ = msvcrt.getwch()
                continue
            if key == "p":
                print()
                return frames
            if key in ("q", "\x1b"):
                print()
                return []

        time.sleep(0.001)


def wait_for_start_pose(robot: SO101Follower, start_pose: dict[str, float], tolerance: float) -> None:
    print("\nMove the arm back to the starting pose by hand.")
    print("Start pose:", format_positions(start_pose))
    input("When it is back at the start, press Enter to replay...")

    current = read_positions(robot)
    error = max_abs_delta(current, start_pose)
    if error > tolerance:
        print(
            f"Warning: current pose is still up to {error:.2f} units from the recorded start "
            f"(tolerance {tolerance:.2f})."
        )
        answer = input("Press Enter to continue anyway, or type q then Enter to cancel: ").strip().lower()
        if answer == "q":
            raise KeyboardInterrupt


def replay_trajectory(robot: SO101Follower, trajectory: list[dict], fps: float) -> None:
    interval = 1.0 / fps
    print("Enabling torque and replaying...")
    robot.bus.enable_torque()

    start_t = time.perf_counter()
    last_print_t = start_t
    for idx, frame in enumerate(trajectory):
        loop_t = time.perf_counter()
        robot.send_action(action_from_positions(frame["positions"]))

        if loop_t - last_print_t >= 1.0:
            print(f"\rPlaying frame {idx + 1}/{len(trajectory)}", end="", flush=True)
            last_print_t = loop_t

        next_t = start_t + ((idx + 1) * interval)
        sleep_s = next_t - time.perf_counter()
        if sleep_s > 0:
            time.sleep(sleep_s)

    print(f"\rPlaying frame {len(trajectory)}/{len(trajectory)}")


def main() -> int:
    args = parse_args()
    logging.getLogger().setLevel(logging.ERROR)

    robot = SO101Follower(
        SO101FollowerConfig(
            port=args.port,
            id=args.robot_id,
            max_relative_target={motor: args.max_relative_target for motor in MOTORS},
        )
    )

    print(f"Connecting to SO-101 follower on {args.port} with id '{args.robot_id}'...")
    robot.connect()
    apply_elbow_phase(
        robot,
        args.elbow_phase,
        torque_limit=args.elbow_torque_limit,
        startup_force=args.elbow_startup_force,
        p_coefficient=args.elbow_p_coefficient,
    )

    try:
        print_controls()
        print("Disabling torque. Hold the arm before it goes limp.")
        robot.bus.disable_torque()
        time.sleep(0.2)

        trajectory = record_trajectory(robot, args.fps)
        if not trajectory:
            print("No playback. Exiting.")
            return 0

        save_path = save_recording(Path(args.save_dir), args.port, args.robot_id, args.fps, trajectory)
        print(f"Saved {len(trajectory)} frames to {save_path}")

        start_pose = trajectory[0]["positions"]
        wait_for_start_pose(robot, start_pose, args.start_tolerance)
        replay_trajectory(robot, trajectory, args.fps)

        print("Replay complete. Torque will be disabled on disconnect.")
        return 0
    except KeyboardInterrupt:
        print("\nCanceled.")
        return 1
    finally:
        robot.disconnect()
        print("Disconnected. Torque disabled.")


if __name__ == "__main__":
    raise SystemExit(main())
