from __future__ import annotations

import json
from argparse import ArgumentParser
from pathlib import Path

from lerobot.robots.so_follower import SO101Follower


DEFAULT_PHASE_FILE = Path(__file__).resolve().parents[1] / "config" / "elbow_flex_phase.json"
DEFAULT_ELBOW_TORQUE_LIMIT = 1000
DEFAULT_ELBOW_MIN_STARTUP_FORCE = 800
DEFAULT_ELBOW_P_COEFFICIENT = 32


def load_saved_elbow_phase(path: Path = DEFAULT_PHASE_FILE) -> int | None:
    if not path.exists():
        return None
    data = json.loads(path.read_text(encoding="utf-8"))
    return int(data["phase"])


def add_elbow_phase_arg(parser: ArgumentParser, default: int | None = 12) -> None:
    parser.add_argument(
        "--elbow-phase",
        type=int,
        default=default,
        help="Feetech Phase value to force on elbow_flex before motion. Default is 12. Use -1 to skip.",
    )


def apply_elbow_phase(robot: SO101Follower, phase: int | None) -> None:
    if phase is None:
        phase = load_saved_elbow_phase()
        if phase is None:
            return
    if phase < 0:
        return
    torque_was_enabled = bool(int(robot.bus.read("Torque_Enable", "elbow_flex", normalize=False)))
    robot.bus.write("Torque_Enable", "elbow_flex", 0, normalize=False, num_retry=3)
    try:
        robot.bus.write("Lock", "elbow_flex", 0, normalize=False, num_retry=3)
    except Exception:
        pass
    robot.bus.write("Phase", "elbow_flex", int(phase), normalize=False, num_retry=3)
    robot.bus.write("P_Coefficient", "elbow_flex", DEFAULT_ELBOW_P_COEFFICIENT, normalize=False, num_retry=3)
    robot.bus.write(
        "Minimum_Startup_Force",
        "elbow_flex",
        DEFAULT_ELBOW_MIN_STARTUP_FORCE,
        normalize=False,
        num_retry=3,
    )
    robot.bus.write("Torque_Limit", "elbow_flex", DEFAULT_ELBOW_TORQUE_LIMIT, normalize=False, num_retry=3)
    actual = int(robot.bus.read("Phase", "elbow_flex", normalize=False))
    if actual != phase:
        raise RuntimeError(f"Failed to set elbow_flex Phase={phase}; servo reports Phase={actual}.")
    if torque_was_enabled:
        robot.bus.enable_torque("elbow_flex", num_retry=3)
    print(
        f"elbow_flex Feetech Phase set to {actual}; "
        f"torque_limit={DEFAULT_ELBOW_TORQUE_LIMIT}, "
        f"startup={DEFAULT_ELBOW_MIN_STARTUP_FORCE}."
    )
