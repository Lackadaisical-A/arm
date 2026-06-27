#!/usr/bin/env python
"""Scan a Feetech servo bus and report responding IDs."""

from __future__ import annotations

import argparse
import logging

from lerobot.motors.feetech import FeetechMotorsBus


EXPECTED_SO101 = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Scan Feetech servo IDs on a COM port.")
    parser.add_argument("--port", required=True, help="Serial port, for example COM6.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logging.getLogger().setLevel(logging.ERROR)

    print(f"Scanning {args.port} for Feetech servos...")
    found_by_baud = FeetechMotorsBus.scan_port(args.port)
    if not found_by_baud:
        print("No motors found. Check the COM port, USB cable, controller jumpers, and external servo power.")
        return 1

    all_ids = sorted({id_ for ids in found_by_baud.values() for id_ in ids})
    print("\nFound IDs:", all_ids)
    print("Expected SO-101 IDs:", sorted(EXPECTED_SO101))

    missing = [id_ for id_ in EXPECTED_SO101 if id_ not in all_ids]
    extra = [id_ for id_ in all_ids if id_ not in EXPECTED_SO101]

    if missing:
        print("\nMissing expected IDs:")
        for id_ in missing:
            print(f"  ID {id_}: {EXPECTED_SO101[id_]}")

    if extra:
        print("\nUnexpected IDs:")
        for id_ in extra:
            print(f"  ID {id_}")

    if not missing and not extra:
        print("\nAll expected SO-101 IDs are visible.")
    else:
        print("\nFix missing/unexpected IDs before running teleop.")

    return 0 if not missing else 2


if __name__ == "__main__":
    raise SystemExit(main())
