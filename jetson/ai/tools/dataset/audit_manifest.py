#!/usr/bin/env python3
"""Validate a reviewed full-frame occupancy manifest and report balance."""

from __future__ import annotations

import argparse
import csv
from collections import Counter
from pathlib import Path
import sys


SLOTS = (
    "C1_L3", "C2_L3", "C3_L3",
    "C1_L2", "C2_L2", "C3_L2",
    "C1_L1", "C2_L1", "C3_L1",
)
VALID_STATES = {"EMPTY", "OCCUPIED"}
VALID_SPLITS = {"train", "validation", "test"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument(
        "--dataset-root",
        type=Path,
        help="If set, verify every source_relpath below this directory.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    with args.manifest.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))

    errors: list[str] = []
    ids = [row.get("capture_id", "") for row in rows]
    duplicates = sorted(name for name, count in Counter(ids).items() if count > 1)
    if duplicates:
        errors.append(f"duplicate capture_id: {duplicates}")

    expected_ids = [f"a{number:03d}" for number in range(1, 51)]
    if sorted(ids) != expected_ids:
        errors.append("capture IDs are not exactly a001..a050")

    split_counts: Counter[str] = Counter()
    split_occupied: Counter[str] = Counter()
    slot_occupied: dict[str, Counter[str]] = {split: Counter() for split in VALID_SPLITS}
    for row_number, row in enumerate(rows, start=2):
        split = row.get("proposed_split", "")
        if split not in VALID_SPLITS:
            errors.append(f"line {row_number}: invalid split {split!r}")
            continue
        states = [row.get(slot, "") for slot in SLOTS]
        bad_states = sorted(set(states) - VALID_STATES)
        if bad_states:
            errors.append(f"line {row_number}: invalid states {bad_states}")
            continue
        occupied = states.count("OCCUPIED")
        if row.get("occupied_count") != str(occupied):
            errors.append(
                f"line {row_number}: occupied_count={row.get('occupied_count')}, actual={occupied}"
            )
        split_counts[split] += 1
        split_occupied[split] += occupied
        for slot, state in zip(SLOTS, states):
            slot_occupied[split][slot] += state == "OCCUPIED"
        if args.dataset_root is not None:
            source = args.dataset_root / row.get("source_relpath", "")
            if not source.is_file():
                errors.append(f"line {row_number}: missing source {source}")

    expected_split_counts = {"train": 30, "validation": 10, "test": 10}
    if dict(split_counts) != expected_split_counts:
        errors.append(f"split counts {dict(split_counts)} != {expected_split_counts}")

    if errors:
        for error in errors:
            print(f"FAIL {error}", file=sys.stderr)
        return 1

    print(f"PASS manifest rows: {len(rows)} unique captures")
    for split in ("train", "validation", "test"):
        roi_total = split_counts[split] * len(SLOTS)
        occupied = split_occupied[split]
        print(
            f"PASS {split}: frames={split_counts[split]}, "
            f"occupied={occupied}/{roi_total} ({occupied / roi_total:.1%}), "
            f"empty={roi_total - occupied}/{roi_total}"
        )
        print("  " + " ".join(f"{slot}={slot_occupied[split][slot]}" for slot in SLOTS))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
