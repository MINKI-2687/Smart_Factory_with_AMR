#!/usr/bin/env python3
"""Create the immutable v2 base manifest from audited correction ledgers.

The v1 manifest is read-only input. The output must not already exist. Every
label correction and consumed-test split move is verified against its source
row and SHA-256 evidence before the new manifest is written.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import csv
import hashlib
import json
from pathlib import Path
import sys


SLOTS = (
    "C1_L3", "C2_L3", "C3_L3",
    "C1_L2", "C2_L2", "C3_L2",
    "C1_L1", "C2_L1", "C3_L1",
)
LABELS = {"EMPTY", "OCCUPIED"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    return parser.parse_args()


def read_csv(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"Missing CSV header: {path}")
        return list(reader.fieldnames), list(reader)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def rooted_path(dataset_root: Path, relative: str) -> Path:
    candidate = (dataset_root / relative).resolve()
    if not candidate.is_relative_to(dataset_root):
        raise ValueError(f"Path escapes dataset root: {relative}")
    return candidate


def require_hash(dataset_root: Path, relative: str, expected: str, kind: str) -> None:
    path = rooted_path(dataset_root, relative)
    if not path.is_file():
        raise ValueError(f"Missing {kind}: {path}")
    actual = sha256_file(path)
    if actual != expected:
        raise ValueError(
            f"{kind} SHA-256 mismatch for {relative}: expected={expected}, actual={actual}"
        )


def main() -> int:
    args = parse_args()
    dataset_root = args.dataset_root.resolve()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    if int(config.get("schema_version", 0)) != 2:
        raise ValueError("This tool only accepts schema_version=2")

    source_path = rooted_path(dataset_root, config["source_manifest_before_correction"])
    correction_path = rooted_path(dataset_root, config["label_correction_ledger"])
    split_path = rooted_path(dataset_root, config["split_consumption_ledger"])
    output_path = rooted_path(dataset_root, config["split_manifests"]["train"])
    if output_path.exists():
        raise ValueError(f"Refusing to overwrite corrected manifest: {output_path}")

    fields, source_rows = read_csv(source_path)
    _, corrections = read_csv(correction_path)
    _, split_moves = read_csv(split_path)
    rows_by_id = {row["capture_id"]: dict(row) for row in source_rows}
    if len(rows_by_id) != len(source_rows):
        raise ValueError("Duplicate capture_id in source manifest")

    expected_train_ids = set(config["expected_capture_ids"]["train"])
    if set(rows_by_id) != expected_train_ids:
        raise ValueError("Source manifest IDs do not exactly match v2 train IDs")
    if len(corrections) != 8:
        raise ValueError(f"Expected exactly 8 semantic label corrections, got {len(corrections)}")

    correction_ids = [row["correction_id"] for row in corrections]
    correction_keys = [(row["capture_id"], row["slot"]) for row in corrections]
    if len(correction_ids) != len(set(correction_ids)):
        raise ValueError("Duplicate correction_id")
    if len(correction_keys) != len(set(correction_keys)):
        raise ValueError("Duplicate capture_id/slot correction")

    grouped: dict[str, list[dict[str, str]]] = defaultdict(list)
    for correction in corrections:
        capture_id = correction["capture_id"]
        slot = correction["slot"]
        if capture_id not in rows_by_id:
            raise ValueError(f"Unknown corrected capture: {capture_id}")
        if slot not in SLOTS:
            raise ValueError(f"Unknown corrected slot: {slot}")
        if correction["old_label"] not in LABELS or correction["new_label"] not in LABELS:
            raise ValueError(f"Invalid correction labels: {correction['correction_id']}")
        if correction["old_label"] == correction["new_label"]:
            raise ValueError(f"No-op correction: {correction['correction_id']}")
        source_row = rows_by_id[capture_id]
        if source_row[slot] != correction["old_label"]:
            raise ValueError(f"Old label mismatch: {correction['correction_id']}")
        if source_row["source_relpath"] != correction["source_relpath"]:
            raise ValueError(f"Source path mismatch: {correction['correction_id']}")
        if int(source_row["occupied_count"]) != int(correction["old_occupied_count"]):
            raise ValueError(f"Old occupied_count mismatch: {correction['correction_id']}")
        require_hash(
            dataset_root,
            correction["source_relpath"],
            correction["source_sha256"],
            "source frame",
        )
        require_hash(
            dataset_root,
            correction["evidence_relpath"],
            correction["evidence_sha256"],
            "visual evidence",
        )
        if correction["review_status"] != "verified_from_source_image":
            raise ValueError(f"Unverified correction: {correction['correction_id']}")
        grouped[capture_id].append(correction)

    for capture_id, capture_corrections in grouped.items():
        row = rows_by_id[capture_id]
        new_counts = {int(item["new_occupied_count"]) for item in capture_corrections}
        if len(new_counts) != 1:
            raise ValueError(f"Inconsistent new occupied_count for {capture_id}")
        for correction in capture_corrections:
            row[correction["slot"]] = correction["new_label"]
        observed = sum(row[slot] == "OCCUPIED" for slot in SLOTS)
        expected = new_counts.pop()
        if observed != expected:
            raise ValueError(
                f"Corrected occupied_count mismatch for {capture_id}: {observed} != {expected}"
            )
        row["occupied_count"] = str(observed)
        row["label_source"] = "manual_visual_reaudit_20260801"
        row["review_status"] = "corrected_after_full_visual_reaudit"

    retired_ids = set(config["retired_test_capture_ids"])
    move_ids = {row["capture_id"] for row in split_moves}
    if move_ids != retired_ids or len(split_moves) != len(retired_ids):
        raise ValueError("Split ledger does not exactly match retired test IDs")
    for move in split_moves:
        capture_id = move["capture_id"]
        row = rows_by_id.get(capture_id)
        if row is None:
            raise ValueError(f"Unknown split move capture: {capture_id}")
        if move["old_split"] != "test" or move["new_split"] != "train":
            raise ValueError(f"Invalid consumed-test move: {capture_id}")
        if row.get("proposed_split") != move["old_split"]:
            raise ValueError(f"Old proposed_split mismatch: {capture_id}")
        if move["never_test_again"].lower() != "true":
            raise ValueError(f"Retired capture is not locked: {capture_id}")
        if move["prior_test_report_sha256"] != config["retired_test_report_sha256"]:
            raise ValueError(f"Retired test report hash mismatch: {capture_id}")
        if row["source_relpath"] != move["source_relpath"]:
            raise ValueError(f"Split ledger source path mismatch: {capture_id}")
        require_hash(
            dataset_root,
            move["source_relpath"],
            move["source_sha256"],
            "retired test source frame",
        )
        row["proposed_split"] = "train"

    output_rows = [rows_by_id[row["capture_id"]] for row in source_rows]
    occupied = sum(row[slot] == "OCCUPIED" for row in output_rows for slot in SLOTS)
    empty = len(output_rows) * len(SLOTS) - occupied
    if (occupied, empty) != (232, 218):
        raise ValueError(f"Unexpected corrected class totals: occupied={occupied}, empty={empty}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_name(f".{output_path.name}.tmp")
    if temporary.exists():
        raise ValueError(f"Stale temporary output exists: {temporary}")
    try:
        with temporary.open("x", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(output_rows)
        temporary.rename(output_path)
    except Exception:
        if temporary.exists():
            temporary.unlink()
        raise

    print(f"PASS source manifest preserved: {source_path}")
    print("PASS semantic corrections: 8 across a014/a031/a039/a041")
    print(f"PASS consumed test moves: {len(split_moves)} test->train, locked never_test_again")
    print(f"PASS corrected original ROI classes: OCCUPIED={occupied} EMPTY={empty}")
    print(f"PASS wrote new manifest: {output_path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
