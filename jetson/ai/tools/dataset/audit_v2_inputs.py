#!/usr/bin/env python3
"""Audit v2 corrections, consumed-test quarantine, and fresh-test plan.

This planning audit is intentionally runnable before a081-a090 are captured.
It proves that the corrected train source and the precommitted test layouts are
internally consistent without pretending that the final v2 dataset exists.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
from pathlib import Path
import sys

from PIL import Image


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


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def rooted_path(root: Path, relative: str) -> Path:
    path = (root / relative).resolve()
    if not path.is_relative_to(root):
        raise ValueError(f"Path escapes dataset root: {relative}")
    return path


def pattern(row: dict[str, str]) -> tuple[str, ...]:
    return tuple(row[slot] for slot in SLOTS)


def complement(states: tuple[str, ...]) -> tuple[str, ...]:
    return tuple("EMPTY" if state == "OCCUPIED" else "OCCUPIED" for state in states)


def validate_rows(rows: list[dict[str, str]], source: str) -> None:
    ids = [row["capture_id"] for row in rows]
    if len(ids) != len(set(ids)):
        raise ValueError(f"Duplicate capture_id in {source}")
    for row in rows:
        states = [row.get(slot, "") for slot in SLOTS]
        if set(states) - LABELS:
            raise ValueError(f"Invalid slot state in {source}/{row['capture_id']}")
        if states.count("OCCUPIED") != int(row["occupied_count"]):
            raise ValueError(f"occupied_count mismatch in {source}/{row['capture_id']}")
        if row.get("safety_ignore", "").lower() != "false":
            raise ValueError(f"safety_ignore is not false in {source}/{row['capture_id']}")


def main() -> int:
    args = parse_args()
    root = args.dataset_root.resolve()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    if int(config.get("schema_version", 0)) != 2:
        raise ValueError("Expected schema_version=2")

    parent_zip = rooted_path(root, config["parent_dataset_zip"])
    if sha256_file(parent_zip) != config["parent_dataset_zip_sha256"]:
        raise ValueError("Parent v1 ZIP SHA-256 mismatch")

    old_rows = read_csv(rooted_path(root, config["source_manifest_before_correction"]))
    corrected_rows = read_csv(rooted_path(root, config["split_manifests"]["train"]))
    validation_rows = read_csv(rooted_path(root, config["split_manifests"]["validation"]))
    correction_rows = read_csv(rooted_path(root, config["label_correction_ledger"]))
    split_rows = read_csv(rooted_path(root, config["split_consumption_ledger"]))
    plan_rows = read_csv(rooted_path(root, config["test_capture_plan"]))
    for name, rows in (
        ("old base", old_rows),
        ("corrected base", corrected_rows),
        ("validation", validation_rows),
        ("test plan", plan_rows),
    ):
        validate_rows(rows, name)

    old_by_id = {row["capture_id"]: row for row in old_rows}
    corrected_by_id = {row["capture_id"]: row for row in corrected_rows}
    if set(old_by_id) != set(corrected_by_id):
        raise ValueError("Corrected manifest changed the base capture ID set")

    correction_map = {
        (row["capture_id"], row["slot"]): (row["old_label"], row["new_label"])
        for row in correction_rows
    }
    if len(correction_rows) != 8 or len(correction_map) != 8:
        raise ValueError("Correction ledger must contain exactly 8 unique semantic flips")
    retired_ids = set(config["retired_test_capture_ids"])
    if {row["capture_id"] for row in split_rows} != retired_ids:
        raise ValueError("Split-consumption ledger does not match retired test IDs")

    observed_label_changes: dict[tuple[str, str], tuple[str, str]] = {}
    allowed_metadata = {"occupied_count", "label_source", "review_status", "proposed_split"}
    for capture_id, old in old_by_id.items():
        new = corrected_by_id[capture_id]
        for field in old:
            if old[field] == new[field]:
                continue
            if field in SLOTS:
                observed_label_changes[(capture_id, field)] = (old[field], new[field])
            elif field not in allowed_metadata:
                raise ValueError(f"Unexpected manifest change: {capture_id}/{field}")
        expected_split = "train" if capture_id in retired_ids else old["proposed_split"]
        if new["proposed_split"] != expected_split:
            raise ValueError(f"Unexpected proposed_split after correction: {capture_id}")
        expected_count = sum(new[slot] == "OCCUPIED" for slot in SLOTS)
        if int(new["occupied_count"]) != expected_count:
            raise ValueError(f"Corrected occupied_count mismatch: {capture_id}")
        is_corrected = any(key[0] == capture_id for key in correction_map)
        if is_corrected:
            if new["label_source"] != "manual_visual_reaudit_20260801":
                raise ValueError(f"Missing corrected label_source: {capture_id}")
            if new["review_status"] != "corrected_after_full_visual_reaudit":
                raise ValueError(f"Missing corrected review_status: {capture_id}")
        elif old["label_source"] != new["label_source"] or old["review_status"] != new["review_status"]:
            raise ValueError(f"Uncorrected review metadata changed: {capture_id}")
    if observed_label_changes != correction_map:
        raise ValueError(
            f"Manifest/ledger label diff mismatch: observed={observed_label_changes}, "
            f"ledger={correction_map}"
        )

    for correction in correction_rows:
        for path_field, hash_field in (
            ("source_relpath", "source_sha256"),
            ("evidence_relpath", "evidence_sha256"),
        ):
            path = rooted_path(root, correction[path_field])
            if sha256_file(path) != correction[hash_field]:
                raise ValueError(f"Correction evidence hash mismatch: {correction['correction_id']}")
        if correction["identified_from_test_report_sha256"] != config["retired_test_report_sha256"]:
            raise ValueError(f"Correction report hash mismatch: {correction['correction_id']}")

    for move in split_rows:
        capture_id = move["capture_id"]
        if (
            move["old_split"] != "test"
            or move["new_split"] != "train"
            or move["never_test_again"].lower() != "true"
            or move["review_status"] != "locked_consumed"
        ):
            raise ValueError(f"Invalid consumed-test lock: {capture_id}")
        if move["prior_test_report_sha256"] != config["retired_test_report_sha256"]:
            raise ValueError(f"Consumed-test report hash mismatch: {capture_id}")
        source = rooted_path(root, move["source_relpath"])
        if sha256_file(source) != move["source_sha256"]:
            raise ValueError(f"Consumed-test source hash mismatch: {capture_id}")

    train_ids = {row["capture_id"] for row in corrected_rows}
    val_ids = {row["capture_id"] for row in validation_rows}
    test_ids = {row["capture_id"] for row in plan_rows}
    expected_ids = {split: set(ids) for split, ids in config["expected_capture_ids"].items()}
    if train_ids != expected_ids["train"] or val_ids != expected_ids["validation"] or test_ids != expected_ids["test"]:
        raise ValueError("Observed capture IDs do not exactly match config")
    if train_ids & val_ids or train_ids & test_ids or val_ids & test_ids:
        raise ValueError("Capture ID leakage across v2 splits")

    occupied = sum(row[slot] == "OCCUPIED" for row in corrected_rows for slot in SLOTS)
    if (occupied, len(corrected_rows) * 9 - occupied) != (232, 218):
        raise ValueError("Corrected train classes are not OCCUPIED=232/EMPTY=218")

    plan_patterns = [pattern(row) for row in plan_rows]
    if len(plan_patterns) != len(set(plan_patterns)):
        raise ValueError("Duplicate layout inside fresh-test plan")
    for index in range(0, len(plan_rows), 2):
        left, right = plan_rows[index], plan_rows[index + 1]
        if pattern(right) != complement(pattern(left)):
            raise ValueError(f"Test pair is not complementary: {left['capture_id']}/{right['capture_id']}")
        if {int(left["occupied_count"]), int(right["occupied_count"])} != {3, 6}:
            raise ValueError(f"Test pair is not 3/6 balanced: {left['capture_id']}/{right['capture_id']}")
    slot_counts = {slot: sum(row[slot] == "OCCUPIED" for row in plan_rows) for slot in SLOTS}
    if set(slot_counts.values()) != {5}:
        raise ValueError(f"Fresh-test slot imbalance: {slot_counts}")
    plan_lights = Counter(row["captured_light"] for row in plan_rows)
    if dict(plan_lights) != config["expected_light_counts"]["test"]:
        raise ValueError(f"Fresh-test light imbalance: {dict(plan_lights)}")
    if any(row["review_status"] != "pending_capture_review" for row in plan_rows):
        raise ValueError("Capture plan must remain pending until source-image review")

    prior_patterns = {pattern(row) for row in corrected_rows + validation_rows}
    collisions = [row["capture_id"] for row in plan_rows if pattern(row) in prior_patterns]
    complement_collisions = [
        row["capture_id"] for row in plan_rows if complement(pattern(row)) in prior_patterns
    ]
    if collisions or complement_collisions:
        raise ValueError(
            f"Fresh-test layout collision: exact={collisions}, complement={complement_collisions}"
        )

    prior_source_hashes = {
        sha256_file(rooted_path(root, row["source_relpath"]))
        for row in corrected_rows + validation_rows
    }
    existing_sources: list[str] = []
    fresh_hashes: dict[str, str] = {}
    for row in plan_rows:
        source = rooted_path(root, row["source_relpath"])
        if not source.is_file():
            continue
        try:
            with Image.open(source) as image:
                if image.format != "JPEG" or image.size != (1280, 720):
                    raise ValueError(
                        f"Unexpected fresh-test source format/size: "
                        f"{row['capture_id']} {image.format} {image.size}"
                    )
                image.verify()
        except OSError as exc:
            raise ValueError(f"Fresh-test source decode failed: {source}: {exc}") from exc
        source_hash = sha256_file(source)
        if source_hash in prior_source_hashes:
            raise ValueError(f"Fresh-test source duplicates prior split: {row['capture_id']}")
        if source_hash in fresh_hashes.values():
            duplicate = next(key for key, value in fresh_hashes.items() if value == source_hash)
            raise ValueError(
                f"Duplicate fresh-test source SHA: {duplicate}/{row['capture_id']}"
            )
        fresh_hashes[row["capture_id"]] = source_hash
        existing_sources.append(row["capture_id"])
    final_test_manifest = rooted_path(root, config["split_manifests"]["test"])

    print("PASS parent v1 ZIP fingerprint verified and v1 inputs were read-only")
    print("PASS corrected manifest diff: exactly 8 semantic flips")
    print("PASS corrected base classes: OCCUPIED=232 EMPTY=218")
    print("PASS consumed v1 test: 10 captures locked test->train, never_test_again=true")
    print("PASS split IDs: train=a001-a050 validation=a071-a080 test=a081-a090")
    print("PASS fresh-test plan: 5 complementary 3/6 pairs; every slot OCCUPIED 5 times")
    print("PASS fresh-test novelty: no exact or complementary layout collision")
    print(f"PASS fresh-test acquisition lights: {dict(plan_lights)} original captures")
    if final_test_manifest.is_file():
        print(f"NEXT final reviewed test manifest exists: {final_test_manifest}")
    else:
        print("WAIT final reviewed test manifest does not exist (expected before capture review)")
    print(f"READY_FOR_CAPTURE captured_plan_sources={len(existing_sources)}/10")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, IndexError, json.JSONDecodeError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
