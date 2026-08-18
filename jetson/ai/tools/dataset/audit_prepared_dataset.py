#!/usr/bin/env python3
"""Audit prepared dataset counts, provenance, labels, and split isolation."""

from __future__ import annotations

import argparse
import csv
from collections import Counter, defaultdict
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
SPLITS = ("train", "validation", "test")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--prepared-root", required=True, type=Path)
    parser.add_argument("--roi-config", required=True, type=Path)
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


def fail(errors: list[str], message: str) -> None:
    errors.append(message)


def main() -> int:
    args = parse_args()
    frames = read_csv(args.prepared_root / "manifests/frame_manifest.csv")
    samples = read_csv(args.prepared_root / "manifests/roi_manifest.csv")
    roi_config = json.loads(args.roi_config.read_text(encoding="utf-8"))
    bounds = {slot: tuple(map(int, roi_config["slots"][slot])) for slot in SLOTS}
    errors: list[str] = []

    frame_counts = Counter(row["final_split"] for row in frames)
    if frame_counts != Counter({"train": 40, "validation": 10, "test": 10}):
        fail(errors, f"frame counts: {dict(frame_counts)}")
    capture_splits: dict[str, set[str]] = defaultdict(set)
    for row in frames:
        capture_splits[row["capture_id"]].add(row["final_split"])
    leaked = sorted(capture_id for capture_id, splits in capture_splits.items() if len(splits) != 1)
    if leaked:
        fail(errors, f"capture IDs cross splits: {leaked}")

    sample_counts = Counter(row["split"] for row in samples)
    expected_samples = Counter({"train": 1800, "validation": 90, "test": 90})
    if sample_counts != expected_samples:
        fail(errors, f"sample counts: {dict(sample_counts)}")
    class_counts = Counter((row["split"], row["label"]) for row in samples)
    expected_classes = Counter({
        ("train", "OCCUPIED"): 915,
        ("train", "EMPTY"): 885,
        ("validation", "OCCUPIED"): 45,
        ("validation", "EMPTY"): 45,
        ("test", "OCCUPIED"): 47,
        ("test", "EMPTY"): 43,
    })
    if class_counts != expected_classes:
        fail(errors, f"class counts: {dict(class_counts)}")

    ids = [row["sample_id"] for row in samples]
    paths = [row["relative_path"] for row in samples]
    if len(ids) != len(set(ids)):
        fail(errors, "duplicate sample IDs")
    if len(paths) != len(set(paths)):
        fail(errors, "duplicate output paths")

    originals: dict[tuple[str, str], dict[str, str]] = {}
    augmentation_counts: Counter[tuple[str, str]] = Counter()
    frame_lookup = {row["capture_id"]: row for row in frames}
    slot_original_counts: Counter[tuple[str, str]] = Counter()
    for row in samples:
        capture_id, slot, split = row["capture_id"], row["slot"], row["split"]
        frame = frame_lookup.get(capture_id)
        if frame is None:
            fail(errors, f"unknown capture in ROI manifest: {capture_id}")
            continue
        if frame["final_split"] != split:
            fail(errors, f"split mismatch: {row['sample_id']}")
        if frame[slot] != row["label"]:
            fail(errors, f"label mismatch: {row['sample_id']}")
        if row["variant"] == "original":
            originals[(capture_id, slot)] = row
            slot_original_counts[(split, slot)] += 1
            if row["parent_sample_id"] or row["augmentation_json"]:
                fail(errors, f"original has augmentation provenance: {row['sample_id']}")
        else:
            augmentation_counts[(capture_id, slot)] += 1
            if split != "train":
                fail(errors, f"non-train augmentation: {row['sample_id']}")
            if not row["parent_sample_id"] or not row["augmentation_json"]:
                fail(errors, f"missing augmentation provenance: {row['sample_id']}")
            try:
                json.loads(row["augmentation_json"])
            except json.JSONDecodeError:
                fail(errors, f"invalid augmentation JSON: {row['sample_id']}")

        output_path = args.prepared_root / row["relative_path"]
        if not output_path.is_file():
            fail(errors, f"missing output: {output_path}")
            continue
        if sha256_file(output_path) != row["output_sha256"]:
            fail(errors, f"output checksum mismatch: {row['sample_id']}")
        try:
            with Image.open(output_path) as image:
                expected = (bounds[slot][2] - bounds[slot][0], bounds[slot][3] - bounds[slot][1])
                if image.size != expected or image.mode != "RGB":
                    fail(errors, f"image geometry mismatch: {row['sample_id']}")
                image.verify()
        except OSError as exc:
            fail(errors, f"image decode failure {row['sample_id']}: {exc}")

    expected_originals = len(frames) * len(SLOTS)
    if len(originals) != expected_originals:
        fail(errors, f"original ROI count: {len(originals)} != {expected_originals}")
    for key, original in originals.items():
        expected_augments = 4 if original["split"] == "train" else 0
        if augmentation_counts[key] != expected_augments:
            fail(errors, f"augmentation count for {key}: {augmentation_counts[key]}")
        source = args.dataset_root / original["source_frame_relpath"]
        if not source.is_file() or sha256_file(source) != original["source_frame_sha256"]:
            fail(errors, f"source checksum mismatch: {key}")

    for split, expected_count in (("train", 40), ("validation", 10), ("test", 10)):
        for slot in SLOTS:
            if slot_original_counts[(split, slot)] != expected_count:
                fail(errors, f"slot original count {split}/{slot}: {slot_original_counts[(split, slot)]}")

    validation_lights = Counter(
        row["captured_light"] for row in frames if row["final_split"] == "validation"
    )
    if validation_lights != Counter({f"L{number:02d}": 2 for number in range(5)}):
        fail(errors, f"validation lights: {dict(validation_lights)}")

    if errors:
        for error in errors:
            print(f"FAIL {error}", file=sys.stderr)
        return 1
    print("PASS frame splits: train=40 validation=10 test=10")
    print("PASS ROI samples: train=1800 validation=90 test=90")
    print("PASS classes: train=915/885 validation=45/45 test=47/43 (occupied/empty)")
    print("PASS leakage: no capture crosses splits; augmentation is train-only")
    print("PASS provenance: source/output checksums and parent links verified")
    print("PASS images: all files decode as RGB with calibrated ROI dimensions")
    print("PASS validation lighting: L00-L04 each contain 2 frames")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
