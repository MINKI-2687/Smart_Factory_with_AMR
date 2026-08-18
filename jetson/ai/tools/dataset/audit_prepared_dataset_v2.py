#!/usr/bin/env python3
"""Audit a completed rack_roi_dataset_v2 and all provenance invariants."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
import json
from pathlib import Path
import sys

from PIL import Image

import build_dataset as v1_build
import build_dataset_v2 as v2_build


SLOTS = v1_build.SLOTS
LABELS = v1_build.LABELS
SPLITS = v1_build.SPLITS


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--prepared-root", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    return parser.parse_args()


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def safe_child(root: Path, relative: str) -> Path:
    path = (root / relative).resolve()
    if not path.is_relative_to(root):
        raise ValueError(f"Path escapes root: {relative}")
    return path


def main() -> int:
    args = parse_args()
    root = args.dataset_root.resolve()
    prepared = args.prepared_root.resolve()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    v2_build.validate_config(config)
    if prepared.name != config["dataset_id"]:
        raise ValueError(
            f"Prepared directory name mismatch: {prepared.name} != {config['dataset_id']}"
        )
    if not prepared.is_dir():
        raise ValueError(f"Prepared dataset does not exist: {prepared}")

    roi_config = json.loads(
        v2_build.rooted_path(root, config["roi_config"]).read_text(encoding="utf-8")
    )
    expected_size = (int(roi_config["frame_width"]), int(roi_config["frame_height"]))
    v2_build.validate_correction_contract(root, config)
    source_frames, _, manifest_paths = v2_build.load_and_preflight(
        root, config, expected_size
    )
    frames = read_csv(prepared / "manifests/frame_manifest.csv")
    samples = read_csv(prepared / "manifests/roi_manifest.csv")
    errors: list[str] = []

    def fail(message: str) -> None:
        errors.append(message)

    source_frame_lookup = {row["capture_id"]: row for row in source_frames}
    frame_lookup = {row["capture_id"]: row for row in frames}
    if len(frame_lookup) != len(frames):
        fail("duplicate capture IDs in prepared frame manifest")
    if frame_lookup != source_frame_lookup:
        fail("prepared frame manifest differs from current reviewed split inputs")
    frame_counts = Counter(row["final_split"] for row in frames)
    if dict(frame_counts) != config["expected_frame_counts"]:
        fail(f"frame counts: {dict(frame_counts)}")

    capture_splits: dict[str, set[str]] = defaultdict(set)
    for row in frames:
        capture_splits[row["capture_id"]].add(row["final_split"])
    leaked_ids = sorted(key for key, values in capture_splits.items() if len(values) != 1)
    if leaked_ids:
        fail(f"capture IDs cross splits: {leaked_ids}")

    sample_counts = Counter(row["split"] for row in samples)
    if dict(sample_counts) != config["expected_sample_counts"]:
        fail(f"sample counts: {dict(sample_counts)}")
    class_counts = Counter((row["split"], row["label"]) for row in samples)
    observed_classes = {
        split: {label: class_counts[(split, label)] for label in sorted(LABELS)}
        for split in SPLITS
    }
    if observed_classes != config["expected_class_counts"]:
        fail(f"class counts: {observed_classes}")

    ids = [row["sample_id"] for row in samples]
    relative_paths = [row["relative_path"] for row in samples]
    if len(ids) != len(set(ids)):
        fail("duplicate sample IDs")
    if len(relative_paths) != len(set(relative_paths)):
        fail("duplicate output paths")

    bounds = {slot: tuple(map(int, roi_config["slots"][slot])) for slot in SLOTS}
    originals: dict[tuple[str, str], dict[str, str]] = {}
    augmentations: dict[tuple[str, str], list[dict[str, str]]] = defaultdict(list)
    slot_original_counts: Counter[tuple[str, str]] = Counter()
    source_hashes_by_split: dict[str, set[str]] = defaultdict(set)
    actual_source_hashes: dict[Path, str] = {}
    listed_files: set[Path] = set()

    for row in samples:
        capture_id = row["capture_id"]
        slot = row["slot"]
        split = row["split"]
        label = row["label"]
        frame = frame_lookup.get(capture_id)
        if frame is None:
            fail(f"unknown capture in ROI manifest: {capture_id}")
            continue
        if slot not in SLOTS or label not in LABELS:
            fail(f"invalid slot/label: {row['sample_id']}")
            continue
        if frame["final_split"] != split or frame[slot] != label:
            fail(f"frame/sample split or label mismatch: {row['sample_id']}")
        if row["source_frame_relpath"] != frame["source_relpath"]:
            fail(f"frame/sample source path mismatch: {row['sample_id']}")
        expected_relative = (
            Path("images") / split / label / f"{row['sample_id']}.png"
        ).as_posix()
        if row["relative_path"] != expected_relative:
            fail(f"non-canonical output path: {row['sample_id']}")
        try:
            output_path = safe_child(prepared, row["relative_path"])
        except ValueError as exc:
            fail(str(exc))
            continue
        listed_files.add(output_path)
        if not output_path.is_file():
            fail(f"missing output: {row['relative_path']}")
            continue
        if v1_build.sha256_file(output_path) != row["output_sha256"]:
            fail(f"output checksum mismatch: {row['sample_id']}")
        try:
            with Image.open(output_path) as image:
                expected_roi_size = (
                    bounds[slot][2] - bounds[slot][0],
                    bounds[slot][3] - bounds[slot][1],
                )
                if image.size != expected_roi_size or image.mode != "RGB":
                    fail(f"image geometry/mode mismatch: {row['sample_id']}")
                image.verify()
        except OSError as exc:
            fail(f"image decode failure {row['sample_id']}: {exc}")

        source_path = v2_build.rooted_path(root, row["source_frame_relpath"])
        actual_source_hash = actual_source_hashes.setdefault(
            source_path, v1_build.sha256_file(source_path)
        )
        if actual_source_hash != row["source_frame_sha256"]:
            fail(f"source checksum mismatch: {row['sample_id']}")
        source_hashes_by_split[split].add(row["source_frame_sha256"])

        key = (capture_id, slot)
        if row["variant"] == "original":
            if key in originals:
                fail(f"duplicate original ROI: {key}")
            originals[key] = row
            slot_original_counts[(split, slot)] += 1
            if row["parent_sample_id"] or row["augmentation_json"]:
                fail(f"original has augmentation provenance: {row['sample_id']}")
            expected_id = f"{capture_id}_{slot}_original"
            if row["sample_id"] != expected_id:
                fail(f"non-canonical original sample ID: {row['sample_id']}")
        else:
            augmentations[key].append(row)
            if split != "train":
                fail(f"non-train augmentation: {row['sample_id']}")
            if row["parent_sample_id"] != f"{capture_id}_{slot}_original":
                fail(f"augmentation parent ID mismatch: {row['sample_id']}")
            variant_number = v2_build.AUGMENTATION_VARIANTS.get(row["variant"])
            expected_id = (
                f"{capture_id}_{slot}_aug{variant_number:02d}_{row['variant']}"
                if variant_number is not None
                else ""
            )
            if row["sample_id"] != expected_id:
                fail(f"non-canonical augmentation sample ID: {row['sample_id']}")
            try:
                json.loads(row["augmentation_json"])
            except json.JSONDecodeError:
                fail(f"invalid augmentation JSON: {row['sample_id']}")

    expected_originals = len(frames) * len(SLOTS)
    if len(originals) != expected_originals:
        fail(f"original ROI count: {len(originals)} != {expected_originals}")
    for key, original in originals.items():
        derived = augmentations[key]
        expected_count = 4 if original["split"] == "train" else 0
        if len(derived) != expected_count:
            fail(f"augmentation count for {key}: {len(derived)} != {expected_count}")
        if len({row["variant"] for row in derived}) != len(derived):
            fail(f"duplicate augmentation variant for {key}")
        expected_variants = (
            set(v2_build.AUGMENTATION_VARIANTS) if original["split"] == "train" else set()
        )
        if {row["variant"] for row in derived} != expected_variants:
            fail(f"augmentation variant set mismatch for {key}")
        for row in derived:
            for field in (
                "split", "label", "capture_id", "slot", "source_frame_relpath",
                "source_frame_sha256",
            ):
                if row[field] != original[field]:
                    fail(f"augmentation/original {field} mismatch: {row['sample_id']}")

    for split in SPLITS:
        expected_count = int(config["expected_frame_counts"][split])
        for slot in SLOTS:
            if slot_original_counts[(split, slot)] != expected_count:
                fail(
                    f"slot original count {split}/{slot}: "
                    f"{slot_original_counts[(split, slot)]} != {expected_count}"
                )
    for index, left in enumerate(SPLITS):
        for right in SPLITS[index + 1:]:
            shared = source_hashes_by_split[left] & source_hashes_by_split[right]
            if shared:
                fail(f"source SHA leakage {left}/{right}: {sorted(shared)}")

    actual_files = {path.resolve() for path in (prepared / "images").rglob("*.png")}
    if actual_files != listed_files:
        missing = sorted(str(path.relative_to(prepared)) for path in listed_files - actual_files)
        extra = sorted(str(path.relative_to(prepared)) for path in actual_files - listed_files)
        fail(f"PNG file set mismatch: missing={missing}, extra={extra}")

    corrections = read_csv(v2_build.rooted_path(root, config["label_correction_ledger"]))
    corrected_sample_count = 0
    for correction in corrections:
        key = (correction["capture_id"], correction["slot"])
        rows = ([originals[key]] if key in originals else []) + augmentations[key]
        if len(rows) != 5:
            fail(f"corrected semantic slot did not regenerate 5 samples: {key}")
            continue
        if any(row["label"] != correction["new_label"] for row in rows):
            fail(f"corrected semantic slot uses old class: {key}")
        corrected_sample_count += len(rows)
    if corrected_sample_count != 40:
        fail(f"corrected derived sample count: {corrected_sample_count} != 40")

    current_validation = prepared / "images/validation"
    parent_validation_rows = read_csv(
        prepared / "manifests/parent_v1_validation_sha256.csv"
    )
    parent_validation_paths = [row["relative_path"] for row in parent_validation_rows]
    if (
        len(parent_validation_rows) != 90
        or len(parent_validation_paths) != len(set(parent_validation_paths))
    ):
        fail("parent v1 validation reference is not exactly 90 unique rows")
    legacy_files = {
        Path(row["relative_path"]): row["sha256"] for row in parent_validation_rows
    }
    current_files = {
        path.relative_to(current_validation): v1_build.sha256_file(path)
        for path in current_validation.rglob("*.png")
    }
    if legacy_files != current_files:
        fail("v2 validation PNG set/hashes are not byte-identical to v1")

    fingerprint_rows = read_csv(prepared / "manifests/input_sha256.csv")
    if not fingerprint_rows:
        fail("empty input fingerprint manifest")
    fingerprint_relpaths = [row["relative_path"] for row in fingerprint_rows]
    if len(fingerprint_relpaths) != len(set(fingerprint_relpaths)):
        fail("duplicate input fingerprint path")
    expected_fingerprint_paths = v2_build.input_fingerprint_paths(
        root, args.config.resolve(), config, source_frames, manifest_paths
    )
    expected_fingerprint_relpaths = {
        v2_build.fingerprint_relative_path(root, path)
        for path in expected_fingerprint_paths
    }
    if set(fingerprint_relpaths) != expected_fingerprint_relpaths:
        fail("input fingerprint path set differs from exact expected inputs")
    for row in fingerprint_rows:
        path = v2_build.rooted_path(root, row["relative_path"])
        if not path.is_file() or v1_build.sha256_file(path) != row["sha256"]:
            fail(f"input fingerprint mismatch: {row['relative_path']}")

    summary = json.loads((prepared / "dataset_summary.json").read_text(encoding="utf-8"))
    if summary.get("dataset_id") != config["dataset_id"]:
        fail("dataset summary ID mismatch")
    if summary.get("total_samples") != 2430:
        fail(f"dataset summary total: {summary.get('total_samples')}")
    if summary.get("retired_test_report_sha256") != config["retired_test_report_sha256"]:
        fail("dataset summary retired-test report hash mismatch")
    if summary.get("parent_dataset_zip_sha256") != config["parent_dataset_zip_sha256"]:
        fail("dataset summary parent ZIP hash mismatch")
    reference_path = prepared / summary.get("parent_validation_reference", "")
    if (
        not reference_path.is_file()
        or v1_build.sha256_file(reference_path)
        != summary.get("parent_validation_reference_sha256")
    ):
        fail("parent validation reference fingerprint mismatch")

    if errors:
        for error in errors:
            print(f"FAIL {error}", file=sys.stderr)
        return 1
    print("PASS frame splits: train=50 validation=10 test=10")
    print("PASS ROI samples: train=2250 validation=90 test=90 total=2430")
    print("PASS classes: train=1160/1090 validation=45/45 test=45/45 (occupied/empty)")
    print("PASS corrections: 8 semantic flips regenerated from raw into exactly 40 train files")
    print("PASS leakage: capture IDs/source paths/source SHA are split-disjoint")
    print("PASS augmentation: train-only; validation/test are original-only")
    print("PASS provenance: parent links, paths, source/output/input SHA-256 verified")
    print("PASS files: manifest and PNG tree are exact; all RGB ROI geometry verified")
    print("PASS validation preservation: v2 validation PNGs are byte-identical to v1")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
