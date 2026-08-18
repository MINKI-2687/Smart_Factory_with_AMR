#!/usr/bin/env python3
"""Build rack_roi_dataset_v2 atomically from three reviewed split manifests.

This tool is separate from the released v1 builder. It refuses to create any
output until the fresh a081-a090 test manifest and every source frame pass the
full preflight. Only train receives deterministic photometric augmentation.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
import json
from pathlib import Path
import shutil
import sys
import tempfile
from typing import Any

import numpy as np
from PIL import Image

import build_dataset as v1_build


SLOTS = v1_build.SLOTS
LABELS = v1_build.LABELS
SPLITS = v1_build.SPLITS
AUGMENTATION_VARIANTS = {
    "dark_brightness": 1,
    "bright_contrast": 2,
    "gamma_contrast_jpeg": 3,
    "side_shadow_gamma_jpeg": 4,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    return parser.parse_args()


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def rooted_path(root: Path, relative: str) -> Path:
    path = (root / relative).resolve()
    if not path.is_relative_to(root):
        raise ValueError(f"Path escapes dataset root: {relative}")
    return path


def validate_config(config: dict[str, Any]) -> None:
    if int(config.get("schema_version", 0)) != 2:
        raise ValueError("build_dataset_v2 requires schema_version=2")
    if tuple(config.get("split_manifests", {}).keys()) != SPLITS:
        raise ValueError("split_manifests must be ordered train/validation/test")
    if int(config["train_augmentations_per_roi"]) != 4:
        raise ValueError("v2 requires exactly four train augmentation variants")
    policy = config["augmentation_policy"]
    if policy.get("spatial_transforms") is not False:
        raise ValueError("Spatial transforms are forbidden")
    if policy.get("validation_augmentation") is not False:
        raise ValueError("Validation augmentation must be disabled")
    if policy.get("test_augmentation") is not False:
        raise ValueError("Test augmentation must be disabled")
    test_policy = config["test_policy"]
    for key in (
        "post_capture_photometric_processing",
        "augmentation",
    ):
        if test_policy.get(key) is not False:
            raise ValueError(f"Unsafe test policy: {key} must be false")
    for key in ("freeze_before_inference", "single_evaluation_lock"):
        if test_policy.get(key) is not True:
            raise ValueError(f"Unsafe test policy: {key} must be true")


def validate_test_matches_plan(
    root: Path,
    config: dict[str, Any],
    test_rows: list[dict[str, str]],
) -> None:
    plan_rows = read_csv(rooted_path(root, config["test_capture_plan"]))
    plan = {row["capture_id"]: row for row in plan_rows}
    actual = {row["capture_id"]: row for row in test_rows}
    if set(plan) != set(actual):
        raise ValueError("Reviewed test manifest IDs differ from the frozen capture plan")
    fixed_fields = (
        "source_relpath", "captured_session", "captured_light", "occupied_count", *SLOTS,
    )
    for capture_id in sorted(plan):
        for field in fixed_fields:
            if plan[capture_id].get(field, "") != actual[capture_id].get(field, ""):
                raise ValueError(f"Reviewed test differs from plan: {capture_id}/{field}")
        if actual[capture_id].get("split", "test") != "test":
            raise ValueError(f"Reviewed test split mismatch: {capture_id}")


def validate_fresh_test_capture_provenance(
    root: Path,
    config: dict[str, Any],
    test_rows: list[dict[str, str]],
) -> None:
    provenance = config.get("capture_provenance", [])
    if len(provenance) != 3:
        raise ValueError("capture_provenance must contain log, assignment events, revisions")
    log_rows = read_csv(rooted_path(root, provenance[0]))
    events = read_csv(rooted_path(root, provenance[1]))
    revisions = read_csv(rooted_path(root, provenance[2]))
    test_by_id = {row["capture_id"]: row for row in test_rows}
    log_by_id = {row["capture_id"]: row for row in log_rows}
    if len(log_by_id) != len(log_rows) or set(log_by_id) != set(test_by_id):
        raise ValueError("Fresh-test capture log IDs do not exactly match reviewed manifest")
    for capture_id, reviewed in test_by_id.items():
        log = log_by_id[capture_id]
        source = rooted_path(root, reviewed["source_relpath"])
        source_hash = v1_build.sha256_file(source)
        for left, right, field in (
            (log["source_relpath"], reviewed["source_relpath"], "source_relpath"),
            (log["source_sha256"], reviewed["source_sha256"], "source_sha256"),
            (log["captured_light"], reviewed["captured_light"], "captured_light"),
            (log["shift_x"], reviewed["alignment_shift_x"], "shift_x"),
            (log["shift_y"], reviewed["alignment_shift_y"], "shift_y"),
            (log["captured_at_utc"], reviewed["visual_reviewed_at_utc"], "timestamp"),
        ):
            if left != right:
                raise ValueError(f"Fresh-test capture log mismatch: {capture_id}/{field}")
        if (
            source_hash != reviewed["source_sha256"]
            or int(log["bytes"]) != source.stat().st_size
            or (int(log["width"]), int(log["height"])) != (1280, 720)
            or log["post_capture_photometric_processing"].lower() != "false"
            or not log["visual_review_status"].startswith("verified_against_frozen_plan")
        ):
            raise ValueError(f"Invalid fresh-test capture provenance: {capture_id}")

    event_ids = [row["event_id"] for row in events]
    if len(event_ids) != len(set(event_ids)):
        raise ValueError("Duplicate capture assignment event ID")
    for event in events:
        assigned = event["assigned_capture_id"]
        if (
            assigned not in log_by_id
            or event["source_sha256"] != log_by_id[assigned]["source_sha256"]
            or event["captured_at_utc"] != log_by_id[assigned]["captured_at_utc"]
            or event["model_inference_performed_before_assignment"].lower() != "false"
            or event["review_status"] != "verified_before_manifest_finalization"
        ):
            raise ValueError(f"Invalid capture assignment event: {event.get('event_id', '')}")

    if not revisions:
        raise ValueError("Missing fresh-test plan revision provenance")
    final_revision = revisions[-1]
    plan_sha = v1_build.sha256_file(rooted_path(root, config["test_capture_plan"]))
    if (
        final_revision["new_plan_sha256"] != plan_sha
        or final_revision["model_inference_performed_before_revision"].lower() != "false"
    ):
        raise ValueError("Final fresh-test plan revision does not match frozen plan")


def validate_correction_contract(
    root: Path,
    config: dict[str, Any],
    *,
    verify_evidence: bool = True,
) -> tuple[list[dict[str, str]], list[Path]]:
    """Require the corrected train manifest to differ by exactly the ledgered flips."""
    old_rows = read_csv(rooted_path(root, config["source_manifest_before_correction"]))
    corrected_rows = read_csv(rooted_path(root, config["split_manifests"]["train"]))
    correction_rows = read_csv(rooted_path(root, config["label_correction_ledger"]))

    old_ids = [row["capture_id"] for row in old_rows]
    corrected_ids = [row["capture_id"] for row in corrected_rows]
    if len(old_ids) != len(set(old_ids)) or len(corrected_ids) != len(set(corrected_ids)):
        raise ValueError("Duplicate capture ID in old/corrected train manifest")
    old_by_id = {row["capture_id"]: row for row in old_rows}
    corrected_by_id = {row["capture_id"]: row for row in corrected_rows}
    if set(old_by_id) != set(corrected_by_id):
        raise ValueError("Corrected manifest changed the base capture ID set")

    correction_ids = [row["correction_id"] for row in correction_rows]
    correction_keys = [(row["capture_id"], row["slot"]) for row in correction_rows]
    if (
        len(correction_rows) != 8
        or len(correction_ids) != len(set(correction_ids))
        or len(correction_keys) != len(set(correction_keys))
    ):
        raise ValueError("Correction ledger must contain exactly 8 unique corrections")

    correction_map: dict[tuple[str, str], tuple[str, str]] = {}
    evidence_paths: list[Path] = []
    for correction in correction_rows:
        key = (correction["capture_id"], correction["slot"])
        old_label = correction["old_label"]
        new_label = correction["new_label"]
        if (
            key[0] not in old_by_id
            or key[1] not in SLOTS
            or old_label not in LABELS
            or new_label not in LABELS
            or old_label == new_label
        ):
            raise ValueError(f"Invalid correction ledger row: {key}")
        correction_map[key] = (old_label, new_label)
        if correction["identified_from_test_report_sha256"] != config["retired_test_report_sha256"]:
            raise ValueError(f"Correction report hash mismatch: {correction['correction_id']}")
        source_path = rooted_path(root, correction["source_relpath"])
        evidence_path = rooted_path(root, correction["evidence_relpath"])
        evidence_paths.append(evidence_path)
        if verify_evidence:
            for item, field in (
                (source_path, "source_sha256"),
                (evidence_path, "evidence_sha256"),
            ):
                if not item.is_file() or v1_build.sha256_file(item) != correction[field]:
                    raise ValueError(
                        f"Correction evidence mismatch: {correction['correction_id']}/{field}"
                    )

    observed_changes: dict[tuple[str, str], tuple[str, str]] = {}
    allowed_metadata = {"occupied_count", "label_source", "review_status", "proposed_split"}
    retired_ids = set(config["retired_test_capture_ids"])
    for capture_id, old in old_by_id.items():
        new = corrected_by_id[capture_id]
        if set(old) != set(new):
            raise ValueError(f"Manifest column set changed: {capture_id}")
        for field in old:
            if old[field] == new[field]:
                continue
            if field in SLOTS:
                observed_changes[(capture_id, field)] = (old[field], new[field])
            elif field not in allowed_metadata:
                raise ValueError(f"Unexpected corrected-manifest change: {capture_id}/{field}")

        expected_count = sum(new[slot] == "OCCUPIED" for slot in SLOTS)
        if int(new["occupied_count"]) != expected_count:
            raise ValueError(f"Corrected occupied_count mismatch: {capture_id}")
        expected_split = "train" if capture_id in retired_ids else old["proposed_split"]
        if new["proposed_split"] != expected_split:
            raise ValueError(f"Unexpected proposed_split after correction: {capture_id}")
        corrected_here = any(key[0] == capture_id for key in correction_map)
        if corrected_here:
            if (
                new["label_source"] != "manual_visual_reaudit_20260801"
                or new["review_status"] != "corrected_after_full_visual_reaudit"
            ):
                raise ValueError(f"Corrected review metadata mismatch: {capture_id}")
        elif (
            old["label_source"] != new["label_source"]
            or old["review_status"] != new["review_status"]
        ):
            raise ValueError(f"Uncorrected review metadata changed: {capture_id}")

    if observed_changes != correction_map:
        raise ValueError(
            f"Manifest/ledger label diff mismatch: observed={observed_changes}, "
            f"ledger={correction_map}"
        )
    for correction in correction_rows:
        capture_id = correction["capture_id"]
        old = old_by_id[capture_id]
        new = corrected_by_id[capture_id]
        if (
            int(correction["old_occupied_count"]) != int(old["occupied_count"])
            or int(correction["new_occupied_count"]) != int(new["occupied_count"])
        ):
            raise ValueError(f"Correction count evidence mismatch: {correction['correction_id']}")
    return correction_rows, sorted(set(evidence_paths))


def validate_consumed_test_contract(root: Path, config: dict[str, Any]) -> None:
    rows = read_csv(rooted_path(root, config["split_consumption_ledger"]))
    ids = [row["capture_id"] for row in rows]
    expected_ids = set(config["retired_test_capture_ids"])
    if len(ids) != len(set(ids)) or set(ids) != expected_ids:
        raise ValueError("Split-consumption ledger does not exactly match retired test IDs")
    for row in rows:
        capture_id = row["capture_id"]
        if (
            row["from_dataset_id"] != config["parent_dataset_id"]
            or row["to_dataset_id"] != config["dataset_id"]
            or row["old_split"] != "test"
            or row["new_split"] != "train"
            or row["never_test_again"].lower() != "true"
            or row["review_status"] != "locked_consumed"
            or row["prior_test_report_sha256"] != config["retired_test_report_sha256"]
        ):
            raise ValueError(f"Invalid consumed-test lock: {capture_id}")
        source = rooted_path(root, row["source_relpath"])
        if not source.is_file() or v1_build.sha256_file(source) != row["source_sha256"]:
            raise ValueError(f"Consumed-test source hash mismatch: {capture_id}")


def load_and_preflight(
    root: Path,
    config: dict[str, Any],
    expected_size: tuple[int, int],
) -> tuple[list[dict[str, str]], dict[Path, str], list[Path]]:
    frames: list[dict[str, str]] = []
    source_hashes: dict[Path, str] = {}
    manifest_paths: list[Path] = []
    split_sha: dict[str, set[str]] = defaultdict(set)
    split_relpaths: dict[str, set[str]] = defaultdict(set)
    rows_by_split: dict[str, list[dict[str, str]]] = {}

    for split in SPLITS:
        manifest_path = rooted_path(root, config["split_manifests"][split])
        if not manifest_path.is_file():
            if split == "test":
                raise ValueError(
                    "Missing final reviewed test manifest: "
                    f"{manifest_path}. Capture and review a081-a090 first."
                )
            raise ValueError(f"Missing {split} manifest: {manifest_path}")
        manifest_paths.append(manifest_path)
        rows = read_csv(manifest_path)
        rows_by_split[split] = rows
        ids = [row["capture_id"] for row in rows]
        expected_ids = set(config["expected_capture_ids"][split])
        if len(ids) != len(set(ids)) or set(ids) != expected_ids:
            raise ValueError(
                f"{split} capture IDs mismatch: "
                f"missing={sorted(expected_ids - set(ids))}, extra={sorted(set(ids) - expected_ids)}"
            )
        if len(rows) != int(config["expected_frame_counts"][split]):
            raise ValueError(f"Unexpected {split} frame count: {len(rows)}")
        allowed_statuses = set(config["allowed_review_statuses"][split])
        for row in rows:
            capture_id = row["capture_id"]
            if row.get("split") and row["split"] != split:
                raise ValueError(f"Manifest split mismatch: {capture_id}")
            if row.get("safety_ignore", "").lower() != "false":
                raise ValueError(f"Safety-ignore frame cannot enter v2: {capture_id}")
            if row.get("review_status", "") not in allowed_statuses:
                raise ValueError(
                    f"Unreviewed frame cannot enter v2: {capture_id}/{row.get('review_status', '')}"
                )
            states = [row.get(slot, "") for slot in SLOTS]
            if set(states) - LABELS:
                raise ValueError(f"Invalid states for {capture_id}: {states}")
            occupied = states.count("OCCUPIED")
            if int(row["occupied_count"]) != occupied:
                raise ValueError(f"occupied_count mismatch for {capture_id}")

            source_path = rooted_path(root, row["source_relpath"])
            if not source_path.is_file():
                raise ValueError(f"Missing source frame: {source_path}")
            try:
                with Image.open(source_path) as image:
                    if image.size != expected_size:
                        raise ValueError(
                            f"{source_path}: expected {expected_size}, got {image.size}"
                        )
                    image.verify()
            except OSError as exc:
                raise ValueError(f"Source frame decode failed: {source_path}: {exc}") from exc
            source_hash = source_hashes.setdefault(source_path, v1_build.sha256_file(source_path))
            if row.get("source_sha256") and row["source_sha256"] != source_hash:
                raise ValueError(f"Frozen source SHA-256 mismatch: {capture_id}")
            split_sha[split].add(source_hash)
            split_relpaths[split].add(row["source_relpath"])

            normalized = {
                "capture_id": capture_id,
                "source_relpath": row["source_relpath"],
                "captured_session": row["captured_session"],
                "captured_light": row["captured_light"],
                "final_split": split,
                "safety_ignore": "false",
                "occupied_count": str(occupied),
                "label_source": row.get("label_source", ""),
                "review_status": row.get("review_status", ""),
            }
            normalized.update({slot: row[slot] for slot in SLOTS})
            frames.append(normalized)

        if split in config.get("expected_light_counts", {}):
            observed_lights = Counter(row["captured_light"] for row in rows)
            if dict(observed_lights) != config["expected_light_counts"][split]:
                raise ValueError(f"{split} light counts mismatch: {dict(observed_lights)}")

    validate_test_matches_plan(root, config, rows_by_split["test"])
    validate_fresh_test_capture_provenance(root, config, rows_by_split["test"])
    validate_consumed_test_contract(root, config)
    for index, left in enumerate(SPLITS):
        for right in SPLITS[index + 1:]:
            shared_ids = {
                row["capture_id"] for row in frames if row["final_split"] == left
            } & {
                row["capture_id"] for row in frames if row["final_split"] == right
            }
            shared_paths = split_relpaths[left] & split_relpaths[right]
            shared_hashes = split_sha[left] & split_sha[right]
            if shared_ids or shared_paths or shared_hashes:
                raise ValueError(
                    f"Split leakage {left}/{right}: ids={sorted(shared_ids)}, "
                    f"paths={sorted(shared_paths)}, hashes={sorted(shared_hashes)}"
                )

    retired = set(config["retired_test_capture_ids"])
    train_ids = {row["capture_id"] for row in frames if row["final_split"] == "train"}
    nontrain_ids = {row["capture_id"] for row in frames if row["final_split"] != "train"}
    if not retired <= train_ids or retired & nontrain_ids:
        raise ValueError("Retired v1 test IDs are not quarantined to v2 train")

    original_class_counts = Counter(
        (row["final_split"], row[slot]) for row in frames for slot in SLOTS
    )
    for split in SPLITS:
        multiplier = 5 if split == "train" else 1
        observed = {
            label: original_class_counts[(split, label)] * multiplier
            for label in sorted(LABELS)
        }
        if observed != config["expected_class_counts"][split]:
            raise ValueError(f"{split} class counts mismatch: {observed}")
    return sorted(frames, key=lambda row: row["capture_id"]), source_hashes, manifest_paths


def input_fingerprint_paths(
    root: Path,
    config_path: Path,
    config: dict[str, Any],
    frames: list[dict[str, str]],
    manifest_paths: list[Path],
) -> set[Path]:
    correction_rows, evidence_paths = validate_correction_contract(root, config)
    del correction_rows
    paths = {
        config_path.resolve(),
        rooted_path(root, config["roi_config"]),
        rooted_path(root, config["source_manifest_before_correction"]),
        rooted_path(root, config["label_correction_ledger"]),
        rooted_path(root, config["split_consumption_ledger"]),
        rooted_path(root, config["test_capture_plan"]),
        rooted_path(root, "tools/dataset/build_dataset.py"),
        rooted_path(root, "tools/dataset/build_dataset_v2.py"),
        rooted_path(root, "tools/dataset/audit_prepared_dataset_v2.py"),
        *manifest_paths,
        *evidence_paths,
        *(rooted_path(root, row["source_relpath"]) for row in frames),
        *(rooted_path(root, relative) for relative in config.get("capture_provenance", [])),
    }
    missing = sorted(str(path) for path in paths if not path.is_file())
    if missing:
        raise ValueError(f"Missing fingerprint inputs: {missing}")
    return paths


def fingerprint_relative_path(root: Path, path: Path) -> str:
    try:
        return path.relative_to(root).as_posix()
    except ValueError:
        return str(path)


def write_input_fingerprints(
    path: Path,
    root: Path,
    args: argparse.Namespace,
    config: dict[str, Any],
    frames: list[dict[str, str]],
    manifest_paths: list[Path],
) -> None:
    paths = input_fingerprint_paths(
        root, args.config.resolve(), config, frames, manifest_paths
    )
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=("relative_path", "sha256"))
        writer.writeheader()
        for item in sorted(paths):
            writer.writerow({
                "relative_path": fingerprint_relative_path(root, item),
                "sha256": v1_build.sha256_file(item),
            })


def write_parent_validation_reference(path: Path, root: Path) -> None:
    legacy_root = root / "prepared/rack_roi_dataset_v1/images/validation"
    rows = [
        {
            "relative_path": source.relative_to(legacy_root).as_posix(),
            "sha256": v1_build.sha256_file(source),
        }
        for source in sorted(legacy_root.rglob("*.png"))
    ]
    if len(rows) != 90 or len({row["relative_path"] for row in rows}) != 90:
        raise ValueError("Parent v1 validation reference must contain exactly 90 unique PNGs")
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=("relative_path", "sha256"))
        writer.writeheader()
        writer.writerows(rows)


def build_into(
    staging: Path,
    root: Path,
    args: argparse.Namespace,
    config: dict[str, Any],
    roi_config: dict[str, Any],
    frames: list[dict[str, str]],
    source_hashes: dict[Path, str],
    manifest_paths: list[Path],
) -> None:
    for split in SPLITS:
        for label in sorted(LABELS):
            (staging / "images" / split / label).mkdir(parents=True, exist_ok=True)
    manifest_dir = staging / "manifests"
    manifest_dir.mkdir(parents=True, exist_ok=True)
    v1_build.write_frame_manifest(manifest_dir / "frame_manifest.csv", frames)

    slots = {name: tuple(map(int, roi_config["slots"][name])) for name in SLOTS}
    expected_size = (int(roi_config["frame_width"]), int(roi_config["frame_height"]))
    augment_count = int(config["train_augmentations_per_roi"])
    policy = config["augmentation_policy"]
    seed = int(config["augmentation_seed"])
    roi_fields = (
        "sample_id", "relative_path", "split", "label", "capture_id", "slot",
        "variant", "parent_sample_id", "source_frame_relpath", "source_frame_sha256",
        "output_sha256", "augmentation_json",
    )
    roi_rows: list[dict[str, str]] = []

    for frame_row in frames:
        source_path = rooted_path(root, frame_row["source_relpath"])
        with Image.open(source_path) as decoded:
            image = decoded.convert("RGB")
        if image.size != expected_size:
            raise ValueError(f"Frame geometry changed after preflight: {source_path}")
        if v1_build.sha256_file(source_path) != source_hashes[source_path]:
            raise ValueError(f"Source frame changed after preflight: {source_path}")
        frame = np.asarray(image, dtype=np.uint8)
        frame_hash = source_hashes[source_path]
        split = frame_row["final_split"]
        for slot in SLOTS:
            x0, y0, x1, y1 = slots[slot]
            roi = np.ascontiguousarray(frame[y0:y1, x0:x1, :])
            label = frame_row[slot]
            original_id = f"{frame_row['capture_id']}_{slot}_original"
            relative = Path("images") / split / label / f"{original_id}.png"
            output_path = staging / relative
            Image.fromarray(roi, mode="RGB").save(output_path, format="PNG")
            roi_rows.append({
                "sample_id": original_id,
                "relative_path": relative.as_posix(),
                "split": split,
                "label": label,
                "capture_id": frame_row["capture_id"],
                "slot": slot,
                "variant": "original",
                "parent_sample_id": "",
                "source_frame_relpath": frame_row["source_relpath"],
                "source_frame_sha256": frame_hash,
                "output_sha256": v1_build.sha256_file(output_path),
                "augmentation_json": "",
            })
            if split != "train":
                continue
            for variant_number in range(1, augment_count + 1):
                rng = v1_build.stable_rng(
                    seed, frame_row["capture_id"], slot, variant_number
                )
                derived, variant_name, parameters = v1_build.augment(
                    roi, variant_number, rng, policy
                )
                sample_id = (
                    f"{frame_row['capture_id']}_{slot}_aug{variant_number:02d}_{variant_name}"
                )
                relative = Path("images") / split / label / f"{sample_id}.png"
                output_path = staging / relative
                Image.fromarray(derived, mode="RGB").save(output_path, format="PNG")
                roi_rows.append({
                    "sample_id": sample_id,
                    "relative_path": relative.as_posix(),
                    "split": split,
                    "label": label,
                    "capture_id": frame_row["capture_id"],
                    "slot": slot,
                    "variant": variant_name,
                    "parent_sample_id": original_id,
                    "source_frame_relpath": frame_row["source_relpath"],
                    "source_frame_sha256": frame_hash,
                    "output_sha256": v1_build.sha256_file(output_path),
                    "augmentation_json": json.dumps(parameters, sort_keys=True),
                })

    with (manifest_dir / "roi_manifest.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=roi_fields)
        writer.writeheader()
        writer.writerows(roi_rows)

    frame_counts = Counter(row["final_split"] for row in frames)
    sample_counts = Counter(row["split"] for row in roi_rows)
    class_counts = Counter((row["split"], row["label"]) for row in roi_rows)
    if dict(frame_counts) != config["expected_frame_counts"]:
        raise ValueError(f"Built frame counts mismatch: {dict(frame_counts)}")
    if dict(sample_counts) != config["expected_sample_counts"]:
        raise ValueError(f"Built sample counts mismatch: {dict(sample_counts)}")
    observed_classes = {
        split: {label: class_counts[(split, label)] for label in sorted(LABELS)}
        for split in SPLITS
    }
    if observed_classes != config["expected_class_counts"]:
        raise ValueError(f"Built class counts mismatch: {observed_classes}")

    write_input_fingerprints(
        manifest_dir / "input_sha256.csv", root, args, config, frames, manifest_paths
    )
    parent_validation_reference = manifest_dir / "parent_v1_validation_sha256.csv"
    write_parent_validation_reference(parent_validation_reference, root)
    summary = {
        "schema_version": 2,
        "dataset_id": config["dataset_id"],
        "parent_dataset_id": config["parent_dataset_id"],
        "parent_dataset_zip_sha256": config["parent_dataset_zip_sha256"],
        "roi_calibration_id": roi_config["calibration_id"],
        "frame_counts": dict(frame_counts),
        "sample_counts": dict(sample_counts),
        "class_counts": observed_classes,
        "total_samples": len(roi_rows),
        "augmentation_seed": seed,
        "retired_test_capture_ids": config["retired_test_capture_ids"],
        "fresh_test_capture_ids": config["expected_capture_ids"]["test"],
        "retired_test_run_id": config["retired_test_run_id"],
        "retired_test_report_sha256": config["retired_test_report_sha256"],
        "input_fingerprint_manifest": "manifests/input_sha256.csv",
        "parent_validation_reference": "manifests/parent_v1_validation_sha256.csv",
        "parent_validation_reference_sha256": v1_build.sha256_file(
            parent_validation_reference
        ),
    }
    (staging / "dataset_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (staging / "README.md").write_text(
        "# rack_roi_dataset_v2\n\n"
        "- train: corrected a001-a050; original plus four deterministic photometric variants\n"
        "- validation: a071-a080; original crops only\n"
        "- test: fresh a081-a090; original crops only; no post-capture photometric processing\n"
        "- retired v1 test IDs are train-only and must never be used as test again\n",
        encoding="utf-8",
    )


def main() -> int:
    args = parse_args()
    root = args.dataset_root.resolve()
    output_root = args.output_root.resolve()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    validate_config(config)
    parent_zip = rooted_path(root, config["parent_dataset_zip"])
    if v1_build.sha256_file(parent_zip) != config["parent_dataset_zip_sha256"]:
        raise ValueError("Parent v1 ZIP SHA-256 mismatch")
    validate_correction_contract(root, config)
    roi_config = json.loads(rooted_path(root, config["roi_config"]).read_text(encoding="utf-8"))
    if roi_config.get("do_not_use") is True:
        raise ValueError("Selected ROI calibration is marked do_not_use")
    expected_size = (int(roi_config["frame_width"]), int(roi_config["frame_height"]))
    frames, source_hashes, manifest_paths = load_and_preflight(
        root, config, expected_size
    )

    if output_root.exists():
        raise ValueError(f"Refusing to overwrite existing output path: {output_root}")
    output_root.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{output_root.name}.tmp-", dir=output_root.parent))
    try:
        build_into(
            staging,
            root,
            args,
            config,
            roi_config,
            frames,
            source_hashes,
            manifest_paths,
        )
        staging.rename(output_root)
    except Exception:
        if staging.exists() and staging.parent == output_root.parent and staging.name.startswith(
            f".{output_root.name}.tmp-"
        ):
            shutil.rmtree(staging)
        raise

    print(f"PASS frames: {config['expected_frame_counts']}")
    print(f"PASS ROI samples: {config['expected_sample_counts']}, total=2430")
    print(f"PASS classes: {config['expected_class_counts']}")
    print("PASS split leakage preflight: capture IDs, source paths, and source SHA-256 are disjoint")
    print("PASS validation/test augmentation: disabled; original crops only")
    print(f"PASS atomic output: {output_root}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
