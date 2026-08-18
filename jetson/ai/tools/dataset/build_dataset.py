#!/usr/bin/env python3
"""Build a leakage-safe fixed-ROI dataset without OpenCV.

Full frames are decoded with Pillow. Every ROI operation and photometric
augmentation is performed on NumPy buffers. Validation and test samples are
always original crops; only train samples receive derived variants.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
from io import BytesIO
import json
from pathlib import Path
import random
import sys
from typing import Any

import numpy as np
from PIL import Image


SLOTS = (
    "C1_L3", "C2_L3", "C3_L3",
    "C1_L2", "C2_L2", "C3_L2",
    "C1_L1", "C2_L1", "C3_L1",
)
LABELS = {"EMPTY", "OCCUPIED"}
SPLITS = ("train", "validation", "test")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
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


def stable_rng(seed: int, capture_id: str, slot: str, variant: int) -> random.Random:
    payload = f"{seed}:{capture_id}:{slot}:{variant}".encode("utf-8")
    value = int.from_bytes(hashlib.sha256(payload).digest()[:8], "big")
    return random.Random(value)


def clip_uint8(array: np.ndarray) -> np.ndarray:
    return np.clip(np.rint(array), 0, 255).astype(np.uint8)


def brightness(array: np.ndarray, factor: float) -> np.ndarray:
    return clip_uint8(array.astype(np.float32) * factor)


def contrast(array: np.ndarray, factor: float) -> np.ndarray:
    work = array.astype(np.float32)
    mean = work.mean(axis=(0, 1), keepdims=True)
    return clip_uint8((work - mean) * factor + mean)


def gamma_adjust(array: np.ndarray, gamma: float) -> np.ndarray:
    work = np.clip(array.astype(np.float32) / 255.0, 0.0, 1.0)
    return clip_uint8(np.power(work, gamma) * 255.0)


def side_shadow(array: np.ndarray, minimum: float, dark_side: str) -> np.ndarray:
    width = array.shape[1]
    ramp = np.linspace(minimum, 1.0, width, dtype=np.float32)
    if dark_side == "right":
        ramp = ramp[::-1]
    return clip_uint8(array.astype(np.float32) * ramp[None, :, None])


def jpeg_roundtrip(array: np.ndarray, quality: int) -> np.ndarray:
    buffer = BytesIO()
    Image.fromarray(array, mode="RGB").save(
        buffer, format="JPEG", quality=quality, subsampling=2
    )
    buffer.seek(0)
    with Image.open(buffer) as decoded:
        return np.asarray(decoded.convert("RGB"), dtype=np.uint8)


def sample_range(rng: random.Random, values: list[float]) -> float:
    return rng.uniform(float(values[0]), float(values[1]))


def augment(
    roi: np.ndarray,
    variant: int,
    rng: random.Random,
    policy: dict[str, Any],
) -> tuple[np.ndarray, str, dict[str, Any]]:
    if variant == 1:
        factor = sample_range(rng, policy["dark_brightness_range"])
        return brightness(roi, factor), "dark_brightness", {"brightness": round(factor, 4)}

    if variant == 2:
        light = sample_range(rng, policy["bright_brightness_range"])
        level = sample_range(rng, policy["contrast_range"])
        result = contrast(brightness(roi, light), level)
        return result, "bright_contrast", {
            "brightness": round(light, 4),
            "contrast": round(level, 4),
        }

    if variant == 3:
        gamma = sample_range(rng, policy["gamma_range"])
        level = sample_range(rng, policy["contrast_range"])
        quality = rng.randint(*map(int, policy["jpeg_quality_range"]))
        result = jpeg_roundtrip(contrast(gamma_adjust(roi, gamma), level), quality)
        return result, "gamma_contrast_jpeg", {
            "gamma": round(gamma, 4),
            "contrast": round(level, 4),
            "jpeg_quality": quality,
        }

    if variant == 4:
        minimum = sample_range(rng, policy["side_shadow_min_range"])
        dark_side = rng.choice(("left", "right"))
        gamma = rng.uniform(0.9, 1.1)
        quality = rng.randint(*map(int, policy["jpeg_quality_range"]))
        result = side_shadow(roi, minimum, dark_side)
        result = jpeg_roundtrip(gamma_adjust(result, gamma), quality)
        return result, "side_shadow_gamma_jpeg", {
            "shadow_minimum": round(minimum, 4),
            "dark_side": dark_side,
            "gamma": round(gamma, 4),
            "jpeg_quality": quality,
        }

    raise ValueError(f"Unsupported augmentation variant: {variant}")


def load_frame_rows(dataset_root: Path, config: dict[str, Any]) -> list[dict[str, str]]:
    base_rows = read_csv(dataset_root / config["base_manifest"])
    validation_rows = read_csv(dataset_root / config["validation_manifest"])
    test_ids = set(config["base_test_capture_ids"])
    validation_ids = set(config["validation_capture_ids"])

    base_ids = {row["capture_id"] for row in base_rows}
    observed_validation_ids = {row["capture_id"] for row in validation_rows}
    if not test_ids <= base_ids:
        raise ValueError(f"Unknown base test IDs: {sorted(test_ids - base_ids)}")
    if observed_validation_ids != validation_ids:
        raise ValueError(
            "Validation manifest IDs do not match config: "
            f"missing={sorted(validation_ids - observed_validation_ids)}, "
            f"extra={sorted(observed_validation_ids - validation_ids)}"
        )

    frames: list[dict[str, str]] = []
    for source_name, rows in (("base", base_rows), ("validation", validation_rows)):
        for row in rows:
            capture_id = row["capture_id"]
            final_split = (
                "validation"
                if source_name == "validation"
                else "test" if capture_id in test_ids else "train"
            )
            if row.get("safety_ignore", "").lower() != "false":
                raise ValueError(f"Safety-ignore frame cannot enter dataset: {capture_id}")
            states = [row.get(slot, "") for slot in SLOTS]
            if set(states) - LABELS:
                raise ValueError(f"Invalid states for {capture_id}: {states}")
            occupied = states.count("OCCUPIED")
            if int(row["occupied_count"]) != occupied:
                raise ValueError(f"occupied_count mismatch for {capture_id}")
            source_path = dataset_root / row["source_relpath"]
            if not source_path.is_file():
                raise ValueError(f"Missing source frame: {source_path}")
            normalized = {
                "capture_id": capture_id,
                "source_relpath": row["source_relpath"],
                "captured_session": row["captured_session"],
                "captured_light": row["captured_light"],
                "final_split": final_split,
                "safety_ignore": "false",
                "occupied_count": str(occupied),
                "label_source": row.get("label_source", ""),
                "review_status": row.get("review_status", ""),
            }
            normalized.update({slot: row[slot] for slot in SLOTS})
            frames.append(normalized)

    ids = [row["capture_id"] for row in frames]
    if len(ids) != len(set(ids)):
        raise ValueError("Duplicate capture IDs across final frame manifests")
    return sorted(frames, key=lambda row: row["capture_id"])


def write_frame_manifest(path: Path, frames: list[dict[str, str]]) -> None:
    fields = (
        "capture_id", "source_relpath", "captured_session", "captured_light",
        "final_split", "safety_ignore", "occupied_count", *SLOTS,
        "label_source", "review_status",
    )
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(frames)


def main() -> int:
    args = parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    roi_config = json.loads((args.dataset_root / config["roi_config"]).read_text(encoding="utf-8"))
    if roi_config.get("do_not_use") is True:
        raise ValueError("Selected ROI calibration is marked do_not_use")
    slots = {name: tuple(map(int, roi_config["slots"][name])) for name in SLOTS}
    expected_size = (int(roi_config["frame_width"]), int(roi_config["frame_height"]))
    augment_count = int(config["train_augmentations_per_roi"])
    if augment_count != 4:
        raise ValueError("dataset_v1 requires exactly four train augmentation variants")
    policy = config["augmentation_policy"]
    if policy.get("spatial_transforms") is not False:
        raise ValueError("Spatial transforms are forbidden for dataset_v1")
    if policy.get("validation_augmentation") is not False or policy.get("test_augmentation") is not False:
        raise ValueError("Validation/test augmentation must be disabled")

    if args.output_root.exists() and any(args.output_root.iterdir()):
        raise ValueError(f"Output directory is not empty: {args.output_root}")
    for split in SPLITS:
        for label in sorted(LABELS):
            (args.output_root / "images" / split / label).mkdir(parents=True, exist_ok=True)
    manifest_dir = args.output_root / "manifests"
    manifest_dir.mkdir(parents=True, exist_ok=True)

    frames = load_frame_rows(args.dataset_root, config)
    write_frame_manifest(manifest_dir / "frame_manifest.csv", frames)

    roi_fields = (
        "sample_id", "relative_path", "split", "label", "capture_id", "slot",
        "variant", "parent_sample_id", "source_frame_relpath", "source_frame_sha256",
        "output_sha256", "augmentation_json",
    )
    roi_rows: list[dict[str, str]] = []
    source_hashes: dict[Path, str] = {}
    seed = int(config["augmentation_seed"])

    for frame_row in frames:
        source_path = args.dataset_root / frame_row["source_relpath"]
        with Image.open(source_path) as decoded:
            image = decoded.convert("RGB")
        if image.size != expected_size:
            raise ValueError(f"{source_path}: expected {expected_size}, got {image.size}")
        frame = np.asarray(image, dtype=np.uint8)
        frame_hash = source_hashes.setdefault(source_path, sha256_file(source_path))
        split = frame_row["final_split"]

        for slot in SLOTS:
            x0, y0, x1, y1 = slots[slot]
            roi = np.ascontiguousarray(frame[y0:y1, x0:x1, :])
            label = frame_row[slot]
            original_id = f"{frame_row['capture_id']}_{slot}_original"
            relative = Path("images") / split / label / f"{original_id}.png"
            output_path = args.output_root / relative
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
                "output_sha256": sha256_file(output_path),
                "augmentation_json": "",
            })

            if split != "train":
                continue
            for variant_number in range(1, augment_count + 1):
                rng = stable_rng(seed, frame_row["capture_id"], slot, variant_number)
                derived, variant_name, parameters = augment(
                    roi, variant_number, rng, policy
                )
                sample_id = (
                    f"{frame_row['capture_id']}_{slot}_aug{variant_number:02d}_{variant_name}"
                )
                relative = Path("images") / split / label / f"{sample_id}.png"
                output_path = args.output_root / relative
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
                    "output_sha256": sha256_file(output_path),
                    "augmentation_json": json.dumps(parameters, sort_keys=True),
                })

    roi_manifest = manifest_dir / "roi_manifest.csv"
    with roi_manifest.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=roi_fields)
        writer.writeheader()
        writer.writerows(roi_rows)

    split_frames = {split: sum(row["final_split"] == split for row in frames) for split in SPLITS}
    split_samples = {split: sum(row["split"] == split for row in roi_rows) for split in SPLITS}
    summary = {
        "dataset_id": config["dataset_id"],
        "roi_calibration_id": roi_config["calibration_id"],
        "frame_counts": split_frames,
        "sample_counts": split_samples,
        "total_samples": len(roi_rows),
        "augmentation_seed": seed,
    }
    (args.output_root / "dataset_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(f"PASS frames: {split_frames}")
    print(f"PASS ROI samples: {split_samples}, total={len(roi_rows)}")
    print(f"PASS output: {args.output_root}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
