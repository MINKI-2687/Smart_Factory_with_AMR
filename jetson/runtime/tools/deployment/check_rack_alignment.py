#!/usr/bin/env python3
"""Fail closed when the rack moved relative to the ROI reference frame.

This intentionally uses only Pillow and NumPy. A yellow-green rack mask is
downsampled, then a bounded translation search finds the best alignment.
The check is a pose guard, not a replacement for visual ROI calibration.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np
from PIL import Image


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--current", required=True, type=Path)
    parser.add_argument("--max-allowed-shift", type=int, default=8)
    parser.add_argument("--search-radius", type=int, default=120)
    parser.add_argument("--minimum-score", type=float, default=0.35)
    parser.add_argument("--downsample", type=int, default=4)
    return parser.parse_args()


def rack_mask_rgb(image: np.ndarray, downsample: int) -> np.ndarray:
    image = np.asarray(image, dtype=np.float32)
    if image.shape[:2] != (720, 1280):
        raise ValueError(
            f"expected 1280x720 RGB input, got {image.shape[1]}x{image.shape[0]}"
        )
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f"expected RGB input, got shape {image.shape}")

    red, green, blue = image[:, :, 0], image[:, :, 1], image[:, :, 2]
    # The fluorescent dividers vary from yellow to green as auto exposure and
    # side lighting change. Detect their red/green separation from blue rather
    # than requiring green to dominate red. This preserves the rack structure
    # under bright, dark, left-lit, and right-lit validation conditions.
    mask = (
        (green > 90)
        & (red > 70)
        & ((green - blue) > 25)
        & ((red - blue) > 10)
        & (green > red * 0.70)
    )
    # Remove most table reflections and side background before alignment.
    mask = mask[0:700, 180:1240]
    height = mask.shape[0] // downsample * downsample
    width = mask.shape[1] // downsample * downsample
    blocks = mask[:height, :width].reshape(
        height // downsample, downsample, width // downsample, downsample
    )
    return blocks.mean(axis=(1, 3)) > 0.15


def rack_mask(path: Path, downsample: int) -> np.ndarray:
    with Image.open(path) as decoded:
        image = np.asarray(decoded.convert("RGB"), dtype=np.float32)
    return rack_mask_rgb(image, downsample)


def dice_at_shift(reference: np.ndarray, current: np.ndarray, dx: int, dy: int) -> float:
    y0 = max(0, dy)
    y1 = min(reference.shape[0], reference.shape[0] + dy)
    x0 = max(0, dx)
    x1 = min(reference.shape[1], reference.shape[1] + dx)
    ref_view = reference[y0:y1, x0:x1]
    cur_view = current[y0 - dy:y1 - dy, x0 - dx:x1 - dx]
    denominator = int(ref_view.sum()) + int(cur_view.sum())
    if denominator == 0:
        return 0.0
    return 2.0 * int(np.logical_and(ref_view, cur_view).sum()) / denominator


def measure_alignment(
    reference_path: Path,
    current_path: Path,
    *,
    downsample: int = 4,
    search_radius: int = 120,
) -> tuple[float, int, int]:
    """Return rack-mask Dice score and current-frame x/y translation."""
    if downsample <= 0:
        raise ValueError("downsample must be positive")
    reference = rack_mask(reference_path, downsample)
    current = rack_mask(current_path, downsample)
    return measure_alignment_masks(
        reference,
        current,
        downsample=downsample,
        search_radius=search_radius,
    )


def measure_alignment_masks(
    reference: np.ndarray,
    current: np.ndarray,
    *,
    downsample: int = 4,
    search_radius: int = 120,
) -> tuple[float, int, int]:
    """Measure translation between two already-downsampled rack masks."""
    if reference.shape != current.shape:
        raise ValueError(
            f"rack mask shapes differ: reference={reference.shape} current={current.shape}"
        )
    search = search_radius // downsample
    best_score, best_dx, best_dy = -1.0, 0, 0
    for dy in range(-search, search + 1):
        for dx in range(-search, search + 1):
            score = dice_at_shift(reference, current, dx, dy)
            if score > best_score:
                best_score, best_dx, best_dy = score, dx, dy
    return best_score, -best_dx * downsample, -best_dy * downsample


def main() -> int:
    args = parse_args()
    best_score, shift_x, shift_y = measure_alignment(
        args.reference,
        args.current,
        downsample=args.downsample,
        search_radius=args.search_radius,
    )
    print(f"alignment_score={best_score:.3f} shift_x={shift_x} shift_y={shift_y}")

    if best_score < args.minimum_score:
        print("FAIL CAMERA_POSE_UNKNOWN: rack-mask alignment score is too low", file=sys.stderr)
        return 2
    if abs(shift_x) > args.max_allowed_shift or abs(shift_y) > args.max_allowed_shift:
        print("FAIL CAMERA_MOVED: fixed ROI must not be used", file=sys.stderr)
        return 3
    print("PASS CAMERA_ALIGNED: fixed ROI may be evaluated")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as exc:
        print(f"FAIL CAMERA_POSE_UNKNOWN: {exc}", file=sys.stderr)
        raise SystemExit(2)
