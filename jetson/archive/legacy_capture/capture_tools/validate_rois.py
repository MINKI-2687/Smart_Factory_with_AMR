#!/usr/bin/env python3
"""Generate lossless ROI crops and annotated previews without OpenCV.

Pillow is used only for JPEG decoding/encoding and annotations. The actual ROI
operation is NumPy buffer slicing with exclusive [x0, y0, x1, y1] bounds.
Original frames are never modified.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont


DISPLAY_ORDER = (
    "C1_L3", "C2_L3", "C3_L3",
    "C1_L2", "C2_L2", "C3_L2",
    "C1_L1", "C2_L1", "C3_L1",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--input-dir", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument(
        "--ids",
        nargs="+",
        required=True,
        help="Capture IDs such as a001 a002 (matched as *_a001.jpg).",
    )
    return parser.parse_args()


def load_config(path: Path) -> tuple[dict[str, tuple[int, int, int, int]], tuple[int, int]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("do_not_use") is True:
        raise ValueError(f"Calibration {data.get('calibration_id', path.name)} is marked do_not_use")
    expected_size = (int(data["frame_width"]), int(data["frame_height"]))
    slots = {name: tuple(map(int, coords)) for name, coords in data["slots"].items()}
    if tuple(slots) != DISPLAY_ORDER:
        raise ValueError("ROI slot order must follow the physical 3x3 display order")
    for name, (x0, y0, x1, y1) in slots.items():
        if not (0 <= x0 < x1 <= expected_size[0] and 0 <= y0 < y1 <= expected_size[1]):
            raise ValueError(f"Invalid bounds for {name}: {slots[name]}")
    return slots, expected_size


def find_frame(input_dir: Path, capture_id: str) -> Path:
    matches = sorted(input_dir.glob(f"*_{capture_id}.jpg"))
    if len(matches) != 1:
        raise ValueError(f"Expected exactly one frame for {capture_id}, found {len(matches)}")
    return matches[0]


def draw_overview(image: Image.Image, slots: dict[str, tuple[int, int, int, int]]) -> Image.Image:
    overview = image.copy()
    draw = ImageDraw.Draw(overview)
    font = ImageFont.load_default()
    for name, (x0, y0, x1, y1) in slots.items():
        draw.rectangle((x0, y0, x1 - 1, y1 - 1), outline=(255, 0, 255), width=3)
        draw.rectangle((x0, y0, x0 + 52, y0 + 13), fill=(0, 0, 0))
        draw.text((x0 + 2, y0 + 2), name, fill=(255, 255, 255), font=font)
    return overview


def make_contact_sheet(crops: dict[str, Image.Image]) -> Image.Image:
    tile_w, tile_h = 320, 240
    label_h = 22
    resampling = getattr(Image, "Resampling", Image)
    sheet = Image.new("RGB", (tile_w * 3, (tile_h + label_h) * 3), "white")
    draw = ImageDraw.Draw(sheet)
    font = ImageFont.load_default()
    for index, name in enumerate(DISPLAY_ORDER):
        row, col = divmod(index, 3)
        crop = crops[name]
        display = crop.copy()
        display.thumbnail((tile_w, tile_h), resampling.LANCZOS)
        left = col * tile_w + (tile_w - display.width) // 2
        top = row * (tile_h + label_h) + label_h + (tile_h - display.height) // 2
        sheet.paste(display, (left, top))
        draw.text((col * tile_w + 6, row * (tile_h + label_h) + 5), name, fill="black", font=font)
    return sheet


def main() -> int:
    args = parse_args()
    slots, expected_size = load_config(args.config)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    processed = 0
    for capture_id in args.ids:
        frame_path = find_frame(args.input_dir, capture_id)
        with Image.open(frame_path) as decoded:
            image = decoded.convert("RGB")
        if image.size != expected_size:
            raise ValueError(f"{frame_path}: expected {expected_size}, got {image.size}")

        frame = np.asarray(image)
        capture_dir = args.output_dir / capture_id
        capture_dir.mkdir(parents=True, exist_ok=True)
        crops: dict[str, Image.Image] = {}
        for name, (x0, y0, x1, y1) in slots.items():
            roi_buffer = np.ascontiguousarray(frame[y0:y1, x0:x1, :])
            crop = Image.fromarray(roi_buffer, mode="RGB")
            crop.save(capture_dir / f"{name}.png", format="PNG", optimize=True)
            crops[name] = crop

        draw_overview(image, slots).save(
            args.output_dir / f"{capture_id}_overview.jpg", quality=95, subsampling=0
        )
        make_contact_sheet(crops).save(
            args.output_dir / f"{capture_id}_roi_sheet.jpg", quality=95, subsampling=0
        )
        print(f"PASS {capture_id}: {frame_path.name}, 9 crops, frame={image.width}x{image.height}")
        processed += 1

    print(f"PASS total: {processed} frames, {processed * 9} ROI crops")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
