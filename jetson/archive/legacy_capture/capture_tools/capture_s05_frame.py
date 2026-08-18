#!/usr/bin/env python3
"""Capture one preplanned s05 test frame through V4L2 without OpenCV.

The script never overwrites an existing image and never applies post-capture
brightness, contrast, gamma, shadow, or JPEG augmentation. The planned light
condition must be explicitly confirmed at acquisition time.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
from pathlib import Path
import subprocess
import sys

from PIL import Image


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--plan", required=True, type=Path)
    parser.add_argument("--capture-id", required=True)
    parser.add_argument("--confirm-light", required=True)
    parser.add_argument("--device", default="/dev/video0", type=Path)
    return parser.parse_args()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    args = parse_args()
    root = args.dataset_root.resolve()
    with args.plan.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    matches = [row for row in rows if row["capture_id"] == args.capture_id]
    if len(matches) != 1:
        raise ValueError(f"capture_id must match exactly one plan row: {args.capture_id}")
    row = matches[0]
    if args.confirm_light != row["captured_light"]:
        raise ValueError(
            f"Light confirmation mismatch: plan={row['captured_light']} "
            f"confirmed={args.confirm_light}"
        )
    output_path = (root / row["source_relpath"]).resolve()
    if not output_path.is_relative_to(root):
        raise ValueError(f"Output path escapes dataset root: {output_path}")
    if output_path.exists():
        raise ValueError(f"Refusing to overwrite captured frame: {output_path}")
    if not args.device.exists():
        raise ValueError(f"Camera device is unavailable: {args.device}")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_name(f".{output_path.name}.partial")
    if temporary.exists():
        raise ValueError(f"Stale partial capture exists: {temporary}")

    command = [
        "v4l2-ctl",
        f"--device={args.device}",
        "--set-fmt-video=width=1280,height=720,pixelformat=MJPG",
        "--set-parm=30",
        "--stream-mmap=3",
        "--stream-skip=60",
        "--stream-count=1",
        f"--stream-to={temporary}",
    ]
    try:
        subprocess.run(command, check=True)
        if not temporary.is_file() or temporary.stat().st_size == 0:
            raise ValueError("V4L2 returned without a non-empty capture")
        with Image.open(temporary) as image:
            if image.format != "JPEG" or image.size != (1280, 720):
                raise ValueError(
                    f"Unexpected capture format/size: {image.format} {image.size}"
                )
            image.verify()
        temporary.rename(output_path)
    except Exception:
        if temporary.exists():
            temporary.unlink()
        raise

    print(f"PASS capture_id: {args.capture_id}")
    print(f"PASS light condition confirmed at acquisition: {args.confirm_light}")
    print("PASS source: original MJPEG frame; no post-capture photometric processing")
    print(f"PASS geometry: 1280x720 JPEG, bytes={output_path.stat().st_size}")
    print(f"PASS sha256: {sha256_file(output_path)}")
    print(f"PASS output: {output_path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
