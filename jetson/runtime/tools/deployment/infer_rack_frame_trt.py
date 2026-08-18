#!/usr/bin/env python3
"""Run fail-closed TensorRT inference on the nine fixed rack ROIs.

This command never sends UART/OpenRB commands. A single frame is diagnostic
only and therefore always reports command_authorized=false.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

from check_rack_alignment import measure_alignment
from validate_tensorrt_parity import (
    TensorRTRunner,
    decide,
    preprocess_pil_image,
    require,
    sha256_file,
    write_json_new,
)


SLOT_ORDER = (
    "C1_L1",
    "C1_L2",
    "C1_L3",
    "C2_L1",
    "C2_L2",
    "C2_L3",
    "C3_L1",
    "C3_L2",
    "C3_L3",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--engine", required=True, type=Path)
    parser.add_argument("--frame", required=True, type=Path)
    parser.add_argument("--alignment-reference", required=True, type=Path)
    parser.add_argument("--deployment-metadata", required=True, type=Path)
    parser.add_argument("--thresholds", required=True, type=Path)
    parser.add_argument("--report-json", required=True, type=Path)
    parser.add_argument("--max-allowed-shift", type=int, default=8)
    parser.add_argument("--minimum-alignment-score", type=float, default=0.35)
    return parser.parse_args()


def validate_rois(calibration: dict[str, Any]) -> dict[str, list[int]]:
    require(
        calibration["coordinate_format"] == "x0_y0_x1_y1_exclusive",
        "Unexpected ROI coordinate format",
    )
    require(
        (calibration["frame_width"], calibration["frame_height"]) == (1280, 720),
        "Unexpected ROI calibration frame size",
    )
    slots = calibration["slots"]
    require(set(slots) == set(SLOT_ORDER), "ROI calibration must define exactly nine slots")
    for slot in SLOT_ORDER:
        coordinates = slots[slot]
        require(len(coordinates) == 4, f"{slot}: expected four ROI coordinates")
        x0, y0, x1, y1 = map(int, coordinates)
        require(0 <= x0 < x1 <= 1280, f"{slot}: invalid x coordinates")
        require(0 <= y0 < y1 <= 720, f"{slot}: invalid y coordinates")
    return slots


def main() -> int:
    args = parse_args()
    for path in (
        args.engine,
        args.frame,
        args.alignment_reference,
        args.deployment_metadata,
        args.thresholds,
    ):
        require(path.is_file(), f"Required file is missing: {path}")
    require(not args.report_json.exists(), f"Refusing to overwrite {args.report_json}")

    metadata = json.loads(args.deployment_metadata.read_text(encoding="utf-8"))
    thresholds_doc = json.loads(args.thresholds.read_text(encoding="utf-8"))
    require(metadata["status"] == "ONNX_BASELINE_READY", "Unexpected source model status")
    require(metadata["deployment_authorized"] is False, "Source safety status changed")
    require(metadata["model"]["onnx_sha256"] == thresholds_doc["model"]["onnx_sha256"],
            "Model metadata mismatch")
    preprocessing = metadata["preprocessing"]
    calibration = metadata["roi_calibration"]
    slots = validate_rois(calibration)
    t_empty = float(thresholds_doc["thresholds"]["t_empty"])
    t_occupied = float(thresholds_doc["thresholds"]["t_occupied"])
    require((t_empty, t_occupied) == (0.005, 0.995), "Frozen thresholds changed")

    alignment_score, shift_x, shift_y = measure_alignment(
        args.alignment_reference,
        args.frame,
        downsample=4,
        search_radius=120,
    )
    camera_aligned = (
        alignment_score >= args.minimum_alignment_score
        and abs(shift_x) <= args.max_allowed_shift
        and abs(shift_y) <= args.max_allowed_shift
    )
    if not camera_aligned:
        report = {
            "schema_version": 1,
            "status": "CAMERA_FAULT",
            "frame": {
                "path": str(args.frame.resolve()),
                "sha256": sha256_file(args.frame),
            },
            "alignment": {
                "passed": False,
                "score": alignment_score,
                "shift_x": shift_x,
                "shift_y": shift_y,
                "minimum_score": args.minimum_alignment_score,
                "max_allowed_shift": args.max_allowed_shift,
            },
            "slots": {slot: "UNKNOWN" for slot in SLOT_ORDER},
            "command_authorized": False,
            "command_block_reason": "CAMERA_FAULT",
            "stm32_bytes_sent": 0,
        }
        write_json_new(args.report_json, report)
        print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
        print("FAIL CLOSED: CAMERA_FAULT; no TensorRT inference; no OpenRB command")
        return 2

    tensors = []
    with Image.open(args.frame) as frame:
        require(frame.size == (1280, 720), f"Expected 1280x720 frame, got {frame.size}")
        rgb = frame.convert("RGB")
        for slot in SLOT_ORDER:
            x0, y0, x1, y1 = slots[slot]
            tensors.append(preprocess_pil_image(rgb.crop((x0, y0, x1, y1)), preprocessing))

    runner = TensorRTRunner(args.engine)
    try:
        runner.run(np.stack(tensors))
        probabilities, elapsed_ms = runner.run(np.stack(tensors))
    finally:
        runner.close()

    slot_results = {}
    decisions = Counter()
    for slot, probability in zip(SLOT_ORDER, probabilities, strict=True):
        value = float(probability)
        require(np.isfinite(value) and 0.0 <= value <= 1.0, f"{slot}: invalid output")
        decision = decide(value, t_empty, t_occupied)
        decisions[decision] += 1
        slot_results[slot] = {
            "p_occupied": value,
            "decision": decision,
        }

    report = {
        "schema_version": 1,
        "status": "SINGLE_FRAME_INFERENCE_COMPLETE",
        "scope": "current live frame; not train/validation/test",
        "frame": {
            "path": str(args.frame.resolve()),
            "sha256": sha256_file(args.frame),
            "width": 1280,
            "height": 720,
        },
        "alignment": {
            "passed": True,
            "score": alignment_score,
            "shift_x": shift_x,
            "shift_y": shift_y,
            "minimum_score": args.minimum_alignment_score,
            "max_allowed_shift": args.max_allowed_shift,
        },
        "engine": {
            "path": str(args.engine.resolve()),
            "sha256": sha256_file(args.engine),
            "batch_size": 9,
            "inference_ms": elapsed_ms,
        },
        "thresholds": {"t_empty": t_empty, "t_occupied": t_occupied},
        "decision_counts": dict(decisions),
        "slots": slot_results,
        "command_authorized": False,
        "command_block_reason": "TEMPORAL_FILTER_NOT_YET_SATISFIED",
        "stm32_bytes_sent": 0,
    }
    write_json_new(args.report_json, report)
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    print("PASS: SINGLE_FRAME_INFERENCE_COMPLETE")
    print("BLOCKED BY DESIGN: temporal filter not yet satisfied; no OpenRB command")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
