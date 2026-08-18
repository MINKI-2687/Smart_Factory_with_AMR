#!/usr/bin/env python3
"""Validate a Jetson TensorRT engine against frozen Colab ONNX predictions.

This script intentionally uses Pillow, NumPy, TensorRT, and the CUDA runtime.
It does not import OpenCV, touch the held-out test split, or tune thresholds.
"""

from __future__ import annotations

import argparse
import csv
import ctypes
import hashlib
import json
import math
import os
import statistics
import time
from collections import Counter
from pathlib import Path
from typing import Any, Iterable

import numpy as np
from PIL import Image
import tensorrt as trt


CUDA_MEMCPY_HOST_TO_DEVICE = 1
CUDA_MEMCPY_DEVICE_TO_HOST = 2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--engine", required=True, type=Path)
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--dataset-config", required=True, type=Path)
    parser.add_argument("--reference-csv", required=True, type=Path)
    parser.add_argument("--preprocessing", required=True, type=Path)
    parser.add_argument("--thresholds", required=True, type=Path)
    parser.add_argument("--report-json", required=True, type=Path)
    parser.add_argument("--predictions-csv", required=True, type=Path)
    return parser.parse_args()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def percentile(values: list[float], quantile: float) -> float:
    if not values:
        raise ValueError("Cannot calculate a percentile of an empty list")
    ordered = sorted(values)
    position = (len(ordered) - 1) * quantile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


class CudaRuntime:
    def __init__(self) -> None:
        self.lib = ctypes.CDLL("libcudart.so")
        self.lib.cudaGetErrorString.argtypes = [ctypes.c_int]
        self.lib.cudaGetErrorString.restype = ctypes.c_char_p
        self.lib.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
        self.lib.cudaMalloc.restype = ctypes.c_int
        self.lib.cudaFree.argtypes = [ctypes.c_void_p]
        self.lib.cudaFree.restype = ctypes.c_int
        self.lib.cudaMemcpyAsync.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_size_t,
            ctypes.c_int,
            ctypes.c_void_p,
        ]
        self.lib.cudaMemcpyAsync.restype = ctypes.c_int
        self.lib.cudaStreamCreate.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
        self.lib.cudaStreamCreate.restype = ctypes.c_int
        self.lib.cudaStreamSynchronize.argtypes = [ctypes.c_void_p]
        self.lib.cudaStreamSynchronize.restype = ctypes.c_int
        self.lib.cudaStreamDestroy.argtypes = [ctypes.c_void_p]
        self.lib.cudaStreamDestroy.restype = ctypes.c_int

    def check(self, status: int, operation: str) -> None:
        if status != 0:
            raw = self.lib.cudaGetErrorString(status)
            detail = raw.decode("utf-8", errors="replace") if raw else "unknown"
            raise RuntimeError(f"{operation} failed: CUDA {status}: {detail}")

    def malloc(self, size: int) -> ctypes.c_void_p:
        pointer = ctypes.c_void_p()
        self.check(self.lib.cudaMalloc(ctypes.byref(pointer), size), "cudaMalloc")
        return pointer

    def free(self, pointer: ctypes.c_void_p) -> None:
        if pointer.value:
            self.check(self.lib.cudaFree(pointer), "cudaFree")

    def stream_create(self) -> ctypes.c_void_p:
        stream = ctypes.c_void_p()
        self.check(self.lib.cudaStreamCreate(ctypes.byref(stream)), "cudaStreamCreate")
        return stream

    def stream_destroy(self, stream: ctypes.c_void_p) -> None:
        if stream.value:
            self.check(self.lib.cudaStreamDestroy(stream), "cudaStreamDestroy")

    def copy_async(
        self,
        destination: int,
        source: int,
        size: int,
        kind: int,
        stream: ctypes.c_void_p,
    ) -> None:
        self.check(
            self.lib.cudaMemcpyAsync(
                ctypes.c_void_p(destination),
                ctypes.c_void_p(source),
                size,
                kind,
                stream,
            ),
            "cudaMemcpyAsync",
        )

    def synchronize(self, stream: ctypes.c_void_p) -> None:
        self.check(self.lib.cudaStreamSynchronize(stream), "cudaStreamSynchronize")


class TensorRTRunner:
    def __init__(self, engine_path: Path) -> None:
        self.cuda = CudaRuntime()
        self.logger = trt.Logger(trt.Logger.ERROR)
        self.runtime = trt.Runtime(self.logger)
        self.engine = self.runtime.deserialize_cuda_engine(engine_path.read_bytes())
        require(self.engine is not None, "TensorRT engine deserialization failed")
        require(self.engine.num_io_tensors == 2, "Expected exactly one input and one output")

        inputs = []
        outputs = []
        for index in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(index)
            mode = self.engine.get_tensor_mode(name)
            (inputs if mode == trt.TensorIOMode.INPUT else outputs).append(name)
        require(inputs == ["images"], f"Unexpected engine inputs: {inputs}")
        require(outputs == ["p_occupied"], f"Unexpected engine outputs: {outputs}")
        require(self.engine.get_tensor_dtype("images") == trt.float32, "Input must be FP32")
        require(
            self.engine.get_tensor_dtype("p_occupied") == trt.float32,
            "Output must be FP32",
        )
        minimum, optimum, maximum = self.engine.get_tensor_profile_shape("images", 0)
        require(tuple(minimum) == (1, 3, 224, 224), f"Unexpected min shape: {minimum}")
        require(tuple(optimum) == (9, 3, 224, 224), f"Unexpected opt shape: {optimum}")
        require(tuple(maximum) == (9, 3, 224, 224), f"Unexpected max shape: {maximum}")

        self.context = self.engine.create_execution_context()
        require(self.context is not None, "TensorRT execution context creation failed")
        self.stream = self.cuda.stream_create()
        self.input_capacity = int(np.prod(maximum)) * np.dtype(np.float32).itemsize
        self.output_capacity = maximum[0] * np.dtype(np.float32).itemsize
        self.device_input = self.cuda.malloc(self.input_capacity)
        self.device_output = self.cuda.malloc(self.output_capacity)
        require(
            self.context.set_tensor_address("images", int(self.device_input.value)),
            "Could not bind TensorRT input",
        )
        require(
            self.context.set_tensor_address("p_occupied", int(self.device_output.value)),
            "Could not bind TensorRT output",
        )

    def close(self) -> None:
        self.cuda.free(self.device_output)
        self.cuda.free(self.device_input)
        self.cuda.stream_destroy(self.stream)

    def run(self, batch: np.ndarray) -> tuple[np.ndarray, float]:
        batch = np.ascontiguousarray(batch, dtype=np.float32)
        require(batch.ndim == 4, f"Expected NCHW input, got {batch.shape}")
        require(1 <= batch.shape[0] <= 9, f"Unsupported batch size: {batch.shape[0]}")
        require(batch.shape[1:] == (3, 224, 224), f"Unexpected input shape: {batch.shape}")
        require(batch.nbytes <= self.input_capacity, "Input exceeds allocated CUDA buffer")
        require(
            self.context.set_input_shape("images", tuple(batch.shape)),
            f"TensorRT rejected input shape {batch.shape}",
        )
        output_shape = tuple(self.context.get_tensor_shape("p_occupied"))
        require(output_shape == (batch.shape[0], 1), f"Unexpected output shape: {output_shape}")
        output = np.empty(output_shape, dtype=np.float32)

        started = time.perf_counter()
        self.cuda.copy_async(
            int(self.device_input.value),
            int(batch.ctypes.data),
            batch.nbytes,
            CUDA_MEMCPY_HOST_TO_DEVICE,
            self.stream,
        )
        require(
            self.context.execute_async_v3(stream_handle=int(self.stream.value)),
            "TensorRT execution failed",
        )
        self.cuda.copy_async(
            int(output.ctypes.data),
            int(self.device_output.value),
            output.nbytes,
            CUDA_MEMCPY_DEVICE_TO_HOST,
            self.stream,
        )
        self.cuda.synchronize(self.stream)
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        return output[:, 0].copy(), elapsed_ms


def preprocess_pil_image(image: Image.Image, config: dict[str, Any]) -> np.ndarray:
    require(config["source_color_space"] == "RGB", "Only RGB input is supported")
    require(config["tensor_layout"] == "NCHW", "Only NCHW input is supported")
    require(config["input_dtype"] == "float32", "Only float32 input is supported")
    resize = config["resize"]
    require(resize["interpolation"] == "bilinear", "Only bilinear resize is supported")
    require(resize["antialias"] is True, "Antialias must remain enabled")
    require(resize["center_crop"] is False, "Center crop must remain disabled")
    width = int(resize["width"])
    height = int(resize["height"])
    require((width, height) == (224, 224), "Engine requires 224x224 input")

    rgb = image.convert("RGB")
    resampling = getattr(Image, "Resampling", Image)
    resized = rgb.resize((width, height), resample=resampling.BILINEAR)
    array = np.asarray(resized, dtype=np.float32) / np.float32(255.0)
    mean = np.asarray(config["normalization"]["mean"], dtype=np.float32)
    std = np.asarray(config["normalization"]["std"], dtype=np.float32)
    array = (array - mean) / std
    return np.ascontiguousarray(array.transpose(2, 0, 1), dtype=np.float32)


def preprocess_image(path: Path, config: dict[str, Any]) -> np.ndarray:
    with Image.open(path) as image:
        return preprocess_pil_image(image, config)


def decide(probability: float, t_empty: float, t_occupied: float) -> str:
    if probability <= t_empty:
        return "EMPTY"
    if probability >= t_occupied:
        return "OCCUPIED"
    return "UNKNOWN"


def chunks(values: list[np.ndarray], size: int) -> Iterable[list[np.ndarray]]:
    for index in range(0, len(values), size):
        yield values[index : index + size]


def load_reference(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    required = {
        "relative_path",
        "capture_id",
        "slot",
        "label",
        "p_occupied",
        "decision",
    }
    require(rows, "Reference prediction CSV is empty")
    require(required <= set(rows[0]), f"Reference CSV lacks columns: {required - set(rows[0])}")
    require(len(rows) == 90, f"Expected 90 validation rows, got {len(rows)}")
    paths = [row["relative_path"] for row in rows]
    require(len(paths) == len(set(paths)), "Reference CSV contains duplicate paths")
    for row in rows:
        require("/validation/" in row["relative_path"], "Non-validation path in reference CSV")
        require(row["relative_path"].endswith("_original.png"), "Augmented validation item found")
        require(row["label"] in {"EMPTY", "OCCUPIED"}, "Unexpected ground-truth label")
    return rows


def load_roi_hashes(path: Path) -> dict[str, str]:
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    hashes: dict[str, str] = {}
    for row in rows:
        relative_path = row["relative_path"]
        if row["split"] == "validation":
            require(row["variant"] == "original", "Validation augmentation is forbidden")
            hashes[relative_path] = row["output_sha256"]
    require(len(hashes) == 90, f"Expected 90 validation manifest rows, got {len(hashes)}")
    return hashes


def write_json_new(path: Path, value: Any) -> None:
    require(not path.exists(), f"Refusing to overwrite {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as handle:
        json.dump(value, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write("\n")


def write_csv_new(path: Path, rows: list[dict[str, Any]]) -> None:
    require(not path.exists(), f"Refusing to overwrite {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    require(rows, "No prediction rows to write")
    with path.open("x", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    args = parse_args()
    for path in (
        args.engine,
        args.dataset_config,
        args.reference_csv,
        args.preprocessing,
        args.thresholds,
    ):
        require(path.is_file(), f"Required file is missing: {path}")
    require(args.dataset_root.is_dir(), f"Dataset root is missing: {args.dataset_root}")
    require(not args.report_json.exists(), f"Refusing to overwrite {args.report_json}")
    require(
        not args.predictions_csv.exists(), f"Refusing to overwrite {args.predictions_csv}"
    )

    preprocessing = json.loads(args.preprocessing.read_text(encoding="utf-8"))
    thresholds_doc = json.loads(args.thresholds.read_text(encoding="utf-8"))
    thresholds = thresholds_doc["thresholds"]
    t_empty = float(thresholds["t_empty"])
    t_occupied = float(thresholds["t_occupied"])
    backend_epsilon = float(thresholds_doc["selection"]["epsilon_total"])
    require((t_empty, t_occupied) == (0.005, 0.995), "Frozen thresholds changed")
    require(backend_epsilon == 0.02, "Frozen backend epsilon changed")
    require(
        thresholds_doc["test_policy"]["retuning_from_test_forbidden"] is True,
        "Test retuning prohibition is missing",
    )

    expected_data = thresholds_doc["data"]
    roi_manifest = args.dataset_root / "manifests" / "roi_manifest.csv"
    dataset_summary = args.dataset_root / "dataset_summary.json"
    dataset_config = args.dataset_config
    source_integrity = {
        "roi_manifest_sha256": sha256_file(roi_manifest),
        "dataset_summary_sha256": sha256_file(dataset_summary),
        "dataset_config_sha256": sha256_file(dataset_config),
    }
    require(
        source_integrity["roi_manifest_sha256"] == expected_data["roi_manifest_sha256"],
        "ROI manifest hash mismatch",
    )
    require(
        source_integrity["dataset_summary_sha256"]
        == expected_data["dataset_summary_sha256"],
        "Dataset summary hash mismatch",
    )
    require(
        source_integrity["dataset_config_sha256"]
        == expected_data["dataset_config_sha256"],
        "Dataset config hash mismatch",
    )

    reference = load_reference(args.reference_csv)
    validation_hashes = load_roi_hashes(roi_manifest)
    tensors = []
    for row in reference:
        image_path = args.dataset_root / row["relative_path"]
        require(image_path.is_file(), f"Validation image is missing: {image_path}")
        require(
            sha256_file(image_path) == validation_hashes[row["relative_path"]],
            f"Validation image hash mismatch: {row['relative_path']}",
        )
        ref_probability = float(row["p_occupied"])
        require(math.isfinite(ref_probability), "Non-finite ONNX reference probability")
        require(0.0 <= ref_probability <= 1.0, "ONNX reference probability out of range")
        require(
            decide(ref_probability, t_empty, t_occupied) == row["decision"],
            f"Frozen ONNX decision mismatch: {row['relative_path']}",
        )
        tensors.append(preprocess_image(image_path, preprocessing))

    runner = TensorRTRunner(args.engine)
    try:
        runner.run(tensors[0][None, ...])
        runner.run(np.stack(tensors[:9]))

        batch1_probabilities = []
        batch1_timings_ms = []
        for tensor in tensors:
            probabilities, elapsed_ms = runner.run(tensor[None, ...])
            batch1_probabilities.append(float(probabilities[0]))
            batch1_timings_ms.append(elapsed_ms)

        batch9_probabilities = []
        batch9_timings_ms = []
        for group in chunks(tensors, 9):
            require(len(group) == 9, "Validation count must form complete batches of 9")
            probabilities, elapsed_ms = runner.run(np.stack(group))
            batch9_probabilities.extend(float(value) for value in probabilities)
            batch9_timings_ms.append(elapsed_ms)
    finally:
        runner.close()

    prediction_rows: list[dict[str, Any]] = []
    ref_differences = []
    batch_differences = []
    batch1_mismatches = 0
    batch9_mismatches = 0
    cross_errors = Counter()
    decisions = Counter()
    for row, batch1, batch9 in zip(
        reference, batch1_probabilities, batch9_probabilities, strict=True
    ):
        require(math.isfinite(batch1) and math.isfinite(batch9), "Non-finite TRT output")
        require(0.0 <= batch1 <= 1.0, "Batch-1 TRT output out of range")
        require(0.0 <= batch9 <= 1.0, "Batch-9 TRT output out of range")
        reference_probability = float(row["p_occupied"])
        decision1 = decide(batch1, t_empty, t_occupied)
        decision9 = decide(batch9, t_empty, t_occupied)
        reference_decision = row["decision"]
        diff_reference = abs(batch9 - reference_probability)
        diff_batches = abs(batch9 - batch1)
        ref_differences.append(diff_reference)
        batch_differences.append(diff_batches)
        batch1_mismatches += int(decision1 != reference_decision)
        batch9_mismatches += int(decision9 != reference_decision)
        decisions[decision9] += 1
        if row["label"] != decision9 and decision9 != "UNKNOWN":
            cross_errors[f"{row['label']}_to_{decision9}"] += 1
        prediction_rows.append(
            {
                "relative_path": row["relative_path"],
                "capture_id": row["capture_id"],
                "slot": row["slot"],
                "label": row["label"],
                "p_occupied_onnx_colab": f"{reference_probability:.12g}",
                "p_occupied_trt_batch1": f"{batch1:.12g}",
                "p_occupied_trt_batch9": f"{batch9:.12g}",
                "abs_diff_trt_batch9_vs_onnx": f"{diff_reference:.12g}",
                "abs_diff_trt_batch9_vs_batch1": f"{diff_batches:.12g}",
                "onnx_decision": reference_decision,
                "trt_batch1_decision": decision1,
                "trt_batch9_decision": decision9,
            }
        )

    gate_reasons = []
    maximum_reference_difference = max(ref_differences)
    maximum_batch_difference = max(batch_differences)
    if maximum_reference_difference > backend_epsilon:
        gate_reasons.append("numeric parity exceeded frozen epsilon")
    if maximum_batch_difference > backend_epsilon:
        gate_reasons.append("batch-1/batch-9 parity exceeded frozen epsilon")
    if batch1_mismatches:
        gate_reasons.append("batch-1 decision mismatch")
    if batch9_mismatches:
        gate_reasons.append("batch-9 decision mismatch")
    if cross_errors:
        gate_reasons.append("ground-truth cross error")
    if decisions["UNKNOWN"]:
        gate_reasons.append("unexpected validation UNKNOWN")
    gate_passed = not gate_reasons

    report = {
        "schema_version": 1,
        "status": "TENSORRT_PARITY_PASSED" if gate_passed else "TENSORRT_PARITY_FAILED",
        "gate_passed": gate_passed,
        "gate_reasons": gate_reasons,
        "run_id": thresholds_doc.get("run_id", "20260801T060414Z_c37b859a"),
        "scope": "validation_original_only; held-out test not opened",
        "reference": "frozen Colab ONNX validation_predictions.csv",
        "engine": {
            "path": str(args.engine.resolve()),
            "sha256": sha256_file(args.engine),
            "bytes": args.engine.stat().st_size,
            "tensorrt_version": trt.__version__,
            "precision_build_flag": "FP16 enabled with FP32 input/output",
            "profile": {
                "minimum": [1, 3, 224, 224],
                "optimum": [9, 3, 224, 224],
                "maximum": [9, 3, 224, 224],
            },
        },
        "frozen_thresholds": {
            "t_empty": t_empty,
            "t_occupied": t_occupied,
            "backend_epsilon": backend_epsilon,
        },
        "source_integrity": source_integrity,
        "validation": {
            "samples": len(reference),
            "captures": len({row["capture_id"] for row in reference}),
            "labels": dict(Counter(row["label"] for row in reference)),
            "decisions_batch9": dict(decisions),
            "cross_errors_batch9": dict(cross_errors),
            "dangerous_occupied_to_empty_batch9": cross_errors["OCCUPIED_to_EMPTY"],
            "decision_mismatches_batch1_vs_onnx": batch1_mismatches,
            "decision_mismatches_batch9_vs_onnx": batch9_mismatches,
            "max_abs_difference_batch9_vs_onnx": maximum_reference_difference,
            "mean_abs_difference_batch9_vs_onnx": statistics.fmean(ref_differences),
            "max_abs_difference_batch9_vs_batch1": maximum_batch_difference,
        },
        "timing": {
            "definition": "CUDA H2D + TensorRT execution + D2H + stream sync",
            "batch1_runs": len(batch1_timings_ms),
            "batch1_mean_ms": statistics.fmean(batch1_timings_ms),
            "batch1_p95_ms": percentile(batch1_timings_ms, 0.95),
            "batch9_runs": len(batch9_timings_ms),
            "batch9_mean_ms": statistics.fmean(batch9_timings_ms),
            "batch9_p95_ms": percentile(batch9_timings_ms, 0.95),
            "batch9_mean_ms_per_roi": statistics.fmean(batch9_timings_ms) / 9.0,
        },
        "software": {
            "python": os.sys.version.split()[0],
            "numpy": np.__version__,
            "pillow": Image.__version__,
        },
    }
    write_csv_new(args.predictions_csv, prediction_rows)
    write_json_new(args.report_json, report)
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    if not gate_passed:
        raise RuntimeError("TensorRT parity gate failed: " + "; ".join(gate_reasons))
    print("PASS: TENSORRT_PARITY_PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
