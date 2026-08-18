#!/usr/bin/env python3
"""Static fail-closed audit for the released rack dataset v2 Colab notebook."""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
from pathlib import Path

import percent_script_to_notebook as converter


EXPECTED_ZIP_BYTES = 90_100_180
EXPECTED_ZIP_SHA256 = (
    "da90b2e5ed976b918f326c8332a36f09bdbe023d7d7bd5e2d08e8c629193ed09"
)
EXPECTED_RELEASE_NAME = "rack-occupancy-mobilenetv3-small-colab-v2"
EXPECTED_SOURCE_NAME = "02_train_mobilenetv3_small_colab_v2.py"
EXPECTED_NOTEBOOK_NAME = "02_train_mobilenetv3_small_colab_v2.ipynb"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--notebook", required=True, type=Path)
    parser.add_argument("--release", required=True, type=Path)
    parser.add_argument("--dataset-zip", required=True, type=Path)
    return parser.parse_args()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def semantic_sha256(notebook: dict) -> str:
    semantic_cells = [
        {
            "cell_type": cell.get("cell_type"),
            "source": (
                "".join(cell.get("source", []))
                if isinstance(cell.get("source", ""), list)
                else cell.get("source", "")
            ),
        }
        for cell in notebook.get("cells", [])
    ]
    canonical = json.dumps(
        semantic_cells,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()


def require(text: str, fragment: str) -> None:
    if fragment not in text:
        raise ValueError(f"Missing required fragment: {fragment}")


def main() -> int:
    args = parse_args()
    source_text = args.source.read_text(encoding="utf-8")
    ast.parse(source_text, filename=str(args.source))
    notebook = json.loads(args.notebook.read_text(encoding="utf-8"))
    release = json.loads(args.release.read_text(encoding="utf-8"))

    if args.source.name != EXPECTED_SOURCE_NAME:
        raise ValueError("Unexpected canonical source filename")
    if args.notebook.name != EXPECTED_NOTEBOOK_NAME:
        raise ValueError("Unexpected notebook filename")
    if args.dataset_zip.stat().st_size != EXPECTED_ZIP_BYTES:
        raise ValueError("Dataset ZIP byte size mismatch")
    if sha256_file(args.dataset_zip) != EXPECTED_ZIP_SHA256:
        raise ValueError("Dataset ZIP SHA-256 mismatch")

    cells = notebook.get("cells")
    if notebook.get("nbformat") != 4 or not isinstance(cells, list):
        raise ValueError("Notebook must use nbformat 4")
    expected_cells = converter.parse_cells(source_text)
    if cells != expected_cells:
        raise ValueError("Notebook cells are not an exact percent-source conversion")
    code_cells = [cell for cell in cells if cell.get("cell_type") == "code"]
    markdown_cells = [cell for cell in cells if cell.get("cell_type") == "markdown"]
    if (len(cells), len(code_cells), len(markdown_cells)) != (15, 13, 2):
        raise ValueError("Unexpected notebook cell topology")
    for index, cell in enumerate(code_cells):
        ast.parse(cell["source"], filename=f"cell_{index:02d}")
        if cell.get("execution_count") is not None or cell.get("outputs"):
            raise ValueError(f"Code cell {index} contains execution state")

    joined = "\n".join(str(cell.get("source", "")) for cell in cells)
    forbidden = (
        "import cv2",
        "from cv2",
        "random_split",
        "RandomCrop",
        "RandomHorizontalFlip",
        "RandomVerticalFlip",
        "RandomRotation",
        "ColorJitter",
        "rack_roi_dataset_v1_colab.zip",
        'prepared" / "rack_roi_dataset_v1',
        '"audit_prepared_dataset.py"',
        "s01/L00",
        "20260801T034050Z_e0939e7c",
        "a6271d1bf9954242540ed01b02ce39e7234016c6dac8e683764d60683075c4eb",
        "e86680610423f9817d1b7509cd451db1b4316c9a89083b4212eb08cee142aa74",
        "(1800, 90)",
    )
    for fragment in forbidden:
        if fragment in joined:
            raise ValueError(f"Forbidden v1/stale notebook behavior: {fragment}")

    required = (
        EXPECTED_ZIP_SHA256,
        "EXPECTED_ZIP_BYTES = 90_100_180",
        'DATASET_ID = "rack_roi_dataset_v2"',
        'ZIP_PATH = DRIVE_ROOT / "rack_roi_dataset_v2_colab.zip"',
        '"audit_prepared_dataset_v2.py"',
        '"--config"',
        '"dataset_v2.json"',
        'DATA_ROOT = EXTRACT_ROOT / "prepared" / "rack_roi_dataset_v2"',
        "(2250, 90)",
        '{"EMPTY": 1090, "OCCUPIED": 1160}',
        '{"EMPTY": 45, "OCCUPIED": 45}',
        'range(1, 51)',
        'range(71, 81)',
        'range(81, 91)',
        '"dataset_summary_sha256"',
        '"input_sha256_manifest_sha256"',
        '"dataset_config_sha256"',
        '"label_corrections_sha256"',
        '"split_consumption_sha256"',
        "EXPECTED_RELEASE_NAME",
        "Notebook release dataset ZIP SHA-256 mismatch",
        "weights_only=True",
        "SpatialMeanPool",
        "smoke_loss.backward()",
        "warn_only=False",
        "InterpolationMode.BILINEAR",
        'source_color_space": "RGB',
        "dynamo=True",
        "opset_version=17",
        'output_names=["p_occupied"]',
        "BACKEND_EPSILON = max(0.02",
        "retuning_from_test_forbidden",
        "TEST_SET_FINGERPRINT",
        "raw_test_source_frame_sha256",
        "SOURCE_FRAME_LOCK_PATHS",
        'TEST_LOCK_PATH.open("x"',
        "assert_valid_probabilities",
        "CLUSTERED_SAMPLE_LIMITATION",
        "test_evaluation_complete.json",
        "verified_test_gate_passed",
        "deployment_authorized\": False",
        "PASS: validation safety gate",
        "PASS: ONNX_BASELINE_READY",
        "s05 fresh holdout captured only at L00",
    )
    for fragment in required:
        require(joined, fragment)

    code_freeze = joined.index("PASS: exact training notebook code frozen")
    training = joined.index("weights = MobileNet_V3_Small_Weights.DEFAULT")
    validation_artifact_copy = joined.index("for file_name in validation_artifacts:")
    threshold_freeze = joined.index("thresholds_sha256 = sha256_file")
    test_lock = joined.index('TEST_LOCK_PATH.open("x"')
    test_start_record = joined.index("test_evaluation_started.json")
    test_open = joined.index("test_dataset = ImageFolderWithPath")
    test_prediction = joined.index("test_predictions = collect_onnx_predictions")
    test_completion = joined.index("test_completion_payload =")
    publication = joined.index("deployment_metadata_payload =")
    if not code_freeze < training:
        raise ValueError("Training begins before source/notebook release is frozen")
    if not (
        validation_artifact_copy
        < threshold_freeze
        < test_lock
        < test_start_record
        < test_open
        < test_prediction
    ):
        raise ValueError("Fresh test is accessed before threshold freeze and raw-frame locks")
    if not test_prediction < test_completion < publication:
        raise ValueError("Publication can occur before test completion is sealed")

    if release.get("schema_version") != 1:
        raise ValueError("Unsupported release schema")
    if release.get("release_name") != EXPECTED_RELEASE_NAME:
        raise ValueError("Release name mismatch")
    if release.get("dataset_zip_bytes") != EXPECTED_ZIP_BYTES:
        raise ValueError("Release ZIP byte size mismatch")
    if release.get("dataset_zip_sha256") != EXPECTED_ZIP_SHA256:
        raise ValueError("Release ZIP SHA-256 mismatch")
    source_release = release.get("files", {}).get("canonical_source", {})
    notebook_release = release.get("files", {}).get("colab_notebook", {})
    if (
        source_release.get("name") != EXPECTED_SOURCE_NAME
        or source_release.get("sha256") != sha256_file(args.source)
    ):
        raise ValueError("Canonical source release fingerprint mismatch")
    if (
        notebook_release.get("name") != EXPECTED_NOTEBOOK_NAME
        or notebook_release.get("initial_file_sha256") != sha256_file(args.notebook)
        or notebook_release.get("semantic_sha256") != semantic_sha256(notebook)
    ):
        raise ValueError("Notebook release fingerprint mismatch")

    print("PASS exact v2 ZIP size/SHA pin")
    print("PASS exact percent-source/notebook cell equivalence and syntax")
    print("PASS exact v2 split IDs, sizes, class counts, and provenance hashes")
    print("PASS validation-only selection and raw-frame locks before fresh-test inference")
    print("PASS release name/source/notebook/semantic fingerprints")
    print("PASS no stale v1 paths, IDs, report hashes, or s01 limitation text")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, json.JSONDecodeError, SyntaxError) as exc:
        raise SystemExit(f"FAIL: {exc}")
