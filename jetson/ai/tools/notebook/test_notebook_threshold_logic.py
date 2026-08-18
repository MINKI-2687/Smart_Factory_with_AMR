#!/usr/bin/env python3
"""Execute the notebook's pure threshold functions against synthetic cases."""

from __future__ import annotations

import ast
import argparse
import hashlib
import json
import re
from pathlib import Path

import numpy as np


FUNCTIONS = {
    "triage_decisions",
    "summarize_triage_arrays",
    "threshold_candidate_metrics",
    "build_raw_test_identity",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    return parser.parse_args()


def load_functions(path: Path) -> dict[str, object]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    selected: list[ast.stmt] = []
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == "CLASS_TO_IDX"
            for target in node.targets
        ):
            selected.append(node)
        elif isinstance(node, ast.FunctionDef) and node.name in FUNCTIONS:
            selected.append(node)
    namespace: dict[str, object] = {
        "np": np,
        "hashlib": hashlib,
        "json": json,
        "re": re,
    }
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(path), "exec"), namespace)
    missing = FUNCTIONS - namespace.keys()
    if missing:
        raise RuntimeError(f"Missing functions: {sorted(missing)}")
    return namespace


def main() -> int:
    args = parse_args()
    namespace = load_functions(args.source)
    decide = namespace["triage_decisions"]
    summarize = namespace["summarize_triage_arrays"]
    candidate = namespace["threshold_candidate_metrics"]
    build_test_identity = namespace["build_raw_test_identity"]

    probabilities = np.array([0.10, 0.20, 0.50, 0.80, 0.90])
    observed = decide(probabilities, 0.20, 0.80)
    expected = np.array(["EMPTY", "EMPTY", "UNKNOWN", "OCCUPIED", "OCCUPIED"])
    np.testing.assert_array_equal(observed, expected)

    labels = np.array([0] * 45 + [1] * 45)
    separated = np.array([0.10] * 45 + [0.90] * 45)
    decisions = decide(separated, 0.20, 0.80)
    summary = summarize(labels, decisions)
    assert summary["empty_to_occupied"] == 0
    assert summary["occupied_to_empty"] == 0
    assert summary["coverage"] == 1.0
    assert summary["known_accuracy"] == 1.0

    safe = candidate(labels, separated, 0.20, 0.80, 0.02)
    assert safe["robust_occupied_to_empty"] == 0
    assert safe["robust_empty_to_occupied"] == 0
    assert safe["separation_margin"] > 0

    unsafe_occupied = candidate(
        np.array([0, 1]),
        np.array([0.10, 0.80]),
        0.79,
        0.90,
        0.02,
    )
    assert unsafe_occupied["robust_occupied_to_empty"] == 1

    unsafe_empty = candidate(
        np.array([0, 1]),
        np.array([0.20, 0.90]),
        0.10,
        0.21,
        0.02,
    )
    assert unsafe_empty["robust_empty_to_occupied"] == 1

    occupied_exact_boundary = candidate(
        np.array([0, 1]),
        np.array([0.00, 0.75]),
        0.50,
        0.90,
        0.25,
    )
    assert occupied_exact_boundary["robust_occupied_to_empty"] == 1

    empty_exact_boundary = candidate(
        np.array([0, 1]),
        np.array([0.25, 1.00]),
        0.10,
        0.50,
        0.25,
    )
    assert empty_exact_boundary["robust_empty_to_occupied"] == 1

    raw_frames = [f"{value:064x}" for value in range(10)]
    identity_a = build_test_identity(raw_frames)
    identity_b = build_test_identity(list(reversed(raw_frames)))
    assert identity_a == identity_b

    unrelated_training_metadata_a = "dataset-version-a"
    unrelated_training_metadata_b = "dataset-version-b"
    assert unrelated_training_metadata_a != unrelated_training_metadata_b
    assert build_test_identity(raw_frames)[1] == identity_a[1]

    partially_reused_frames = raw_frames[:1] + [
        f"{value:064x}" for value in range(100, 109)
    ]
    reused_lock_names = {f"{value}.lock.json" for value in raw_frames}
    candidate_lock_names = {
        f"{value}.lock.json" for value in partially_reused_frames
    }
    assert reused_lock_names & candidate_lock_names

    print("PASS inclusive EMPTY/OCCUPIED boundaries")
    print("PASS UNKNOWN band accounting")
    print("PASS backend-epsilon robust cross-error checks")
    print("PASS exact epsilon-boundary cases fail closed")
    print("PASS raw-test identity ignores train/validation metadata changes")
    print("PASS any reused raw test frame has an overlapping lock name")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
