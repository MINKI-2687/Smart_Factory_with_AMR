#!/usr/bin/env python3
"""Create a deterministic, self-contained Colab ZIP for rack_roi_dataset_v2."""

from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path, PurePosixPath
import stat
import subprocess
import sys
import tempfile
import zipfile

import build_dataset_v2 as v2_build


FIXED_ZIP_TIME = (2026, 8, 1, 0, 0, 0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def add_file(files: set[Path], root: Path, path: Path) -> None:
    resolved = path.resolve()
    if not resolved.is_relative_to(root):
        raise ValueError(f"Package path escapes dataset root: {path}")
    if resolved.is_symlink() or not resolved.is_file():
        raise ValueError(f"Package input is not a regular file: {resolved}")
    files.add(resolved)


def collect_files(root: Path, config_path: Path, config: dict) -> list[Path]:
    files: set[Path] = set()
    for path in (
        config_path,
        v2_build.rooted_path(root, config["roi_config"]),
        v2_build.rooted_path(root, config["source_manifest_before_correction"]),
        v2_build.rooted_path(root, config["label_correction_ledger"]),
        v2_build.rooted_path(root, config["split_consumption_ledger"]),
        v2_build.rooted_path(root, config["test_capture_plan"]),
        v2_build.rooted_path(root, "tools/dataset/build_dataset.py"),
        v2_build.rooted_path(root, "tools/dataset/build_dataset_v2.py"),
        v2_build.rooted_path(root, "tools/dataset/audit_prepared_dataset_v2.py"),
    ):
        add_file(files, root, path)

    manifest_paths = [
        v2_build.rooted_path(root, config["split_manifests"][split])
        for split in v2_build.SPLITS
    ]
    for path in manifest_paths:
        add_file(files, root, path)
    for relative in config.get("capture_provenance", []):
        add_file(files, root, v2_build.rooted_path(root, relative))

    for row in read_csv(v2_build.rooted_path(root, config["label_correction_ledger"])):
        add_file(files, root, v2_build.rooted_path(root, row["evidence_relpath"]))
    for manifest_path in manifest_paths:
        for row in read_csv(manifest_path):
            add_file(files, root, v2_build.rooted_path(root, row["source_relpath"]))

    prepared = root / "prepared" / config["dataset_id"]
    if not prepared.is_dir():
        raise ValueError(f"Prepared dataset missing: {prepared}")
    for path in prepared.rglob("*"):
        if path.is_symlink():
            raise ValueError(f"Symlink is forbidden in prepared dataset: {path}")
        if path.is_file():
            add_file(files, root, path)

    relative_paths = [path.relative_to(root).as_posix() for path in files]
    if len(relative_paths) != len(set(relative_paths)):
        raise ValueError("Duplicate archive member path")
    return sorted(files, key=lambda path: path.relative_to(root).as_posix())


def validate_zip(path: Path, expected_names: list[str]) -> None:
    with zipfile.ZipFile(path) as archive:
        infos = archive.infolist()
        names = [info.filename for info in infos]
        if names != expected_names or len(names) != len(set(names)):
            raise ValueError("ZIP member list/order differs from deterministic input list")
        for info in infos:
            member = PurePosixPath(info.filename)
            file_type = (info.external_attr >> 16) & 0o170000
            if member.is_absolute() or ".." in member.parts:
                raise ValueError(f"Unsafe ZIP path: {info.filename}")
            if file_type == stat.S_IFLNK:
                raise ValueError(f"ZIP symlink is forbidden: {info.filename}")
        bad_member = archive.testzip()
        if bad_member is not None:
            raise ValueError(f"Corrupt ZIP member: {bad_member}")


def main() -> int:
    args = parse_args()
    root = args.dataset_root.resolve()
    config_path = args.config.resolve()
    output = args.output.resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    v2_build.validate_config(config)
    if output.exists():
        raise ValueError(f"Refusing to overwrite ZIP: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)

    prepared = root / "prepared" / config["dataset_id"]
    subprocess.run(
        [
            sys.executable,
            "-B",
            "-u",
            str(root / "tools/dataset/audit_prepared_dataset_v2.py"),
            "--dataset-root",
            str(root),
            "--prepared-root",
            str(prepared),
            "--config",
            str(config_path),
        ],
        check=True,
    )

    files = collect_files(root, config_path, config)
    expected_names = [path.relative_to(root).as_posix() for path in files]
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output.name}.tmp-", dir=output.parent
    )
    os.close(descriptor)
    Path(temporary_name).unlink()
    try:
        with zipfile.ZipFile(
            temporary_name,
            mode="x",
            compression=zipfile.ZIP_DEFLATED,
            compresslevel=9,
            allowZip64=True,
        ) as archive:
            for source, member_name in zip(files, expected_names):
                info = zipfile.ZipInfo(member_name, date_time=FIXED_ZIP_TIME)
                info.compress_type = zipfile.ZIP_DEFLATED
                info.external_attr = (stat.S_IFREG | 0o644) << 16
                archive.writestr(info, source.read_bytes(), compresslevel=9)
        validate_zip(Path(temporary_name), expected_names)
        Path(temporary_name).rename(output)
    finally:
        try:
            Path(temporary_name).unlink()
        except FileNotFoundError:
            pass

    print(f"PASS deterministic ZIP members: {len(expected_names)}")
    print(f"PASS ZIP CRC/path/symlink checks: {output}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, json.JSONDecodeError, subprocess.CalledProcessError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
