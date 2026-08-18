#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$SCRIPT_DIR/runtime/tools/communication:$SCRIPT_DIR/runtime/tools/deployment${PYTHONPATH:+:$PYTHONPATH}"

printf '%s\n' 'Checking Python syntax...'
python3 - "$SCRIPT_DIR" <<'PY'
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
files = list((root / "runtime/tools/communication").glob("*.py"))
files += list((root / "runtime/tools/deployment").glob("*.py"))
for path in files:
    compile(path.read_text(encoding="utf-8"), str(path), "exec")
    print(f"[PASS] syntax {path.relative_to(root)}")
PY

printf '%s\n' 'Running communication tests...'
python3 -m unittest discover -s "$SCRIPT_DIR/tests/communication" -p 'test_*.py' -v

printf '%s\n' 'Running deployment safety tests...'
python3 -m unittest discover -s "$SCRIPT_DIR/tests/deployment" -p 'test_*.py' -v

printf '%s\n' 'VERIFY PASS'
