#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
RUNTIME="$SCRIPT_DIR/runtime"
ENGINE="$RUNTIME/deployment/jetson/20260801T060414Z_c37b859a/engine/rack_occupancy_mobilenetv3_small_fp16_trt10.3.plan"
CAMERA_CONFIG="$RUNTIME/config/camera_c270_live.json"
status=0

pass_item() { printf '[PASS] %s\n' "$1"; }
warn_item() { printf '[WARN] %s\n' "$1"; }
fail_item() { printf '[FAIL] %s\n' "$1"; status=1; }

printf 'Jetson rack project preflight\nroot: %s\n\n' "$SCRIPT_DIR"

for path in \
  "$RUNTIME/tools/deployment/live_rack_monitor.py" \
  "$RUNTIME/tools/communication/robot_dispatch_controller.py" \
  "$RUNTIME/tools/communication/zybo_target_receiver_8byte.py" \
  "$ENGINE" \
  "$CAMERA_CONFIG" \
  "$RUNTIME/openrb/final_openrb_20260810.ino"; do
  if [ -e "$path" ]; then
    pass_item "$(realpath --relative-to="$SCRIPT_DIR" "$path")"
  else
    fail_item "missing: $(realpath --relative-to="$SCRIPT_DIR" "$path")"
  fi
done

if command -v python3 >/dev/null 2>&1; then
  pass_item "python3: $(python3 --version 2>&1)"
else
  fail_item "python3 not found"
fi

for module in serial numpy PIL; do
  if python3 -c "import $module" >/dev/null 2>&1; then
    pass_item "python module: $module"
  else
    warn_item "python module missing: $module"
  fi
done

if python3 -c 'import tensorrt' >/dev/null 2>&1; then
  pass_item "python module: tensorrt"
else
  warn_item "python module missing: tensorrt (required on the Jetson runtime)"
fi

if command -v ffmpeg >/dev/null 2>&1; then
  pass_item "ffmpeg: $(command -v ffmpeg)"
else
  warn_item "ffmpeg not found (required by the live camera monitor)"
fi

if [ -e /dev/video0 ]; then
  pass_item "/dev/video0 present"
else
  warn_item "/dev/video0 not present; connect Logitech C270 before live run"
fi

if compgen -G '/dev/ttyUSB*' >/dev/null 2>&1; then
  printf '[PASS] UART devices:'
  printf ' %s' /dev/ttyUSB*
  printf '\n'
else
  warn_item 'no /dev/ttyUSB*; connect FPGA/OpenRB USB-TTL adapters before integrated run'
fi

if [ "$status" -eq 0 ]; then
  printf '\nPREFLIGHT PASS (hardware warnings, if any, are non-fatal)\n'
else
  printf '\nPREFLIGHT FAIL: required release files are missing\n'
fi
exit "$status"
