# #!/usr/bin/env python3
# """Live 3x3 rack monitor with TensorRT inference and browser overlay.

# Camera input is captured by FFmpeg/V4L2. Pillow draws the overlay and serves
# MJPEG over HTTP. OpenCV is intentionally not used. This monitor never opens a
# UART device and never sends an STM32 command.
# """

# from __future__ import annotations

# import argparse
# from collections import deque
# from dataclasses import dataclass
# from http import HTTPStatus
# from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
# from io import BytesIO
# import json
# from pathlib import Path
# import queue
# import subprocess
# import threading
# import time
# from typing import Any, BinaryIO

# import numpy as np
# from PIL import Image, ImageDraw, ImageEnhance, ImageFont

# from check_rack_alignment import (
#     measure_alignment_masks,
#     rack_mask,
#     rack_mask_rgb,
# )
# from infer_rack_frame_trt import SLOT_ORDER, validate_rois
# from rack_safety_gate import temporal_consensus
# from validate_tensorrt_parity import (
#     TensorRTRunner,
#     decide,
#     preprocess_pil_image,
#     require,
#     sha256_file,
# )


# PROJECT_ROOT = Path(__file__).resolve().parents[2]
# RUN_ID = "20260801T060414Z_c37b859a"
# RUN_ROOT = PROJECT_ROOT / "deployment" / "jetson" / RUN_ID
# FRAME_WIDTH = 1280
# FRAME_HEIGHT = 720
# FRAME_BYTES = FRAME_WIDTH * FRAME_HEIGHT * 3
# STATE_COLORS = {
#     "EMPTY": (32, 220, 90),
#     "OCCUPIED": (245, 55, 55),
#     "UNKNOWN": (255, 205, 35),
# }


# def parse_args() -> argparse.Namespace:
#     parser = argparse.ArgumentParser()
#     parser.add_argument("--device", default="/dev/video0")
#     parser.add_argument(
#         "--camera-config",
#         type=Path,
#         default=PROJECT_ROOT / "config" / "camera_c270_live.json",
#     )
#     parser.add_argument(
#         "--skip-camera-config",
#         action="store_true",
#         help="Do not apply configured V4L2 controls before opening FFmpeg.",
#     )
#     parser.add_argument(
#         "--engine",
#         type=Path,
#         default=RUN_ROOT / "engine" / "rack_occupancy_mobilenetv3_small_fp16_trt10.3.plan",
#     )
#     parser.add_argument(
#         "--deployment-metadata",
#         type=Path,
#         default=RUN_ROOT / "source" / "deployment_metadata.json",
#     )
#     parser.add_argument(
#         "--thresholds",
#         type=Path,
#         default=RUN_ROOT / "source" / "thresholds.json",
#     )
#     parser.add_argument(
#         "--alignment-reference",
#         type=Path,
#         default=PROJECT_ROOT / "raw" / "s01_train" / "c270_calv1_s01_l00_a001.jpg",
#     )
#     parser.add_argument("--host", default="0.0.0.0")
#     parser.add_argument("--port", default=8080, type=int)
#     parser.add_argument("--display-fps", default=10.0, type=float)
#     parser.add_argument(
#         "--preview-brightness",
#         default=0.82,
#         type=float,
#         help="Display-only brightness factor; TensorRT always receives the unmodified frame.",
#     )
#     parser.add_argument("--alignment-interval", default=2.0, type=float)
#     parser.add_argument("--max-allowed-shift", default=8, type=int)
#     parser.add_argument("--minimum-alignment-score", default=0.35, type=float)
#     parser.add_argument(
#         "--max-frames",
#         default=0,
#         type=int,
#         help="Exit after this many processed frames; 0 runs until Ctrl+C.",
#     )
#     return parser.parse_args()


# def read_exact(stream: BinaryIO, size: int) -> bytes:
#     output = bytearray(size)
#     view = memoryview(output)
#     position = 0
#     while position < size:
#         count = stream.readinto(view[position:])
#         if not count:
#             raise EOFError(f"FFmpeg ended after {position}/{size} frame bytes")
#         position += count
#     return bytes(output)


# def load_font(size: int, *, bold: bool = False) -> ImageFont.ImageFont:
#     name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
#     path = Path("/usr/share/fonts/truetype/dejavu") / name
#     try:
#         return ImageFont.truetype(str(path), size=size)
#     except OSError:
#         return ImageFont.load_default()


# def draw_text_box(
#     draw: ImageDraw.ImageDraw,
#     position: tuple[int, int],
#     text: str,
#     *,
#     font: ImageFont.ImageFont,
#     foreground: tuple[int, int, int] = (255, 255, 255),
#     background: tuple[int, int, int] = (0, 0, 0),
# ) -> None:
#     x, y = position
#     try:
#         left, top, right, bottom = draw.textbbox((x, y), text, font=font)
#         width, height = right - left, bottom - top
#     except AttributeError:
#         width, height = draw.textsize(text, font=font)
#     padding = 4
#     draw.rectangle(
#         (x - padding, y - padding, x + width + padding, y + height + padding),
#         fill=background,
#     )
#     draw.text((x, y), text, font=font, fill=foreground)


# class FrameStore:
#     def __init__(self) -> None:
#         self.condition = threading.Condition()
#         self.sequence = 0
#         self.jpeg: bytes | None = None
#         self.status: dict[str, Any] = {"status": "STARTING"}

#     def publish(self, jpeg: bytes, status: dict[str, Any]) -> None:
#         with self.condition:
#             self.sequence += 1
#             self.jpeg = jpeg
#             self.status = status
#             self.condition.notify_all()

#     def snapshot(self) -> tuple[int, bytes | None, dict[str, Any]]:
#         with self.condition:
#             return self.sequence, self.jpeg, dict(self.status)

#     def wait_after(
#         self, sequence: int, timeout: float = 5.0
#     ) -> tuple[int, bytes | None, dict[str, Any]]:
#         with self.condition:
#             self.condition.wait_for(lambda: self.sequence > sequence, timeout=timeout)
#             return self.sequence, self.jpeg, dict(self.status)


# class MonitorHTTPServer(ThreadingHTTPServer):
#     daemon_threads = True
#     allow_reuse_address = True

#     def __init__(self, address: tuple[str, int], store: FrameStore) -> None:
#         self.frame_store = store
#         super().__init__(address, MonitorHandler)


# class MonitorHandler(BaseHTTPRequestHandler):
#     server: MonitorHTTPServer

#     def log_message(self, format: str, *args: object) -> None:
#         return

#     def do_GET(self) -> None:
#         if self.path in {"/", "/index.html"}:
#             self._serve_index()
#         elif self.path == "/stream.mjpg":
#             self._serve_stream()
#         elif self.path == "/snapshot.jpg":
#             self._serve_snapshot()
#         elif self.path == "/status.json":
#             self._serve_status()
#         else:
#             self.send_error(HTTPStatus.NOT_FOUND)

#     def _serve_index(self) -> None:
#         body = """<!doctype html>
# <html lang="ko"><head><meta charset="utf-8">
# <meta name="viewport" content="width=device-width,initial-scale=1">
# <title>3x3 Rack Occupancy</title>
# <style>
# body{margin:0;background:#111;color:#eee;font-family:sans-serif;text-align:center}
# h1{font-size:22px;margin:12px}.summary{box-sizing:border-box;width:min(96vw,1280px);
# margin:0 auto 8px;padding:10px 14px;background:#222;border-left:8px solid #888;
# font-size:17px;font-weight:700;text-align:left}.stable{border-color:#20dc5a}.warming{border-color:#ffcd23}
# .fault{border-color:#f53737}img{width:min(96vw,1280px);height:auto;border:1px solid #444}
# details{width:min(94vw,1240px);margin:10px auto;text-align:left;color:#aaa}
# pre{white-space:pre-wrap;color:#bbb}
# </style></head><body><h1>3x3 Rack Occupancy — TensorRT Live</h1>
# <div id="summary" class="summary">STARTING</div>
# <img src="/stream.mjpg" alt="live rack stream">
# <details><summary>상세 상태 JSON</summary><pre id="status">STARTING</pre></details>
# <script>setInterval(async()=>{try{let r=await fetch('/status.json',{cache:'no-store'});let s=await r.json();
# let p=s.pose||{};let ms=s.inference_ms==null?'blocked':s.inference_ms.toFixed(1)+'ms';
# let summary=document.getElementById('summary');summary.textContent=`${s.status} | pose=(${p.shift_x??'-'},${p.shift_y??'-'}) score=${(p.score??0).toFixed(3)} | TensorRT=${ms} | ${s.display_fps.toFixed(1)} FPS | STM32=0 bytes`;
# summary.className='summary '+(s.status==='STABLE'?'stable':s.status==='CAMERA_FAULT'?'fault':'warming');
# document.getElementById('status').textContent=JSON.stringify(s,null,2)}catch(e){}},500)</script>
# </body></html>""".encode("utf-8")
#         self.send_response(HTTPStatus.OK)
#         self.send_header("Content-Type", "text/html; charset=utf-8")
#         self.send_header("Content-Length", str(len(body)))
#         self.end_headers()
#         self.wfile.write(body)

#     def _serve_snapshot(self) -> None:
#         _, jpeg, _ = self.server.frame_store.snapshot()
#         if jpeg is None:
#             self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "No frame yet")
#             return
#         self.send_response(HTTPStatus.OK)
#         self.send_header("Content-Type", "image/jpeg")
#         self.send_header("Content-Length", str(len(jpeg)))
#         self.send_header("Cache-Control", "no-store")
#         self.end_headers()
#         self.wfile.write(jpeg)

#     def _serve_status(self) -> None:
#         _, _, status = self.server.frame_store.snapshot()
#         body = json.dumps(status, ensure_ascii=False, sort_keys=True).encode("utf-8")
#         self.send_response(HTTPStatus.OK)
#         self.send_header("Content-Type", "application/json; charset=utf-8")
#         self.send_header("Content-Length", str(len(body)))
#         self.send_header("Cache-Control", "no-store")
#         self.end_headers()
#         self.wfile.write(body)

#     def _serve_stream(self) -> None:
#         self.send_response(HTTPStatus.OK)
#         self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
#         self.send_header("Cache-Control", "no-store")
#         self.end_headers()
#         sequence = -1
#         try:
#             while True:
#                 sequence, jpeg, _ = self.server.frame_store.wait_after(sequence)
#                 if jpeg is None:
#                     continue
#                 self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
#                 self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
#                 self.wfile.write(jpeg)
#                 self.wfile.write(b"\r\n")
#         except (BrokenPipeError, ConnectionResetError):
#             return


# @dataclass
# class PoseResult:
#     score: float
#     shift_x: int
#     shift_y: int
#     aligned: bool
#     updated_monotonic: float


# class PoseGuard:
#     def __init__(
#         self,
#         reference: Path,
#         *,
#         minimum_score: float,
#         max_allowed_shift: int,
#         search_radius: int = 120,
#         downsample: int = 4,
#     ) -> None:
#         self.minimum_score = minimum_score
#         self.max_allowed_shift = max_allowed_shift
#         self.search_radius = search_radius
#         self.downsample = downsample
#         self.reference_mask = rack_mask(reference, downsample)
#         self.queue: queue.Queue[np.ndarray | None] = queue.Queue(maxsize=1)
#         self.lock = threading.Lock()
#         self.result: PoseResult | None = None
#         self.thread = threading.Thread(target=self._worker, name="pose-guard", daemon=True)

#     def start(self) -> None:
#         self.thread.start()

#     def stop(self) -> None:
#         try:
#             self.queue.put_nowait(None)
#         except queue.Full:
#             try:
#                 self.queue.get_nowait()
#             except queue.Empty:
#                 pass
#             self.queue.put_nowait(None)
#         self.thread.join(timeout=3.0)

#     def submit(self, rgb: np.ndarray) -> None:
#         try:
#             self.queue.put_nowait(rgb.copy())
#         except queue.Full:
#             try:
#                 self.queue.get_nowait()
#             except queue.Empty:
#                 pass
#             try:
#                 self.queue.put_nowait(rgb.copy())
#             except queue.Full:
#                 pass

#     def current(self) -> PoseResult | None:
#         with self.lock:
#             return self.result

#     def check_now(self, rgb: np.ndarray) -> PoseResult:
#         result = self._measure(rgb)
#         with self.lock:
#             self.result = result
#         return result

#     def _measure(self, rgb: np.ndarray) -> PoseResult:
#         current_mask = rack_mask_rgb(rgb, self.downsample)
#         score, shift_x, shift_y = measure_alignment_masks(
#             self.reference_mask,
#             current_mask,
#             downsample=self.downsample,
#             search_radius=self.search_radius,
#         )
#         aligned = (
#             score >= self.minimum_score
#             and abs(shift_x) <= self.max_allowed_shift
#             and abs(shift_y) <= self.max_allowed_shift
#         )
#         return PoseResult(score, shift_x, shift_y, aligned, time.monotonic())

#     def _worker(self) -> None:
#         while True:
#             rgb = self.queue.get()
#             if rgb is None:
#                 return
#             try:
#                 result = self._measure(rgb)
#             except Exception:
#                 result = PoseResult(0.0, 0, 0, False, time.monotonic())
#             with self.lock:
#                 self.result = result


# def start_ffmpeg(args: argparse.Namespace) -> subprocess.Popen[bytes]:
#     command = [
#         "ffmpeg",
#         "-hide_banner",
#         "-loglevel",
#         "warning",
#         "-f",
#         "v4l2",
#         "-input_format",
#         "mjpeg",
#         "-video_size",
#         f"{FRAME_WIDTH}x{FRAME_HEIGHT}",
#         "-framerate",
#         "30",
#         "-i",
#         args.device,
#         "-vf",
#         f"fps={args.display_fps:g}",
#         "-pix_fmt",
#         "rgb24",
#         "-f",
#         "rawvideo",
#         "pipe:1",
#     ]
#     return subprocess.Popen(command, stdout=subprocess.PIPE, bufsize=FRAME_BYTES * 2)


# def apply_camera_controls(args: argparse.Namespace) -> str:
#     if args.skip_camera_config:
#         return "SKIPPED by --skip-camera-config"
#     require(args.camera_config.is_file(), f"Camera config is missing: {args.camera_config}")
#     config = json.loads(args.camera_config.read_text(encoding="utf-8"))
#     require(config.get("schema_version") == 1, "Unexpected camera config schema")
#     require(config.get("camera_model") == "Logitech C270 HD Webcam", "Camera config model mismatch")
#     controls = config.get("controls")
#     require(isinstance(controls, dict) and controls, "Camera controls are missing")
#     allowed = {
#         "auto_exposure",
#         "exposure_dynamic_framerate",
#         "backlight_compensation",
#         "brightness",
#         "contrast",
#         "saturation",
#         "white_balance_automatic",
#         "power_line_frequency",
#         "sharpness",
#     }
#     require(set(controls) <= allowed, f"Unexpected camera controls: {set(controls) - allowed}")
#     assignments = ",".join(f"{name}={int(value)}" for name, value in controls.items())
#     subprocess.run(
#         ["v4l2-ctl", f"--device={args.device}", f"--set-ctrl={assignments}"],
#         check=True,
#     )
#     names = ",".join(controls)
#     verified = subprocess.run(
#         ["v4l2-ctl", f"--device={args.device}", f"--get-ctrl={names}"],
#         check=True,
#         capture_output=True,
#         text=True,
#     )
#     return " ".join(line.strip() for line in verified.stdout.splitlines() if line.strip())


# def encode_overlay(
#     frame_rgb: np.ndarray,
#     *,
#     slots: dict[str, list[int]],
#     raw_probabilities: dict[str, float | None],
#     displayed_states: dict[str, str],
#     preview_brightness: float,
# ) -> bytes:
#     image = Image.fromarray(frame_rgb, mode="RGB")
#     if preview_brightness != 1.0:
#         image = ImageEnhance.Brightness(image).enhance(preview_brightness)
#     draw = ImageDraw.Draw(image)
#     label_font = load_font(19, bold=True)
#     for slot in SLOT_ORDER:
#         x0, y0, x1, y1 = slots[slot]
#         state = displayed_states[slot]
#         color = STATE_COLORS[state]
#         draw.rectangle((x0, y0, x1, y1), outline=color, width=5)
#         probability = raw_probabilities.get(slot)
#         probability_text = "" if probability is None else f" p={probability:.3f}"
#         draw_text_box(
#             draw,
#             (x0 + 8, y0 + 8),
#             f"{slot} {state}{probability_text}",
#             font=label_font,
#             background=(0, 0, 0),
#             foreground=color,
#         )
#     output = BytesIO()
#     image.save(output, format="JPEG", quality=86, optimize=False)
#     return output.getvalue()


# def main() -> int:
#     args = parse_args()
#     require(args.display_fps > 0, "display-fps must be positive")
#     require(0.1 <= args.preview_brightness <= 1.5, "preview-brightness is out of range")
#     require(args.alignment_interval > 0, "alignment-interval must be positive")
#     require(0 <= args.port <= 65535, "Invalid HTTP port")
#     for path in (args.engine, args.deployment_metadata, args.thresholds, args.alignment_reference):
#         require(path.is_file(), f"Required file is missing: {path}")
#     require(Path(args.device).exists(), f"Camera device is unavailable: {args.device}")

#     metadata = json.loads(args.deployment_metadata.read_text(encoding="utf-8"))
#     thresholds_doc = json.loads(args.thresholds.read_text(encoding="utf-8"))
#     slots = validate_rois(metadata["roi_calibration"])
#     preprocessing = metadata["preprocessing"]
#     t_empty = float(thresholds_doc["thresholds"]["t_empty"])
#     t_occupied = float(thresholds_doc["thresholds"]["t_occupied"])
#     require((t_empty, t_occupied) == (0.005, 0.995), "Frozen thresholds changed")

#     camera_control_status = apply_camera_controls(args)
#     store = FrameStore()
#     server = MonitorHTTPServer((args.host, args.port), store)
#     server_thread = threading.Thread(target=server.serve_forever, name="http", daemon=True)
#     server_thread.start()
#     pose_guard = PoseGuard(
#         args.alignment_reference,
#         minimum_score=args.minimum_alignment_score,
#         max_allowed_shift=args.max_allowed_shift,
#     )
#     pose_guard.start()
#     runner = TensorRTRunner(args.engine)
#     runner.run(np.zeros((9, 3, 224, 224), dtype=np.float32))
#     ffmpeg = start_ffmpeg(args)
#     require(ffmpeg.stdout is not None, "FFmpeg stdout pipe is unavailable")

#     print(f"PASS engine_sha256={sha256_file(args.engine)}")
#     print(f"PASS camera controls: {camera_control_status}")
#     print(f"PASS TensorRT live monitor: http://127.0.0.1:{server.server_port}/")
#     if args.host == "0.0.0.0":
#         print(f"LAN URL: http://<JETSON-IP>:{server.server_port}/")
#     print("SAFETY: monitor-only; no UART device opened; STM32 bytes sent=0")

#     history: deque[dict[str, str]] = deque(maxlen=5)
#     frame_number = 0
#     last_pose_submit = 0.0
#     started = time.monotonic()
#     recent_completion_times: deque[float] = deque(maxlen=30)
#     try:
#         while args.max_frames <= 0 or frame_number < args.max_frames:
#             raw = read_exact(ffmpeg.stdout, FRAME_BYTES)
#             frame_number += 1
#             frame_rgb = np.frombuffer(raw, dtype=np.uint8).reshape(
#                 FRAME_HEIGHT, FRAME_WIDTH, 3
#             )
#             now = time.monotonic()
#             if frame_number == 1:
#                 pose = pose_guard.check_now(frame_rgb)
#                 last_pose_submit = now
#             else:
#                 pose = pose_guard.current()
#                 if now - last_pose_submit >= args.alignment_interval:
#                     pose_guard.submit(frame_rgb)
#                     last_pose_submit = now
#             pose_stale = pose is None or now - pose.updated_monotonic > max(
#                 8.0, args.alignment_interval * 4.0
#             )
#             camera_aligned = bool(pose is not None and pose.aligned and not pose_stale)

#             raw_probabilities: dict[str, float | None]
#             raw_states: dict[str, str]
#             inference_ms: float | None
#             if not camera_aligned:
#                 history.clear()
#                 raw_probabilities = {slot: None for slot in SLOT_ORDER}
#                 raw_states = {slot: "UNKNOWN" for slot in SLOT_ORDER}
#                 displayed_states = raw_states
#                 inference_ms = None
#                 monitor_status = "CAMERA_FAULT"
#             else:
#                 tensors = []
#                 image = Image.fromarray(frame_rgb, mode="RGB")
#                 for slot in SLOT_ORDER:
#                     x0, y0, x1, y1 = slots[slot]
#                     tensors.append(
#                         preprocess_pil_image(
#                             image.crop((x0, y0, x1, y1)), preprocessing
#                         )
#                     )
#                 probabilities, inference_ms = runner.run(np.stack(tensors))
#                 raw_probabilities = {}
#                 raw_states = {}
#                 for slot, probability in zip(SLOT_ORDER, probabilities, strict=True):
#                     value = float(probability)
#                     raw_probabilities[slot] = value
#                     raw_states[slot] = decide(value, t_empty, t_occupied)
#                 history.append(raw_states)
#                 if len(history) == 5:
#                     consensus = temporal_consensus(
#                         list(history), camera_faults=[False] * 5
#                     )
#                     displayed_states = consensus.stable_states
#                     monitor_status = (
#                         "STABLE" if consensus.occupancy_snapshot_valid else "UNKNOWN"
#                     )
#                 else:
#                     displayed_states = {slot: "UNKNOWN" for slot in SLOT_ORDER}
#                     monitor_status = "WARMING_UP"

#             recent_completion_times.append(time.monotonic())
#             display_rate = 0.0
#             if len(recent_completion_times) >= 2:
#                 display_rate = (len(recent_completion_times) - 1) / (
#                     recent_completion_times[-1] - recent_completion_times[0]
#                 )
#             jpeg = encode_overlay(
#                 frame_rgb,
#                 slots=slots,
#                 raw_probabilities=raw_probabilities,
#                 displayed_states=displayed_states,
#                 preview_brightness=args.preview_brightness,
#             )
#             status = {
#                 "status": monitor_status,
#                 "frame": frame_number,
#                 "uptime_seconds": now - started,
#                 "display_fps": display_rate,
#                 "preview_brightness": args.preview_brightness,
#                 "inference_uses_original_brightness": True,
#                 "inference_ms": inference_ms,
#                 "pose": None
#                 if pose is None
#                 else {
#                     "score": pose.score,
#                     "shift_x": pose.shift_x,
#                     "shift_y": pose.shift_y,
#                     "aligned": camera_aligned,
#                     "stale": pose_stale,
#                 },
#                 "raw_states": raw_states,
#                 "stable_states": displayed_states,
#                 "temporal_frames": len(history),
#                 "command_authorized": False,
#                 "command_block_reason": "MONITOR_ONLY_NO_STM32_TRANSPORT",
#                 "stm32_bytes_sent": 0,
#             }
#             store.publish(jpeg, status)
#     except KeyboardInterrupt:
#         print("STOP requested by user")
#     finally:
#         if ffmpeg.poll() is None:
#             ffmpeg.terminate()
#             try:
#                 ffmpeg.wait(timeout=3.0)
#             except subprocess.TimeoutExpired:
#                 ffmpeg.kill()
#                 ffmpeg.wait(timeout=3.0)
#         pose_guard.stop()
#         runner.close()
#         server.shutdown()
#         server.server_close()
#         server_thread.join(timeout=3.0)
#     print("PASS clean shutdown; STM32 bytes sent=0")
#     return 0


# if __name__ == "__main__":
#     raise SystemExit(main())

#!/usr/bin/env python3
"""Live 3x3 rack monitor with TensorRT inference and browser overlay.

Camera input is captured by FFmpeg/V4L2. Pillow draws the overlay and serves
MJPEG over HTTP. OpenCV is intentionally not used. This monitor never opens a
UART device and never sends an OpenRB robot command; transport is owned by the
external integration controller.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
import json
from pathlib import Path
import queue
import subprocess
import threading
import time
from typing import Any, BinaryIO

import numpy as np
from PIL import Image, ImageDraw, ImageEnhance, ImageFont

from check_rack_alignment import (
    measure_alignment_masks,
    rack_mask,
    rack_mask_rgb,
)
from infer_rack_frame_trt import SLOT_ORDER, validate_rois
from rack_safety_gate import temporal_consensus
from validate_tensorrt_parity import (
    TensorRTRunner,
    decide,
    preprocess_pil_image,
    require,
    sha256_file,
)


PROJECT_ROOT = Path(__file__).resolve().parents[2]
RUN_ID = "20260801T060414Z_c37b859a"
RUN_ROOT = PROJECT_ROOT / "deployment" / "jetson" / RUN_ID
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
FRAME_BYTES = FRAME_WIDTH * FRAME_HEIGHT * 3
STATE_COLORS = {
    "EMPTY": (32, 220, 90),
    "OCCUPIED": (245, 55, 55),
    "UNKNOWN": (255, 205, 35),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument(
        "--camera-config",
        type=Path,
        default=PROJECT_ROOT / "config" / "camera_c270_live.json",
    )
    parser.add_argument(
        "--skip-camera-config",
        action="store_true",
        help="Do not apply configured V4L2 controls before opening FFmpeg.",
    )
    parser.add_argument(
        "--engine",
        type=Path,
        default=RUN_ROOT / "engine" / "rack_occupancy_mobilenetv3_small_fp16_trt10.3.plan",
    )
    parser.add_argument(
        "--deployment-metadata",
        type=Path,
        default=RUN_ROOT / "source" / "deployment_metadata.json",
    )
    parser.add_argument(
        "--thresholds",
        type=Path,
        default=RUN_ROOT / "source" / "thresholds.json",
    )
    parser.add_argument(
        "--alignment-reference",
        type=Path,
        default=PROJECT_ROOT / "raw" / "s01_train" / "c270_calv1_s01_l00_a001.jpg",
    )
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", default=8080, type=int)
    parser.add_argument("--display-fps", default=10.0, type=float)
    parser.add_argument(
        "--preview-brightness",
        default=0.82,
        type=float,
        help="Display-only brightness factor; TensorRT always receives the unmodified frame.",
    )
    parser.add_argument("--alignment-interval", default=2.0, type=float)
    parser.add_argument("--max-allowed-shift", default=8, type=int)
    parser.add_argument("--minimum-alignment-score", default=0.35, type=float)
    parser.add_argument(
        "--max-frames",
        default=0,
        type=int,
        help="Exit after this many processed frames; 0 runs until Ctrl+C.",
    )
    return parser.parse_args()


def read_exact(stream: BinaryIO, size: int) -> bytes:
    output = bytearray(size)
    view = memoryview(output)
    position = 0
    while position < size:
        count = stream.readinto(view[position:])
        if not count:
            raise EOFError(f"FFmpeg ended after {position}/{size} frame bytes")
        position += count
    return bytes(output)


def load_font(size: int, *, bold: bool = False) -> ImageFont.ImageFont:
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    path = Path("/usr/share/fonts/truetype/dejavu") / name
    try:
        return ImageFont.truetype(str(path), size=size)
    except OSError:
        return ImageFont.load_default()


def draw_text_box(
    draw: ImageDraw.ImageDraw,
    position: tuple[int, int],
    text: str,
    *,
    font: ImageFont.ImageFont,
    foreground: tuple[int, int, int] = (255, 255, 255),
    background: tuple[int, int, int] = (0, 0, 0),
) -> None:
    x, y = position
    try:
        left, top, right, bottom = draw.textbbox((x, y), text, font=font)
        width, height = right - left, bottom - top
    except AttributeError:
        width, height = draw.textsize(text, font=font)
    padding = 4
    draw.rectangle(
        (x - padding, y - padding, x + width + padding, y + height + padding),
        fill=background,
    )
    draw.text((x, y), text, font=font, fill=foreground)


class FrameStore:
    def __init__(self) -> None:
        self.condition = threading.Condition()
        self.sequence = 0
        self.jpeg: bytes | None = None
        self.status: dict[str, Any] = {"status": "STARTING"}
        # 모니터가 재시작되어도 브라우저가 새 이벤트를 구분할 수 있도록
        # 순번을 현재 시각(밀리초)에서 시작한다.
        self.controller_event_sequence = int(time.time() * 1000)
        self.controller_events: deque[dict[str, Any]] = deque(maxlen=100)

    def _status_with_events(self) -> dict[str, Any]:
        status = dict(self.status)
        status["controller_events"] = list(self.controller_events)
        return status

    def publish(self, jpeg: bytes, status: dict[str, Any]) -> None:
        with self.condition:
            self.sequence += 1
            self.jpeg = jpeg
            self.status = status
            self.condition.notify_all()

    def snapshot(self) -> tuple[int, bytes | None, dict[str, Any]]:
        with self.condition:
            return self.sequence, self.jpeg, self._status_with_events()

    def add_controller_event(self, kind: str, tag: str, message: str) -> None:
        with self.condition:
            self.controller_event_sequence += 1
            self.controller_events.append(
                {
                    "id": self.controller_event_sequence,
                    "kind": kind,
                    "tag": tag,
                    "message": message,
                    "time": time.strftime("%H:%M:%S"),
                }
            )
            self.condition.notify_all()

    def wait_after(
        self, sequence: int, timeout: float = 5.0
    ) -> tuple[int, bytes | None, dict[str, Any]]:
        with self.condition:
            self.condition.wait_for(lambda: self.sequence > sequence, timeout=timeout)
            return self.sequence, self.jpeg, self._status_with_events()


class MonitorHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address: tuple[str, int], store: FrameStore) -> None:
        self.frame_store = store
        super().__init__(address, MonitorHandler)


class MonitorHandler(BaseHTTPRequestHandler):
    server: MonitorHTTPServer

    def log_message(self, format: str, *args: object) -> None:
        return

    def do_GET(self) -> None:
        if self.path in {"/", "/index.html"}:
            self._serve_index()
        elif self.path == "/stream.mjpg":
            self._serve_stream()
        elif self.path == "/snapshot.jpg":
            self._serve_snapshot()
        elif self.path == "/status.json":
            self._serve_status()
        else:
            self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        if self.path != "/controller-event":
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        if self.client_address[0] not in {"127.0.0.1", "::1"}:
            self.send_error(HTTPStatus.FORBIDDEN, "Local controller only")
            return
        try:
            content_length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self.send_error(HTTPStatus.BAD_REQUEST, "Invalid Content-Length")
            return
        if content_length < 2 or content_length > 4096:
            self.send_error(HTTPStatus.BAD_REQUEST, "Invalid event body size")
            return
        try:
            document = json.loads(self.rfile.read(content_length))
        except (json.JSONDecodeError, UnicodeDecodeError):
            self.send_error(HTTPStatus.BAD_REQUEST, "Invalid JSON")
            return
        if not isinstance(document, dict):
            self.send_error(HTTPStatus.BAD_REQUEST, "Event must be an object")
            return
        kind = document.get("kind")
        tag = document.get("tag")
        message = document.get("message")
        if kind not in {"info", "full", "warn", "fault"}:
            self.send_error(HTTPStatus.BAD_REQUEST, "Invalid event kind")
            return
        if not isinstance(tag, str) or not 1 <= len(tag) <= 20:
            self.send_error(HTTPStatus.BAD_REQUEST, "Invalid event tag")
            return
        if not isinstance(message, str) or not 1 <= len(message) <= 300:
            self.send_error(HTTPStatus.BAD_REQUEST, "Invalid event message")
            return
        self.server.frame_store.add_controller_event(kind, tag, message)
        self.send_response(HTTPStatus.NO_CONTENT)
        self.send_header("Cache-Control", "no-store")
        self.end_headers()

    def _serve_index(self) -> None:
        body = """<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>3×3 적재함 실시간 모니터</title>
<style>
:root{--bg:#0d0f12;--panel:#161a20;--panel2:#11151a;--line:#2b323d;--text:#f3f6fa;
--muted:#a9b3c1;--empty:#20dc5a;--occupied:#ff4545;--unknown:#ffcd23;--idle:#596575;
--out:#78aaff;--accent:#6aa8ff}
*{box-sizing:border-box}
html,body{height:100%}
body{margin:0;background:var(--bg);color:var(--text);display:flex;flex-direction:column;
overflow:hidden;font-size:16px;font-family:system-ui,-apple-system,"Segoe UI",Roboto,"Noto Sans KR",sans-serif}
header{flex:0 0 auto;display:flex;align-items:center;gap:18px;padding:13px 20px;
background:var(--panel);border-bottom:1px solid var(--line)}
header h1{margin:0;font-size:clamp(21px,1.15vw,30px);font-weight:800;letter-spacing:-.02em}
header .sub{color:var(--muted);font-size:clamp(14px,.72vw,18px);font-weight:600}
#fill{margin-left:auto;font-size:clamp(21px,1.15vw,30px);font-weight:900;font-variant-numeric:tabular-nums;
font-family:ui-monospace,SFMono-Regular,Menlo,monospace}
#badge{display:flex;align-items:center;gap:9px;padding:8px 15px;border:1px solid var(--line);
border-radius:999px;font-size:clamp(14px,.75vw,19px);font-weight:800;letter-spacing:.04em;
transition:color .25s,border-color .25s,box-shadow .25s}
#dot{width:11px;height:11px;border-radius:50%;background:var(--idle);transition:background .25s}
#badge[data-status="STABLE"] #dot{animation:statusPulse 1.8s ease-out infinite}
#fs{background:transparent;color:var(--muted);border:1px solid var(--line);border-radius:8px;
padding:8px 14px;font-size:14px;cursor:pointer;font-family:inherit;font-weight:700}
#fs:hover{color:var(--text);border-color:var(--muted)}
main{flex:1 1 auto;min-height:0;display:grid;grid-template-columns:minmax(0,1fr) clamp(520px,25vw,640px);
gap:14px;padding:14px}
@media(max-width:1200px){main{grid-template-columns:minmax(0,1fr)}}
.col{display:flex;flex-direction:column;gap:14px;min-height:0}
.col.side{overflow-y:auto;overflow-x:hidden}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:12px;
display:flex;flex-direction:column;min-height:0;overflow:hidden}
.panel h2{flex:0 0 auto;margin:0;padding:11px 15px;font-size:clamp(14px,.74vw,18px);font-weight:800;
letter-spacing:.12em;color:var(--muted);text-transform:uppercase;
border-bottom:1px solid var(--line);display:flex;align-items:center;gap:10px}
.panel h2 .note{margin-left:auto;letter-spacing:0;text-transform:none;font-weight:600;
font-size:clamp(13px,.68vw,17px);font-family:ui-monospace,SFMono-Regular,Menlo,monospace}
.panel .body{padding:16px;min-height:0}
.live-note{display:inline-flex;align-items:center;gap:8px;color:var(--empty)}
.live-note::before{content:"";width:9px;height:9px;border-radius:50%;background:currentColor;
box-shadow:0 0 0 0 rgba(32,220,90,.5);animation:statusPulse 1.8s ease-out infinite}
#streampanel{flex:1 1 auto;min-height:0}
#streampanel .body{flex:1 1 auto;min-height:0;padding:0;background:#000;
display:flex;align-items:center;justify-content:center}
#view{width:100%;height:100%;object-fit:contain;display:block}
#logpanel{flex:0 0 clamp(180px,25vh,300px)}
#log{flex:1 1 auto;min-height:0;overflow-y:auto}
#log:empty::after{content:"이벤트를 기다리는 중입니다.";display:block;padding:16px;
color:var(--idle);font-size:17px}
.logrow{display:flex;gap:13px;align-items:baseline;padding:10px 16px;border-bottom:1px solid #1d222a;
animation:logIn .35s ease-out both}
.logrow .t{flex:0 0 auto;font-size:14px;color:var(--muted);
font-family:ui-monospace,SFMono-Regular,Menlo,monospace}
.logrow .tag{flex:0 0 auto;font-size:13px;font-weight:800;padding:4px 10px;border-radius:6px;
background:#232a34;color:var(--muted)}
.logrow .m{font-size:clamp(18px,.95vw,24px);font-weight:750;line-height:1.35}
.logrow.load .m{color:var(--empty)}
.logrow.load .tag{background:#123020;color:var(--empty)}
.logrow.full .m{color:var(--empty);font-size:25px}
.logrow.full .tag{background:#123020;color:var(--empty)}
.logrow.out .m{color:var(--out)}
.logrow.out .tag{background:#16233c;color:var(--out)}
.logrow.warn .m{color:var(--unknown)}
.logrow.warn .tag{background:#332b0e;color:var(--unknown)}
.logrow.bad .m{color:var(--occupied)}
.logrow.bad .tag{background:#331616;color:var(--occupied)}
.logrow.info .m{color:var(--muted);font-size:clamp(17px,.88vw,22px);font-weight:650}
#grid{display:grid;gap:12px}
.cell{border:1px solid var(--line);border-left:7px solid var(--idle);border-radius:10px;
padding:15px 16px;background:var(--panel2);transition:border-color .25s,background .25s,
box-shadow .25s,transform .25s}
.cell .slot{display:block;font-size:clamp(14px,.72vw,18px);font-weight:700;color:var(--muted);letter-spacing:.08em}
.cell .state{display:block;margin-top:7px;font-size:clamp(22px,1.2vw,30px);font-weight:900;
line-height:1.1;color:var(--idle);white-space:nowrap}
.cell .raw{display:block;margin-top:6px;font-size:13px;color:var(--muted);min-height:17px}
.cell.EMPTY{border-left-color:var(--empty)}
.cell.EMPTY .state{color:var(--empty)}
.cell.OCCUPIED{border-left-color:var(--occupied)}
.cell.OCCUPIED .state{color:var(--occupied)}
.cell.UNKNOWN{border-left-color:var(--unknown)}
.cell.UNKNOWN .state{color:var(--unknown)}
.cell.changed{animation:slotChanged .65s ease-out}
#final-count{font-size:clamp(42px,2.3vw,58px);font-weight:900;line-height:1;color:var(--text);
font-variant-numeric:tabular-nums;font-family:ui-monospace,SFMono-Regular,Menlo,monospace}
#final-count.bump{animation:countBump .5s ease-out}
#final-bar{margin:15px 0 12px;height:13px;border-radius:999px;background:#0e1217;overflow:hidden}
#final-fill{display:block;height:100%;width:0;background:linear-gradient(90deg,#14b84a,var(--empty),#75ff9d);
background-size:200% 100%;transition:width .55s cubic-bezier(.2,.8,.2,1);animation:barFlow 2.2s linear infinite}
#final-list{font-size:clamp(16px,.84vw,21px);line-height:1.7;word-break:break-all;
font-family:ui-monospace,SFMono-Regular,Menlo,monospace}
#final-note{margin-top:10px;font-size:15px;color:var(--muted);line-height:1.5}
#finalpanel.done{border-color:var(--empty)}
#finalpanel.done #final-count{color:var(--empty)}
#finalpanel.done{animation:doneGlow 2s ease-in-out infinite}
.metrics{display:grid;grid-template-columns:1fr 1fr;gap:1px;background:var(--line)}
.metric{background:var(--panel);padding:13px 15px}
.metric .k{font-size:13px;letter-spacing:.06em;color:var(--muted);font-weight:700}
.metric .v{margin-top:5px;font-size:clamp(19px,1vw,25px);font-weight:800;font-variant-numeric:tabular-nums;
font-family:ui-monospace,SFMono-Regular,Menlo,monospace}
.health-summary{display:flex;align-items:center;gap:13px;padding:13px 14px;margin-bottom:8px;
border:1px solid var(--line);border-radius:10px;background:var(--panel2)}
.health-icon{width:14px;height:14px;flex:0 0 auto;border-radius:50%;background:var(--idle)}
.health-title{font-size:20px;font-weight:850}.health-sub{margin-top:3px;color:var(--muted);font-size:14px}
.rows{font-size:15px}
.row{display:flex;justify-content:space-between;gap:12px;padding:9px 0;border-top:1px solid var(--line)}
.row:first-child{border-top:0}
.row .k{color:var(--muted);font-weight:650}
.row .v{font-size:15px;font-weight:750;font-family:ui-monospace,SFMono-Regular,Menlo,monospace;text-align:right}
.ok{color:var(--empty)}.warn{color:var(--unknown)}.bad{color:var(--occupied)}
footer{flex:0 0 auto;padding:0 18px 11px;color:var(--muted);font-size:13px}
@keyframes statusPulse{0%{box-shadow:0 0 0 0 currentColor}70%{box-shadow:0 0 0 8px transparent}100%{box-shadow:0 0 0 0 transparent}}
@keyframes logIn{from{opacity:0;transform:translateY(-8px)}to{opacity:1;transform:translateY(0)}}
@keyframes slotChanged{0%{transform:scale(.96);box-shadow:0 0 0 0 currentColor}45%{transform:scale(1.025);box-shadow:0 0 24px -5px currentColor}100%{transform:scale(1);box-shadow:none}}
@keyframes countBump{40%{transform:scale(1.08)}100%{transform:scale(1)}}
@keyframes barFlow{to{background-position:-200% 0}}
@keyframes doneGlow{50%{box-shadow:0 0 24px -10px var(--empty)}}
@media(prefers-reduced-motion:reduce){*,*::before,*::after{animation:none!important;transition:none!important}}
</style></head><body>
<header>
<h1>3×3 적재함 실시간 모니터</h1>
<span class="sub">TensorRT 비전 판독 &middot; OpenRB 연동</span>
<span id="fill">적재 - / -</span>
<span id="badge"><span id="dot"></span><span id="badgetext">CONNECTING</span></span>
<button id="fs" type="button">전체 화면 (F)</button>
</header>
<main>
<div class="col">
<section class="panel" id="streampanel">
<h2>실시간 카메라</h2>
<div class="body"><img id="view" src="/stream.mjpg" alt="live rack stream"></div>
</section>
<section class="panel" id="logpanel">
<h2>이벤트 기록<span class="note" id="logcount"></span></h2>
<div id="log"></div>
</section>
</div>
<div class="col side">
<section class="panel">
<h2>슬롯 상태<span class="note live-note" id="slotnote">실시간</span></h2>
<div class="body"><div id="grid"></div></div>
</section>
<section class="panel" id="finalpanel">
<h2>최종 적재 현황<span class="note" id="final-time">최근 확정 -</span></h2>
<div class="body">
<div id="final-count">- / -</div>
<div id="final-bar"><span id="final-fill"></span></div>
<div id="final-list">-</div>
<div id="final-note">-</div>
</div>
</section>
<section class="panel">
<h2>실행 상태</h2>
<div class="metrics">
<div class="metric"><div class="k">TensorRT 추론</div><div class="v" id="m-trt">-</div></div>
<div class="metric"><div class="k">화면 FPS</div><div class="v" id="m-fps">-</div></div>
<div class="metric"><div class="k">처리 프레임</div><div class="v" id="m-frame">-</div></div>
<div class="metric"><div class="k">실행 시간</div><div class="v" id="m-up">-</div></div>
</div>
</section>
<section class="panel" id="healthpanel">
<h2>카메라 &middot; 판정 상태</h2>
<div class="body">
<div class="health-summary">
<span class="health-icon" id="health-icon"></span>
<div><div class="health-title" id="health-title">상태 확인 중</div>
<div class="health-sub" id="health-sub">카메라 판독 정보를 기다리고 있습니다.</div></div>
</div>
<div class="rows">
<div class="row"><span class="k">적재함 정렬</span><span class="v" id="p-align">-</span></div>
<div class="row"><span class="k">정렬 신뢰도</span><span class="v" id="p-score">-</span></div>
<div class="row"><span class="k">위치 보정 (x, y)</span><span class="v" id="p-shift">-</span></div>
<div class="row"><span class="k">판정 합의 프레임</span><span class="v" id="p-hist">-</span></div>
<div class="row"><span class="k">제어 모듈</span><span class="v">OpenRB · 별도 실행</span></div>
</div>
</div>
</section>
</div>
</main>
<footer>화면 밝기는 미리보기에만 적용됩니다. TensorRT는 원본 프레임을 사용합니다.</footer>
<script>
const el=id=>document.getElementById(id);
const BADGE={STABLE:"var(--empty)",WARMING_UP:"var(--unknown)",UNKNOWN:"var(--unknown)",
CAMERA_FAULT:"var(--occupied)",DISCONNECTED:"var(--occupied)"};
const STATUS_LABEL={STABLE:"판정 안정",WARMING_UP:"판정 준비",UNKNOWN:"판정 보류",
CAMERA_FAULT:"카메라 오류",DISCONNECTED:"연결 끊김"};
const STATE_LABEL={EMPTY:"비어 있음",OCCUPIED:"적재됨",UNKNOWN:"판정 대기"};
const LOG_MAX=300;
let gridSig=null,cells=[];
let prevStable=null,prevStatus=null,prevFull=false,connected=true,logCount=0;
let lastStable=null,lastStableAt=null,lastFinalCount=null;
let lastControllerEventId=0;
const loggedOccupied=new Set();

function stamp(){
 const d=new Date();
 return [d.getHours(),d.getMinutes(),d.getSeconds()]
  .map(v=>String(v).padStart(2,"0")).join(":");
}
function orderKeys(keys){
 const parsed=keys.map(k=>{
  const m=/^C([0-9]+)_L([0-9]+)$/.exec(k);
  return {k:k,c:m?parseInt(m[1],10):0,l:m?parseInt(m[2],10):0,ok:m!==null};
 });
 if(parsed.length&&parsed.every(p=>p.ok)){
  parsed.sort((a,b)=>(b.l-a.l)||(a.c-b.c));
  return parsed.map(p=>p.k);
 }
 return keys.slice().sort();
}
function logEvent(cls,tag,text,eventTime){
 const box=el("log");
 const row=document.createElement("div");
 row.className="logrow "+cls;
 const t=document.createElement("span");t.className="t";t.textContent=eventTime||stamp();
 const g=document.createElement("span");g.className="tag";g.textContent=tag;
 const m=document.createElement("span");m.className="m";m.textContent=text;
 row.appendChild(t);row.appendChild(g);row.appendChild(m);
 box.insertBefore(row,box.firstChild);
 while(box.childElementCount>LOG_MAX)box.removeChild(box.lastChild);
 logCount+=1;
 el("logcount").textContent=logCount+"건";
}
function renderControllerEvents(events){
 if(!Array.isArray(events))return;
 events.forEach(event=>{
  const id=Number(event.id)||0;
  if(id<=lastControllerEventId)return;
  lastControllerEventId=id;
  const cls=event.kind==="fault"?"bad":(event.kind==="info"?"info":"warn");
  logEvent(cls,event.tag||"제어",event.message||"제어 이벤트가 발생했습니다.",event.time);
 });
}
function statusMessage(st){
 if(st==="STABLE")return "판독이 안정되어 전체 슬롯 판정이 확정되었습니다.";
 if(st==="UNKNOWN")return "일부 슬롯의 판정이 보류되었습니다. 로봇 팔 등에 가려졌을 수 있습니다.";
 if(st==="WARMING_UP")return "합의 프레임을 수집하고 있습니다. 5프레임이 모이면 판정이 확정됩니다.";
 if(st==="CAMERA_FAULT")return "카메라 정렬이 기준을 벗어나 판정을 중단했습니다.";
 return "판독 상태가 "+st+" 로 변경되었습니다.";
}
function statusStyle(st){
 if(st==="STABLE")return {cls:"info",tag:"판독"};
 if(st==="CAMERA_FAULT")return {cls:"bad",tag:"카메라"};
 return {cls:"warn",tag:"판독"};
}
function uptime(v){
 const t=Math.max(0,Math.floor(v||0));
 const h=Math.floor(t/3600),m=Math.floor((t%3600)/60),s=t%60;
 return (h?h+"시간 ":"")+m+"분 "+String(s).padStart(2,"0")+"초";
}
function buildGrid(keys){
 const s=keys.join("|");
 if(s===gridSig)return;
 gridSig=s;
 const g=el("grid");
 g.textContent="";
 cells=[];
 g.style.gridTemplateColumns="repeat("+(Math.ceil(Math.sqrt(keys.length))||1)+",minmax(0,1fr))";
 keys.forEach(k=>{
  const c=document.createElement("div");c.className="cell";
  const a=document.createElement("span");a.className="slot";a.textContent=k;
  const b=document.createElement("span");b.className="state";b.textContent="-";
  const d=document.createElement("span");d.className="raw";
  c.appendChild(a);c.appendChild(b);c.appendChild(d);
  g.appendChild(c);
  cells.push({key:k,box:c,state:b,raw:d});
 });
}
function setBadge(status,color){
 el("badgetext").textContent=STATUS_LABEL[status]||status;
 el("dot").style.background=color||"var(--idle)";
 el("badge").style.color=color||"var(--text)";
 el("badge").style.borderColor=color||"var(--line)";
 el("badge").dataset.status=status;
}
function renderHealth(s){
 const st=s.status||"DISCONNECTED",color=BADGE[st]||"var(--idle)";
 const title=STATUS_LABEL[st]||st;
 const sub={STABLE:"적재함 정렬과 5프레임 판정이 모두 안정적입니다.",
  WARMING_UP:"안정적인 판정을 위해 5프레임을 수집하고 있습니다.",
  UNKNOWN:"일부 슬롯이 가려지거나 판정이 흔들려 잠시 보류했습니다.",
  CAMERA_FAULT:"카메라 위치와 밝기, 적재함 정렬을 확인해 주세요.",
  DISCONNECTED:"모니터 서버와의 연결을 확인해 주세요."}[st]||"현재 판독 상태를 확인하고 있습니다.";
 el("health-title").textContent=title;
 el("health-sub").textContent=sub;
 el("health-title").style.color=color;
 el("health-icon").style.background=color;
 el("health-icon").style.boxShadow="0 0 16px "+color;
}
function renderFinal(){
 if(lastStable===null){
  el("final-count").textContent="- / -";
  el("final-list").textContent="아직 확정된 판독 결과가 없습니다.";
  el("final-note").textContent="판독이 안정되면 이곳에 확정 결과가 표시됩니다.";
  el("final-time").textContent="최근 확정 -";
  el("finalpanel").className="panel";
  return;
 }
 const ks=orderKeys(Object.keys(lastStable));
 const occ=ks.filter(k=>lastStable[k]==="OCCUPIED");
 const done=ks.length>0&&occ.length===ks.length;
 el("final-count").textContent=occ.length+" / "+ks.length;
 if(lastFinalCount!==null&&lastFinalCount!==occ.length){
  const count=el("final-count");count.classList.remove("bump");void count.offsetWidth;count.classList.add("bump");
 }
 lastFinalCount=occ.length;
 el("final-fill").style.width=(ks.length?(occ.length/ks.length*100):0)+"%";
 el("final-list").textContent=occ.length?occ.join("   "):"적재된 슬롯이 없습니다.";
 el("final-time").textContent="최근 확정 "+(lastStableAt||"-");
 el("final-note").textContent=done
  ?"전체 슬롯 적재가 완료되었습니다."
  :"마지막으로 판독이 확정된 시점을 기준으로 표시합니다.";
 el("finalpanel").className=done?"panel done":"panel";
}
async function tick(){
 let s;
 try{
  const r=await fetch("/status.json",{cache:"no-store"});
  if(!r.ok)throw new Error("bad status");
  s=await r.json();
 }catch(e){
  setBadge("DISCONNECTED",BADGE.DISCONNECTED);
  renderHealth({status:"DISCONNECTED"});
  if(connected){
   connected=false;prevStatus=null;
   logEvent("bad","연결","모니터 서버와의 연결이 끊겼습니다.");
  }
  return;
 }
 if(!connected){connected=true;logEvent("info","연결","모니터 서버와 다시 연결되었습니다.");}
 setBadge(s.status,BADGE[s.status]||"var(--idle)");
 renderHealth(s);
 renderControllerEvents(s.controller_events);
 const live=el("slotnote");
 live.textContent=s.status==="STABLE"?"실시간 · 판정 확정":"실시간 · 확인 중";
 live.style.color=BADGE[s.status]||"var(--muted)";

 const stable=s.stable_states||{},raw=s.raw_states||{};
 const keys=orderKeys(Object.keys(stable));
 buildGrid(keys);
 cells.forEach(c=>{
  const st=stable[c.key]||"UNKNOWN",rw=raw[c.key]||"UNKNOWN";
  const before=c.box.dataset.state;
  c.box.className="cell "+st;
  c.box.dataset.state=st;
  c.state.textContent=STATE_LABEL[st]||st;
  c.raw.textContent=(rw===st)?"":"즉시 판정: "+(STATE_LABEL[rw]||rw);
  if(before&&before!==st){
   c.box.classList.add("changed");
   window.setTimeout(()=>c.box.classList.remove("changed"),700);
  }
 });
 const occupied=keys.filter(k=>stable[k]==="OCCUPIED").length;
 el("fill").textContent="적재 "+occupied+" / "+keys.length;

 if(s.status!==prevStatus){
  const sty=statusStyle(s.status);
  if(prevStatus===null)logEvent("info","시스템","모니터에 연결되었습니다. "+statusMessage(s.status));
  else logEvent(sty.cls,sty.tag,statusMessage(s.status));
  prevStatus=s.status;
 }
 if(s.status==="STABLE"){
  if(prevStable===null){
   logEvent("info","시스템","기준 상태를 등록했습니다. 현재 "+occupied+" / "+keys.length+"칸이 적재되어 있습니다.");
   keys.forEach(k=>{if(stable[k]==="OCCUPIED")loggedOccupied.add(k);});
   prevFull=(keys.length>0&&occupied===keys.length);
  }else{
   keys.forEach(k=>{
    const a=prevStable[k],b=stable[k];
    if(a===b||a===undefined)return;
    if(b==="OCCUPIED"&&!loggedOccupied.has(k)){
     loggedOccupied.add(k);
     logEvent("load","슬롯",k+" 슬롯 적재가 완료되었습니다.");
    }
   });
   const full=(keys.length>0&&occupied===keys.length);
   if(full&&!prevFull)logEvent("full","슬롯","전체 슬롯 적재가 완료되었습니다. ("+occupied+" / "+keys.length+")");
   prevFull=full;
  }
  prevStable=Object.assign({},stable);
  lastStable=Object.assign({},stable);
  lastStableAt=stamp();
 }
 renderFinal();

 el("m-trt").textContent=(s.inference_ms==null)?"차단됨":s.inference_ms.toFixed(1)+" ms";
 el("m-fps").textContent=(s.display_fps??0).toFixed(1);
 el("m-frame").textContent=s.frame??"-";
 el("m-up").textContent=uptime(s.uptime_seconds);
 const p=s.pose;
 el("p-score").textContent=p?p.score.toFixed(3):"-";
 el("p-shift").textContent=p?(p.shift_x+", "+p.shift_y):"-";
 const al=el("p-align");
 al.textContent=p?(p.aligned?"정상":(p.stale?"갱신 필요":"확인 필요")):"-";
 al.className="v "+((p&&p.aligned)?"ok":"bad");
 el("p-hist").textContent=(s.temporal_frames??0)+" / 5";
}
function toggleFullscreen(){
 if(document.fullscreenElement)document.exitFullscreen();
 else document.documentElement.requestFullscreen();
}
el("fs").addEventListener("click",toggleFullscreen);
document.addEventListener("keydown",e=>{
 if(e.key==="f"||e.key==="F")toggleFullscreen();
});
renderFinal();
tick();
setInterval(tick,500);
</script>
</body></html>""".encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_snapshot(self) -> None:
        _, jpeg, _ = self.server.frame_store.snapshot()
        if jpeg is None:
            self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "No frame yet")
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(jpeg)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(jpeg)

    def _serve_status(self) -> None:
        _, _, status = self.server.frame_store.snapshot()
        body = json.dumps(status, ensure_ascii=False, sort_keys=True).encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _serve_stream(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        sequence = -1
        try:
            while True:
                sequence, jpeg, _ = self.server.frame_store.wait_after(sequence)
                if jpeg is None:
                    continue
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            return


@dataclass
class PoseResult:
    score: float
    shift_x: int
    shift_y: int
    aligned: bool
    updated_monotonic: float


class PoseGuard:
    def __init__(
        self,
        reference: Path,
        *,
        minimum_score: float,
        max_allowed_shift: int,
        search_radius: int = 120,
        downsample: int = 4,
    ) -> None:
        self.minimum_score = minimum_score
        self.max_allowed_shift = max_allowed_shift
        self.search_radius = search_radius
        self.downsample = downsample
        self.reference_mask = rack_mask(reference, downsample)
        self.queue: queue.Queue[np.ndarray | None] = queue.Queue(maxsize=1)
        self.lock = threading.Lock()
        self.result: PoseResult | None = None
        self.thread = threading.Thread(target=self._worker, name="pose-guard", daemon=True)

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        try:
            self.queue.put_nowait(None)
        except queue.Full:
            try:
                self.queue.get_nowait()
            except queue.Empty:
                pass
            self.queue.put_nowait(None)
        self.thread.join(timeout=3.0)

    def submit(self, rgb: np.ndarray) -> None:
        try:
            self.queue.put_nowait(rgb.copy())
        except queue.Full:
            try:
                self.queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.queue.put_nowait(rgb.copy())
            except queue.Full:
                pass

    def current(self) -> PoseResult | None:
        with self.lock:
            return self.result

    def check_now(self, rgb: np.ndarray) -> PoseResult:
        result = self._measure(rgb)
        with self.lock:
            self.result = result
        return result

    def _measure(self, rgb: np.ndarray) -> PoseResult:
        current_mask = rack_mask_rgb(rgb, self.downsample)
        score, shift_x, shift_y = measure_alignment_masks(
            self.reference_mask,
            current_mask,
            downsample=self.downsample,
            search_radius=self.search_radius,
        )
        aligned = (
            score >= self.minimum_score
            and abs(shift_x) <= self.max_allowed_shift
            and abs(shift_y) <= self.max_allowed_shift
        )
        return PoseResult(score, shift_x, shift_y, aligned, time.monotonic())

    def _worker(self) -> None:
        while True:
            rgb = self.queue.get()
            if rgb is None:
                return
            try:
                result = self._measure(rgb)
            except Exception:
                result = PoseResult(0.0, 0, 0, False, time.monotonic())
            with self.lock:
                self.result = result


def start_ffmpeg(args: argparse.Namespace) -> subprocess.Popen[bytes]:
    command = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "warning",
        "-f",
        "v4l2",
        "-input_format",
        "mjpeg",
        "-video_size",
        f"{FRAME_WIDTH}x{FRAME_HEIGHT}",
        "-framerate",
        "30",
        "-i",
        args.device,
        "-vf",
        f"fps={args.display_fps:g}",
        "-pix_fmt",
        "rgb24",
        "-f",
        "rawvideo",
        "pipe:1",
    ]
    return subprocess.Popen(command, stdout=subprocess.PIPE, bufsize=FRAME_BYTES * 2)


def apply_camera_controls(args: argparse.Namespace) -> str:
    if args.skip_camera_config:
        return "SKIPPED by --skip-camera-config"
    require(args.camera_config.is_file(), f"Camera config is missing: {args.camera_config}")
    config = json.loads(args.camera_config.read_text(encoding="utf-8"))
    require(config.get("schema_version") == 1, "Unexpected camera config schema")
    require(config.get("camera_model") == "Logitech C270 HD Webcam", "Camera config model mismatch")
    controls = config.get("controls")
    require(isinstance(controls, dict) and controls, "Camera controls are missing")
    allowed = {
        "auto_exposure",
        "exposure_dynamic_framerate",
        "backlight_compensation",
        "brightness",
        "contrast",
        "saturation",
        "white_balance_automatic",
        "power_line_frequency",
        "sharpness",
    }
    require(set(controls) <= allowed, f"Unexpected camera controls: {set(controls) - allowed}")
    assignments = ",".join(f"{name}={int(value)}" for name, value in controls.items())
    subprocess.run(
        ["v4l2-ctl", f"--device={args.device}", f"--set-ctrl={assignments}"],
        check=True,
    )
    names = ",".join(controls)
    verified = subprocess.run(
        ["v4l2-ctl", f"--device={args.device}", f"--get-ctrl={names}"],
        check=True,
        capture_output=True,
        text=True,
    )
    return " ".join(line.strip() for line in verified.stdout.splitlines() if line.strip())


def encode_overlay(
    frame_rgb: np.ndarray,
    *,
    slots: dict[str, list[int]],
    raw_probabilities: dict[str, float | None],
    displayed_states: dict[str, str],
    preview_brightness: float,
) -> bytes:
    image = Image.fromarray(frame_rgb, mode="RGB")
    if preview_brightness != 1.0:
        image = ImageEnhance.Brightness(image).enhance(preview_brightness)
    draw = ImageDraw.Draw(image)
    label_font = load_font(19, bold=True)
    for slot in SLOT_ORDER:
        x0, y0, x1, y1 = slots[slot]
        state = displayed_states[slot]
        color = STATE_COLORS[state]
        draw.rectangle((x0, y0, x1, y1), outline=color, width=5)
        probability = raw_probabilities.get(slot)
        probability_text = "" if probability is None else f" p={probability:.3f}"
        draw_text_box(
            draw,
            (x0 + 8, y0 + 8),
            f"{slot} {state}{probability_text}",
            font=label_font,
            background=(0, 0, 0),
            foreground=color,
        )
    output = BytesIO()
    image.save(output, format="JPEG", quality=86, optimize=False)
    return output.getvalue()


def main() -> int:
    args = parse_args()
    require(args.display_fps > 0, "display-fps must be positive")
    require(0.1 <= args.preview_brightness <= 1.5, "preview-brightness is out of range")
    require(args.alignment_interval > 0, "alignment-interval must be positive")
    require(0 <= args.port <= 65535, "Invalid HTTP port")
    for path in (args.engine, args.deployment_metadata, args.thresholds, args.alignment_reference):
        require(path.is_file(), f"Required file is missing: {path}")
    require(Path(args.device).exists(), f"Camera device is unavailable: {args.device}")

    metadata = json.loads(args.deployment_metadata.read_text(encoding="utf-8"))
    thresholds_doc = json.loads(args.thresholds.read_text(encoding="utf-8"))
    slots = validate_rois(metadata["roi_calibration"])
    preprocessing = metadata["preprocessing"]
    t_empty = float(thresholds_doc["thresholds"]["t_empty"])
    t_occupied = float(thresholds_doc["thresholds"]["t_occupied"])
    require((t_empty, t_occupied) == (0.005, 0.995), "Frozen thresholds changed")

    camera_control_status = apply_camera_controls(args)
    store = FrameStore()
    server = MonitorHTTPServer((args.host, args.port), store)
    server_thread = threading.Thread(target=server.serve_forever, name="http", daemon=True)
    server_thread.start()
    pose_guard = PoseGuard(
        args.alignment_reference,
        minimum_score=args.minimum_alignment_score,
        max_allowed_shift=args.max_allowed_shift,
    )
    pose_guard.start()
    runner = TensorRTRunner(args.engine)
    runner.run(np.zeros((9, 3, 224, 224), dtype=np.float32))
    ffmpeg = start_ffmpeg(args)
    require(ffmpeg.stdout is not None, "FFmpeg stdout pipe is unavailable")

    print(f"PASS engine_sha256={sha256_file(args.engine)}")
    print(f"PASS camera controls: {camera_control_status}")
    print(f"PASS TensorRT live monitor: http://127.0.0.1:{server.server_port}/")
    if args.host == "0.0.0.0":
        print(f"LAN URL: http://<JETSON-IP>:{server.server_port}/")
    print("SAFETY: vision monitor only; OpenRB transport runs in the integration controller")

    history: deque[dict[str, str]] = deque(maxlen=5)
    frame_number = 0
    last_pose_submit = 0.0
    started = time.monotonic()
    recent_completion_times: deque[float] = deque(maxlen=30)
    try:
        while args.max_frames <= 0 or frame_number < args.max_frames:
            raw = read_exact(ffmpeg.stdout, FRAME_BYTES)
            frame_number += 1
            frame_rgb = np.frombuffer(raw, dtype=np.uint8).reshape(
                FRAME_HEIGHT, FRAME_WIDTH, 3
            )
            now = time.monotonic()
            if frame_number == 1:
                pose = pose_guard.check_now(frame_rgb)
                last_pose_submit = now
            else:
                pose = pose_guard.current()
                if now - last_pose_submit >= args.alignment_interval:
                    pose_guard.submit(frame_rgb)
                    last_pose_submit = now
            pose_stale = pose is None or now - pose.updated_monotonic > max(
                8.0, args.alignment_interval * 4.0
            )
            camera_aligned = bool(pose is not None and pose.aligned and not pose_stale)

            raw_probabilities: dict[str, float | None]
            raw_states: dict[str, str]
            inference_ms: float | None
            if not camera_aligned:
                history.clear()
                raw_probabilities = {slot: None for slot in SLOT_ORDER}
                raw_states = {slot: "UNKNOWN" for slot in SLOT_ORDER}
                displayed_states = raw_states
                inference_ms = None
                monitor_status = "CAMERA_FAULT"
            else:
                tensors = []
                image = Image.fromarray(frame_rgb, mode="RGB")
                for slot in SLOT_ORDER:
                    x0, y0, x1, y1 = slots[slot]
                    tensors.append(
                        preprocess_pil_image(
                            image.crop((x0, y0, x1, y1)), preprocessing
                        )
                    )
                probabilities, inference_ms = runner.run(np.stack(tensors))
                raw_probabilities = {}
                raw_states = {}
                for slot, probability in zip(SLOT_ORDER, probabilities, strict=True):
                    value = float(probability)
                    raw_probabilities[slot] = value
                    raw_states[slot] = decide(value, t_empty, t_occupied)
                history.append(raw_states)
                if len(history) == 5:
                    consensus = temporal_consensus(
                        list(history), camera_faults=[False] * 5
                    )
                    displayed_states = consensus.stable_states
                    monitor_status = (
                        "STABLE" if consensus.occupancy_snapshot_valid else "UNKNOWN"
                    )
                else:
                    displayed_states = {slot: "UNKNOWN" for slot in SLOT_ORDER}
                    monitor_status = "WARMING_UP"

            recent_completion_times.append(time.monotonic())
            display_rate = 0.0
            if len(recent_completion_times) >= 2:
                display_rate = (len(recent_completion_times) - 1) / (
                    recent_completion_times[-1] - recent_completion_times[0]
                )
            jpeg = encode_overlay(
                frame_rgb,
                slots=slots,
                raw_probabilities=raw_probabilities,
                displayed_states=displayed_states,
                preview_brightness=args.preview_brightness,
            )
            status = {
                "status": monitor_status,
                "frame": frame_number,
                "uptime_seconds": now - started,
                "display_fps": display_rate,
                "preview_brightness": args.preview_brightness,
                "inference_uses_original_brightness": True,
                "inference_ms": inference_ms,
                "pose": None
                if pose is None
                else {
                    "score": pose.score,
                    "shift_x": pose.shift_x,
                    "shift_y": pose.shift_y,
                    "aligned": camera_aligned,
                    "stale": pose_stale,
                },
                "raw_states": raw_states,
                "stable_states": displayed_states,
                "temporal_frames": len(history),
                "monitor_mode": "VISION_ONLY",
                "robot_transport": "EXTERNAL_OPENRB_CONTROLLER",
            }
            store.publish(jpeg, status)
    except KeyboardInterrupt:
        print("STOP requested by user")
    finally:
        if ffmpeg.poll() is None:
            ffmpeg.terminate()
            try:
                ffmpeg.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                ffmpeg.kill()
                ffmpeg.wait(timeout=3.0)
        pose_guard.stop()
        runner.close()
        server.shutdown()
        server.server_close()
        server_thread.join(timeout=3.0)
    print("PASS clean shutdown; vision monitor stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
