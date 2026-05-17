"""
test_semantic.py — live visualizer for the semantic_service /detect endpoint.

Default mode (no args) is MONITOR: passively polls /debug_image +
/debug_json and shows whatever the service most recently processed,
regardless of who pushed it (typically RTAB-Map GUI feeding it Unity TCP
frames). Use this to watch live keyframes get segmented in real time
without disturbing the pipeline.

Passing an input file or --camera switches to SENDER mode, where this
script pushes frames itself — useful as a standalone sanity check when
RTAB-Map isn't running.

Usage:
    # MONITOR (default) — watch what RTAB-Map is sending right now
    python test_semantic.py

    # SENDER — single image
    python test_semantic.py path/to/image.jpg

    # SENDER — video file (auto-play, every frame)
    python test_semantic.py path/to/video.mp4

    # SENDER — video sampled every Nth frame
    python test_semantic.py video.mp4 --stride 2

    # SENDER — webcam
    python test_semantic.py --camera 0

    # remote service
    python test_semantic.py --service http://127.0.0.1:7788

Keys (OpenCV window):
    SPACE       pause / resume (sender mode only)
    s           save current annotated frame to ./test_output/
    q / ESC     quit
"""

from __future__ import annotations

import argparse
import base64
import io
import json
import time
from collections import deque
from pathlib import Path
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import cv2
import numpy as np
from PIL import Image


# Catalog-aware colors. Wall-like classes (centerLine / parkingLine) get red;
# others get distinct hues so future labels remain visible without changes.
_LABEL_COLOR_BGR: dict[str, tuple[int, int, int]] = {
    "centerLine":   ( 40,  40, 220),
    "parkingLine":  ( 40,  40, 220),
    "Arrow":        (220, 200,  60),
    "rubberCone":   (  0, 140, 255),
    "sign":         (  0, 200, 255),
    "word":         (200,  60, 200),
}
_DEFAULT_COLOR_BGR = (180, 180, 180)


def _color(label: str) -> tuple[int, int, int]:
    return _LABEL_COLOR_BGR.get(label, _DEFAULT_COLOR_BGR)


# ---------------------------------------------------------------------------
# Service call
# ---------------------------------------------------------------------------

def detect_via_service(
    bgr: np.ndarray,
    service_url: str,
    request_id: int,
    timeout_s: float = 5.0,
) -> tuple[list[dict], np.ndarray | None, int]:
    """POST one frame to /detect. Returns (detections, mask, inference_ms)."""
    pil = Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
    buf = io.BytesIO()
    pil.save(buf, format="JPEG", quality=90)
    b64 = base64.b64encode(buf.getvalue()).decode("ascii")

    payload = json.dumps({"image_jpeg_b64": b64, "request_id": request_id}).encode()
    req = Request(
        f"{service_url}/detect",
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urlopen(req, timeout=timeout_s) as resp:
            result = json.loads(resp.read())
    except HTTPError as e:
        body = e.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"HTTP {e.code} from service: {body}") from e
    except URLError as e:
        raise RuntimeError(
            f"Cannot reach service at {service_url} ({e.reason}). "
            "Is semantic_service\\run_semantic.bat running?"
        ) from e

    mask_np: np.ndarray | None = None
    if result.get("mask_png_b64"):
        mask_bytes = base64.b64decode(result["mask_png_b64"])
        mask_pil = Image.open(io.BytesIO(mask_bytes)).convert("L")
        mask_np = np.array(mask_pil, dtype=np.uint8)

    return result["detections"], mask_np, int(result["inference_ms"])


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------

def annotate(
    bgr: np.ndarray,
    detections: list[dict[str, Any]],
    mask_np: np.ndarray | None,
    inference_ms: int,
    display_fps: float,
    request_id: int,
    paused: bool,
) -> np.ndarray:
    out = bgr.copy()
    h, w = out.shape[:2]

    # Mask overlay: greenish tint where any instance was segmented.
    if mask_np is not None and mask_np.any():
        sel = mask_np > 0
        overlay = out.copy()
        overlay[sel] = (
            overlay[sel].astype(np.uint16) * np.array([1, 3, 1], np.uint16) // 4
        ).astype(np.uint8)
        overlay[sel, 1] = np.maximum(overlay[sel, 1], 140)
        cv2.addWeighted(overlay, 0.55, out, 0.45, 0, out)

    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = max(0.4, w / 1920.0)
    thick = max(1, int(scale * 2))

    for det in detections:
        x1, y1, x2, y2 = (int(v) for v in det["box"])
        label = det["label"]
        conf = det["confidence"]
        col = _color(label)

        cv2.rectangle(out, (x1, y1), (x2, y2), col, thick + 1)

        txt = f"{label} {conf:.2f}"
        (tw, th), _ = cv2.getTextSize(txt, font, scale, thick)
        ty = max(y1, th + 4)
        cv2.rectangle(out, (x1, ty - th - 4), (x1 + tw + 4, ty), col, -1)
        cv2.putText(out, txt, (x1 + 2, ty - 2), font, scale,
                    (255, 255, 255), thick, cv2.LINE_AA)

    # Status bar (top-left)
    pause_tag = "  [PAUSED]" if paused else ""
    status = (
        f"req:{request_id}  det:{len(detections)}  "
        f"infer:{inference_ms}ms  disp:{display_fps:.1f}fps  "
        f"{w}x{h}{pause_tag}"
    )
    (sw, sh), _ = cv2.getTextSize(status, font, scale, thick)
    cv2.rectangle(out, (4, 4), (sw + 12, sh + 12), (0, 0, 0), -1)
    cv2.putText(out, status, (8, sh + 8), font, scale,
                (0, 240, 0), thick, cv2.LINE_AA)

    return out


# ---------------------------------------------------------------------------
# Monitor mode — poll /debug_image + /debug_json
# ---------------------------------------------------------------------------

def _http_get(url: str, timeout_s: float) -> tuple[int, bytes]:
    req = Request(url, method="GET")
    try:
        with urlopen(req, timeout=timeout_s) as resp:
            return resp.status, resp.read()
    except HTTPError as e:
        return e.code, e.read() if hasattr(e, "read") else b""
    except URLError as e:
        raise RuntimeError(f"Cannot reach {url} ({e.reason})") from e


def run_monitor(args: argparse.Namespace) -> int:
    """Poll /debug_image and /debug_json. Shows whatever the service most
    recently processed — independent of who pushed it (RTAB / sender / etc)."""
    base = args.service.rstrip("/")
    win = f"semantic monitor - {base}  [s=save  q=quit]"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    last_req_id: Any = object()  # sentinel: definitely != any first response
    disp_times: deque[float] = deque(maxlen=30)
    fresh_times: deque[float] = deque(maxlen=30)
    prev_t = time.perf_counter()
    last_vis: np.ndarray | None = None
    waiting_logged = False
    waiting_since = time.perf_counter()

    while True:
        loop_t0 = time.perf_counter()

        try:
            status_meta, body_meta = _http_get(f"{base}/debug_json",
                                                timeout_s=args.timeout)
            status_img, body_img = _http_get(f"{base}/debug_image",
                                              timeout_s=args.timeout)
        except RuntimeError as e:
            now = time.perf_counter()
            if now - waiting_since > 2.0:
                print(f"[test_semantic] {e}  (is semantic_service running?)")
                waiting_since = now
            if cv2.waitKey(500) & 0xFF in (ord('q'), 27):
                break
            continue

        if status_meta == 404 or status_img == 404:
            now = time.perf_counter()
            if not waiting_logged or now - waiting_since > 3.0:
                print("[test_semantic] service is up but no frame processed yet "
                      "— is RTAB-Map running and pushing keyframes?")
                waiting_logged = True
                waiting_since = now
            if cv2.waitKey(200) & 0xFF in (ord('q'), 27):
                break
            continue
        if status_meta != 200 or status_img != 200:
            print(f"[test_semantic] HTTP error meta={status_meta} img={status_img}")
            if cv2.waitKey(500) & 0xFF in (ord('q'), 27):
                break
            continue

        meta = json.loads(body_meta)
        np_jpeg = np.frombuffer(body_img, dtype=np.uint8)
        bgr = cv2.imdecode(np_jpeg, cv2.IMREAD_COLOR)
        if bgr is None:
            print("[test_semantic] failed to decode debug_image JPEG")
            continue

        req_id = meta.get("request_id")
        is_fresh = req_id != last_req_id
        if is_fresh:
            fresh_times.append(loop_t0)
            last_req_id = req_id

        now = time.perf_counter()
        disp_times.append(now - prev_t)
        prev_t = now
        disp_fps = (len(disp_times) / sum(disp_times)) if disp_times else 0.0
        # Service-side processing rate = how often req_id changes
        if len(fresh_times) >= 2:
            svc_fps = (len(fresh_times) - 1) / (fresh_times[-1] - fresh_times[0])
        else:
            svc_fps = 0.0

        h, w = bgr.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = max(0.4, w / 1920.0)
        thick = max(1, int(scale * 2))
        n_det = len(meta.get("detections", []))
        infer_ms = int(meta.get("inference_ms", 0))
        bar = (
            f"MONITOR  req:{req_id}  det:{n_det}  "
            f"infer:{infer_ms}ms  svc:{svc_fps:.1f}fps  "
            f"poll:{disp_fps:.1f}fps  {w}x{h}"
        )
        (sw, sh), _ = cv2.getTextSize(bar, font, scale, thick)
        # Service-side already drew its own status; put ours below to avoid overlap.
        y_top = 24
        cv2.rectangle(bgr, (4, y_top), (sw + 12, y_top + sh + 8), (0, 0, 0), -1)
        cv2.putText(bgr, bar, (8, y_top + sh + 4),
                    font, scale, (255, 220, 80), thick, cv2.LINE_AA)

        last_vis = bgr
        cv2.imshow(win, bgr)

        # Throttle to args.poll_hz so we don't spam the service.
        target_dt = 1.0 / max(1.0, args.poll_hz)
        elapsed = time.perf_counter() - loop_t0
        wait_ms = max(1, int((target_dt - elapsed) * 1000))
        key = cv2.waitKey(wait_ms) & 0xFF
        if key in (ord('q'), 27):
            break
        if key == ord('s') and last_vis is not None:
            _save(last_vis, args.save_dir, "monitor", req_id if isinstance(req_id, int) else 0)

    cv2.destroyAllWindows()
    return 0


# ---------------------------------------------------------------------------
# Capture sources
# ---------------------------------------------------------------------------

def open_capture(args: argparse.Namespace) -> tuple[cv2.VideoCapture | None, str]:
    """Return (cap, caption_prefix). cap is None for single-image mode."""
    if args.camera is not None:
        cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW if hasattr(cv2, "CAP_DSHOW") else 0)
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open camera index {args.camera}")
        # Try to coax 720p.
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        return cap, f"camera{args.camera}"

    path = Path(args.input)
    if not path.exists():
        raise RuntimeError(f"file not found: {path}")

    image_exts = {".jpg", ".jpeg", ".png", ".bmp", ".webp", ".tiff"}
    if path.suffix.lower() in image_exts:
        return None, path.name

    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise RuntimeError(f"cannot open video: {path}")
    return cap, path.name


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def run_single_image(args: argparse.Namespace, caption: str) -> int:
    bgr = cv2.imread(args.input)
    if bgr is None:
        print(f"[test_semantic] failed to read {args.input}")
        return 1

    print(f"[test_semantic] sending {caption} ({bgr.shape[1]}x{bgr.shape[0]}) ...")
    try:
        dets, mask, ms = detect_via_service(bgr, args.service, request_id=0)
    except Exception as e:
        print(f"[test_semantic] FAILED: {e}")
        return 1

    print(f"  inference_ms={ms}  detections={len(dets)}")
    for d in dets:
        print(f"    {d['label']:14s}  conf={d['confidence']:.2f}  "
              f"box={[int(v) for v in d['box']]}")

    vis = annotate(bgr, dets, mask, ms, 0.0, 0, paused=False)
    win = f"semantic test - {caption}  [s=save  q=quit]"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.imshow(win, vis)
    while True:
        key = cv2.waitKey(0) & 0xFF
        if key in (ord('q'), 27):
            break
        if key == ord('s'):
            _save(vis, args.save_dir, caption, 0)
    cv2.destroyAllWindows()
    return 0


def run_stream(
    cap: cv2.VideoCapture,
    args: argparse.Namespace,
    caption_prefix: str,
) -> int:
    is_camera = args.camera is not None
    win = f"semantic test - {caption_prefix}  [SPACE=pause  s=save  q=quit]"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    paused = False
    request_id = 0
    last_vis: np.ndarray | None = None
    last_frame: tuple[np.ndarray, list[dict], np.ndarray | None, int] | None = None
    disp_times: deque[float] = deque(maxlen=30)
    prev_t = time.perf_counter()
    last_fail_print = 0.0

    while True:
        if not paused:
            for _ in range(max(1, args.stride)):
                ok, bgr = cap.read()
                if not ok:
                    break
            if not ok:
                if is_camera:
                    print("[test_semantic] camera read failed; exiting")
                    break
                print("[test_semantic] end of video")
                break

            try:
                dets, mask, ms = detect_via_service(
                    bgr, args.service, request_id=request_id,
                    timeout_s=args.timeout,
                )
            except Exception as e:
                now = time.perf_counter()
                if now - last_fail_print > 2.0:
                    print(f"[test_semantic] /detect failed: {e}")
                    last_fail_print = now
                # Show raw frame so the stream doesn't freeze visually.
                dets, mask, ms = [], None, 0

            last_frame = (bgr, dets, mask, ms)
            request_id += 1

        if last_frame is None:
            # Camera produced nothing yet — wait briefly and retry.
            if cv2.waitKey(10) & 0xFF in (ord('q'), 27):
                break
            continue

        now = time.perf_counter()
        disp_times.append(now - prev_t)
        prev_t = now
        disp_fps = (len(disp_times) / sum(disp_times)) if disp_times else 0.0

        bgr, dets, mask, ms = last_frame
        vis = annotate(bgr, dets, mask, ms, disp_fps, request_id - 1, paused)
        last_vis = vis
        cv2.imshow(win, vis)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break
        if key == ord(' '):
            paused = not paused
            print(f"[test_semantic] {'paused' if paused else 'resumed'}")
        elif key == ord('s') and last_vis is not None:
            _save(last_vis, args.save_dir, caption_prefix, request_id - 1)

    cap.release()
    cv2.destroyAllWindows()
    return 0


def _save(vis: np.ndarray, save_dir: str, caption: str, idx: int) -> None:
    out_dir = Path(save_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = Path(caption).stem or caption
    out_path = out_dir / f"{stem}_{idx:05d}.jpg"
    cv2.imwrite(str(out_path), vis)
    print(f"[test_semantic] saved -> {out_path}")


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Live visualizer for the semantic_service /detect endpoint."
    )
    ap.add_argument("input", nargs="?", help="image or video file path (switches to sender mode)")
    ap.add_argument("--camera", type=int, default=None,
                    help="webcam index (switches to sender mode, e.g. --camera 0)")
    ap.add_argument("--poll-hz", type=float, default=20.0,
                    help="monitor mode polling rate (default: 20)")
    ap.add_argument("--service", default="http://127.0.0.1:7788",
                    help="semantic_service base URL")
    ap.add_argument("--stride", type=int, default=1,
                    help="advance the capture by N frames per inference (1 = every frame)")
    ap.add_argument("--timeout", type=float, default=5.0,
                    help="per-request timeout seconds")
    ap.add_argument("--save-dir", default="test_output",
                    help="directory for saved frames (s key)")
    args = ap.parse_args()

    if args.camera is None and not args.input:
        return run_monitor(args)

    try:
        cap, caption_prefix = open_capture(args)
    except Exception as e:
        print(f"[test_semantic] {e}")
        return 1

    if cap is None:
        return run_single_image(args, caption_prefix)
    return run_stream(cap, args, caption_prefix)


if __name__ == "__main__":
    import sys
    sys.exit(main())
