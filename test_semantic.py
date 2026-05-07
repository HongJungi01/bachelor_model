"""
test_semantic.py — smoke-test the semantic detection pipeline.

Sends frames to the running semantic_service (/detect endpoint) and renders
annotated results with OpenCV. Alternatively, calls the LLM directly via
--direct mode (no FastAPI required, but needs API key in env).

Usage:
    # image
    python test_semantic.py path/to/image.jpg

    # video (sample every 30 frames by default)
    python test_semantic.py path/to/video.mp4

    # video — custom sampling interval
    python test_semantic.py video.mp4 --interval 60

    # point at a non-default service URL
    python test_semantic.py image.jpg --service http://127.0.0.1:7788

    # skip FastAPI — call LLM directly (API key must be set in env, no SAM)
    python test_semantic.py image.jpg --direct

OpenCV window controls:
    SPACE / RIGHT  next frame
    LEFT           previous frame
    s              save annotated frame to ./test_output/<original_name>_NNN.jpg
    q / ESC        quit
"""

from __future__ import annotations

import argparse
import base64
import io
import json
import math
import os
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from PIL import Image


# ---------------------------------------------------------------------------
# Palette (BGR) — matches grid_client.py colors for consistency
# ---------------------------------------------------------------------------
_LABEL_COLOR_BGR: dict[str, tuple[int, int, int]] = {
    "parking_line":       ( 40,  40, 220),  # red
    "lane_divider":       ( 40, 180, 220),  # orange-red
    "pillar":             ( 40,  40, 220),  # red
    "traffic_cone":       (  0, 140, 255),  # orange
    "no_entry_sign":      (  0,   0, 200),  # dark red
    "construction_sign":  (  0, 200, 255),  # yellow
    "exit_area":          ( 60, 200,  60),  # green
    "exit_sign":          (220, 200,  60),  # cyan-teal
    "floor_arrow":        (220, 200,  60),  # cyan-teal
    "one_way_marker":     (200,  60, 200),  # magenta
}
_DEFAULT_COLOR_BGR = (180, 180, 180)


def _color(label: str) -> tuple[int, int, int]:
    return _LABEL_COLOR_BGR.get(label, _DEFAULT_COLOR_BGR)


# ---------------------------------------------------------------------------
# Annotation
# ---------------------------------------------------------------------------

def annotate(
    bgr: np.ndarray,
    detections: list[dict[str, Any]],
    mask_np: np.ndarray | None,
    infer_ms: int,
    timing: dict[str, int] | None = None,
) -> np.ndarray:
    out = bgr.copy()

    # Semi-transparent green mask overlay (30% green tint)
    if mask_np is not None and mask_np.any():
        sel = mask_np > 0
        overlay = out.copy()
        overlay[sel] = (overlay[sel].astype(np.uint16) * np.array([1, 3, 1], np.uint16) // 4).astype(np.uint8)
        overlay[sel, 1] = np.maximum(overlay[sel, 1], 120)
        cv2.addWeighted(overlay, 0.45, out, 0.55, 0, out)

    h, w = out.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = max(0.35, w / 1920.0)
    thick = max(1, int(scale * 2))

    for det in detections:
        x1, y1, x2, y2 = (int(v) for v in det["box"])
        label = det["label"]
        conf = det["confidence"]
        va = det.get("visual_angle")
        col = _color(label)

        cv2.rectangle(out, (x1, y1), (x2, y2), col, thick + 1)

        txt = f"{label} {conf:.2f}"
        if va is not None:
            txt += f" @{va:.0f}deg"

        (tw, th), _ = cv2.getTextSize(txt, font, scale, thick)
        ty = max(y1, th + 4)
        cv2.rectangle(out, (x1, ty - th - 4), (x1 + tw + 4, ty), col, -1)
        cv2.putText(out, txt, (x1 + 2, ty - 2), font, scale, (255, 255, 255), thick,
                    cv2.LINE_AA)

        # Angle arrow inside the box
        if va is not None:
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            length = max(10.0, min(x2 - x1, y2 - y1) * 0.38)
            theta = math.radians(va)
            dx = math.sin(theta) * length
            dy = -math.cos(theta) * length
            cv2.arrowedLine(
                out,
                (int(cx), int(cy)),
                (int(cx + dx), int(cy + dy)),
                col, thick + 1, tipLength=0.35,
            )

    # Status bar
    if timing:
        llm_ms = timing.get("llm_ms", 0)
        sam_ms = timing.get("sam_ms", 0)
        status = (f"det:{len(detections)}  "
                  f"total:{infer_ms/1000:.1f}s  llm:{llm_ms/1000:.1f}s  sam:{sam_ms/1000:.2f}s  "
                  f"{w}x{h}")
    else:
        status = f"det:{len(detections)}  total:{infer_ms/1000:.1f}s  {w}x{h}"
    (sw, sh), _ = cv2.getTextSize(status, font, scale, thick)
    cv2.rectangle(out, (4, 4), (sw + 12, sh + 12), (0, 0, 0), -1)
    cv2.putText(out, status, (8, sh + 8), font, scale, (0, 240, 0), thick, cv2.LINE_AA)

    return out


# ---------------------------------------------------------------------------
# HTTP mode (requires service running)
# ---------------------------------------------------------------------------

def detect_via_service(
    pil: Image.Image,
    service_url: str,
    request_id: int = 0,
) -> tuple[list[dict], np.ndarray | None, int]:
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
        with urlopen(req, timeout=60) as resp:
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

    timing = {
        "total_ms":  result["inference_ms"],
        "llm_ms":    result.get("llm_ms", 0),
        "sam_ms":    result.get("sam_ms", 0),
    }
    return result["detections"], mask_np, timing


# ---------------------------------------------------------------------------
# Direct mode (no FastAPI — LLM only, no SAM)
# ---------------------------------------------------------------------------

def _inject_venv() -> None:
    """Add semantic_service venv site-packages so pydantic/google-genai/etc. are importable."""
    venv_root = Path(__file__).parent / "semantic_service" / ".venv"
    candidates = [
        venv_root / "Lib" / "site-packages",          # Windows
        venv_root / "lib" / "site-packages",           # Linux fallback (no version dir)
    ]
    # Also search lib/python3.x/site-packages on Linux
    lib = venv_root / "lib"
    if lib.is_dir():
        for sub in lib.iterdir():
            candidates.append(sub / "site-packages")

    for sp in candidates:
        if sp.exists() and str(sp) not in sys.path:
            sys.path.insert(0, str(sp))
            return

    print(
        "[test_semantic] WARNING: semantic_service/.venv not found.\n"
        "  --direct mode needs pydantic + LLM SDK in the current Python env.\n"
        "  Option A (recommended): start the service and drop --direct\n"
        "    > semantic_service\\run_semantic.bat   (new terminal)\n"
        "    > python test_semantic.py <image>\n"
        "  Option B: install deps here\n"
        "    > pip install pydantic annotated_types pillow google-genai\n"
        "      (replace google-genai with anthropic if LLM_PROVIDER=anthropic)"
    )


def detect_direct(pil: Image.Image) -> tuple[list[dict], None, int]:
    _inject_venv()
    sys.path.insert(0, str(Path(__file__).parent / "semantic_service"))
    from llm_runner import LlmRunner  # type: ignore

    runner = LlmRunner()
    t0 = time.perf_counter()
    result = runner.detect(pil)
    infer_ms = int((time.perf_counter() - t0) * 1000)

    detections = [
        {
            "box": d.box,
            "label": d.label,
            "confidence": d.confidence,
            "visual_angle": d.visual_angle,
        }
        for d in result.detections
    ]
    timing = {"total_ms": infer_ms, "llm_ms": infer_ms, "sam_ms": 0}
    return detections, None, timing


# ---------------------------------------------------------------------------
# Frame extraction
# ---------------------------------------------------------------------------

def load_frames(path: Path, interval: int) -> list[tuple[Image.Image, str]]:
    """Return (PIL image, caption) pairs from an image or video file."""
    suffix = path.suffix.lower()
    image_exts = {".jpg", ".jpeg", ".png", ".bmp", ".webp", ".tiff"}

    if suffix in image_exts:
        pil = Image.open(path).convert("RGB")
        return [(pil, path.name)]

    # Video
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise ValueError(f"Cannot open video: {path}")

    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    frames: list[tuple[Image.Image, str]] = []

    idx = 0
    while True:
        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ok, bgr = cap.read()
        if not ok:
            break
        pil = Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        ts = idx / fps
        caption = f"{path.name}  frame {idx}  ({ts:.1f}s)"
        frames.append((pil, caption))
        idx += interval
        if idx >= total:
            break

    cap.release()
    print(f"[test_semantic] extracted {len(frames)} frames "
          f"(every {interval} frames, total {total})")
    return frames


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(
        description="Test semantic detection on an image or video."
    )
    ap.add_argument("input", help="Path to image or video file")
    ap.add_argument(
        "--service", default="http://127.0.0.1:7788",
        help="Semantic service base URL (default: http://127.0.0.1:7788)"
    )
    ap.add_argument(
        "--direct", action="store_true",
        help="Call LLM directly via llm_runner.py (no service, no SAM)"
    )
    ap.add_argument(
        "--interval", type=int, default=30,
        help="Frame sampling interval for video (default: 30)"
    )
    ap.add_argument(
        "--save-dir", default="test_output",
        help="Directory to save annotated images (default: test_output)"
    )
    args = ap.parse_args()

    input_path = Path(args.input)
    if not input_path.exists():
        print(f"[test_semantic] ERROR: file not found: {input_path}")
        return 1

    print(f"[test_semantic] loading frames from {input_path} ...")
    frames = load_frames(input_path, args.interval)
    if not frames:
        print("[test_semantic] no frames extracted")
        return 1

    save_dir = Path(args.save_dir)
    annotated_frames: list[np.ndarray] = []
    results_log: list[dict] = []

    print(f"[test_semantic] processing {len(frames)} frame(s) ...")
    for i, (pil, caption) in enumerate(frames):
        print(f"  [{i+1}/{len(frames)}] {caption} ...", end=" ", flush=True)
        try:
            if args.direct:
                detections, mask_np, timing = detect_direct(pil)
            else:
                detections, mask_np, timing = detect_via_service(
                    pil, args.service, request_id=i
                )
        except Exception as e:
            print(f"FAILED ({e})")
            if not args.direct:
                print("  -> Is the service running? Try: semantic_service\\run_semantic.bat")
            continue

        total_ms = timing["total_ms"]
        llm_ms   = timing["llm_ms"]
        sam_ms   = timing["sam_ms"]
        other_ms = max(0, total_ms - llm_ms - sam_ms)
        print(
            f"total={total_ms}ms  "
            f"[llm={llm_ms}ms  sam={sam_ms}ms  other={other_ms}ms]  "
            f"{len(detections)} detection(s)"
        )
        for det in detections:
            va = det.get("visual_angle")
            angle_str = f"  visual_angle={va:.1f}°" if va is not None else ""
            print(f"      {det['label']:20s}  conf={det['confidence']:.2f}"
                  f"  box={[int(v) for v in det['box']]}{angle_str}")

        results_log.append({
            "frame": i,
            "caption": caption,
            "timing": timing,
            "detections": detections,
        })

        bgr = cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)
        vis = annotate(bgr, detections, mask_np, total_ms, timing)
        cv2.putText(vis, caption, (8, vis.shape[0] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1, cv2.LINE_AA)
        annotated_frames.append(vis)

    if not annotated_frames:
        print("[test_semantic] no successful results")
        return 1

    # Save JSON summary
    summary_path = Path(f"test_output_{input_path.stem}.json")
    summary_path.write_text(json.dumps(results_log, indent=2, ensure_ascii=False),
                            encoding="utf-8")
    print(f"\n[test_semantic] JSON summary → {summary_path}")

    # Interactive OpenCV viewer
    win = f"semantic test — {input_path.name}  [SPACE=next  LEFT=prev  s=save  q=quit]"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    idx = 0
    while True:
        img = annotated_frames[idx]
        cv2.imshow(win, img)
        key = cv2.waitKey(0) & 0xFF

        if key in (ord('q'), 27):
            break
        elif key in (ord(' '), 83, 3):   # SPACE, right arrow
            idx = min(idx + 1, len(annotated_frames) - 1)
        elif key in (81, 2):              # left arrow
            idx = max(idx - 1, 0)
        elif key == ord('s'):
            save_dir.mkdir(parents=True, exist_ok=True)
            stem = input_path.stem
            out_path = save_dir / f"{stem}_{idx:04d}.jpg"
            cv2.imwrite(str(out_path), img)
            print(f"[test_semantic] saved → {out_path}")

    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
