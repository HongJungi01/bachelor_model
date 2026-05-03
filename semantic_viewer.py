"""
semantic_viewer.py - Live view of semantic_service detections.

Polls GET http://127.0.0.1:7788/debug_image every ~0.5 s and shows the
annotated frame (boxes + SAM mask overlay drawn server-side) in an OpenCV
window. Detection summary is fetched from /debug_json and overlaid locally.

Usage:
    pip install opencv-python requests
    python semantic_viewer.py          (q or ESC to quit)
"""

from __future__ import annotations

import sys
import time

import cv2
import numpy as np
import requests

BASE = "http://127.0.0.1:7788"
POLL_S = 0.5
WIN = "semantic_service detections  [q / ESC = quit]"


def fetch_image() -> np.ndarray | None:
    try:
        r = requests.get(f"{BASE}/debug_image", timeout=2.0)
        if r.status_code == 200:
            arr = np.frombuffer(r.content, dtype=np.uint8)
            return cv2.imdecode(arr, cv2.IMREAD_COLOR)
    except Exception:
        pass
    return None


def fetch_meta() -> dict | None:
    try:
        r = requests.get(f"{BASE}/debug_json", timeout=2.0)
        if r.status_code == 200:
            return r.json()
    except Exception:
        pass
    return None


def wait_for_server(timeout: float = 60.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            r = requests.get(f"{BASE}/healthz", timeout=2.0)
            if r.status_code == 200:
                d = r.json()
                if d.get("model_loaded"):
                    print(f"[semantic_viewer] connected (device={d.get('device')})")
                    return True
                print("[semantic_viewer] server up, model still loading...")
        except Exception:
            pass
        time.sleep(1.0)
        print("[semantic_viewer] waiting for semantic_service...")
    return False


def draw_info(img: np.ndarray, meta: dict | None, frame_count: int) -> None:
    if meta:
        dets = meta.get("detections", [])
        lines = [
            f"frame {frame_count}  req={meta.get('request_id')}",
            f"{meta.get('inference_ms')} ms  "
            f"{meta.get('width')}x{meta.get('height')}",
            f"{len(dets)} detection(s)",
        ]
        for d in dets[:6]:
            lines.append(f"  [{d['label']}]  {d['score']:.3f}")
        if len(dets) > 6:
            lines.append(f"  ... +{len(dets) - 6} more")
    else:
        lines = [f"frame {frame_count}"]

    font, scale, thick = cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1
    pad, lh = 6, 16
    box_w = max(cv2.getTextSize(l, font, scale, thick)[0][0] for l in lines) + pad * 2
    box_h = lh * len(lines) + pad * 2
    overlay = img.copy()
    cv2.rectangle(overlay, (4, 4), (4 + box_w, 4 + box_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, img, 0.4, 0, img)
    for i, line in enumerate(lines):
        cv2.putText(img, line, (4 + pad, 4 + pad + lh * (i + 1) - 3),
                    font, scale, (255, 255, 255), thick, cv2.LINE_AA)


def main() -> int:
    print(f"[semantic_viewer] connecting to {BASE} ...")
    if not wait_for_server():
        print("[semantic_viewer] timed out waiting for server")
        return 1

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)

    frame_count = 0
    last_req_id: int | None = None
    placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(placeholder, "waiting for first detection...",
                (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 1)
    cv2.imshow(WIN, placeholder)

    try:
        while True:
            frame = fetch_image()
            meta = fetch_meta()

            if frame is not None:
                req_id = (meta or {}).get("request_id")
                if req_id != last_req_id:
                    last_req_id = req_id
                    frame_count += 1

                draw_info(frame, meta, frame_count)
                cv2.imshow(WIN, frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break

            time.sleep(POLL_S)

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()

    print("[semantic_viewer] exiting")
    return 0


if __name__ == "__main__":
    sys.exit(main())
