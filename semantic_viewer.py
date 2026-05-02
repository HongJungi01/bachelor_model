"""
semantic_viewer.py - Live view of GroundingDINO detections from semantic_service.

Polls GET http://127.0.0.1:7788/debug_image every ~0.5 s and renders the
annotated frame (boxes + labels drawn server-side) in a matplotlib window.
A text overlay shows the detection summary from /debug_json.

Usage:
    pip install matplotlib pillow requests numpy
    python semantic_viewer.py

Keyboard shortcuts (matplotlib window must be focused):
    q   quit
"""

from __future__ import annotations

import io
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
import requests
from PIL import Image

BASE = "http://127.0.0.1:7788"
POLL_S = 0.5


def fetch_image() -> np.ndarray | None:
    try:
        r = requests.get(f"{BASE}/debug_image", timeout=2.0)
        if r.status_code == 200:
            return np.array(Image.open(io.BytesIO(r.content)).convert("RGB"))
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
                print("[semantic_viewer] server up but model still loading...")
        except Exception:
            pass
        time.sleep(1.0)
        print("[semantic_viewer] waiting for semantic_service...")
    return False


def main() -> int:
    print(f"[semantic_viewer] connecting to {BASE} ...")
    if not wait_for_server():
        print("[semantic_viewer] timed out waiting for server")
        return 1

    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 7))
    ax.axis("off")
    fig.canvas.manager.set_window_title("Semantic detections (GroundingDINO)")

    img_artist = None
    text_artist = None
    last_req_id: int | None = None
    frame_count = 0

    def on_key(event):
        if event.key == "q":
            plt.close("all")

    fig.canvas.mpl_connect("key_press_event", on_key)

    try:
        while plt.get_fignums():
            frame = fetch_image()
            meta = fetch_meta()

            if frame is not None:
                req_id = (meta or {}).get("request_id")
                if req_id != last_req_id:
                    last_req_id = req_id
                    frame_count += 1

                    if img_artist is None:
                        img_artist = ax.imshow(frame)
                    else:
                        img_artist.set_data(frame)
                        img_artist.set_extent(
                            [-0.5, frame.shape[1] - 0.5,
                             frame.shape[0] - 0.5, -0.5]
                        )
                        ax.set_xlim(-0.5, frame.shape[1] - 0.5)
                        ax.set_ylim(frame.shape[0] - 0.5, -0.5)

                    if text_artist is not None:
                        text_artist.remove()

                    if meta:
                        dets = meta.get("detections", [])
                        lines = [
                            f"frame {frame_count}  req={meta.get('request_id')}",
                            f"{meta.get('inference_ms')} ms  "
                            f"{meta.get('width')}×{meta.get('height')}",
                            f"{len(dets)} detection(s)",
                        ]
                        for d in dets[:6]:
                            lines.append(
                                f"  [{d['label']}]  score={d['score']:.3f}"
                            )
                        if len(dets) > 6:
                            lines.append(f"  ... +{len(dets) - 6} more")
                        info_text = "\n".join(lines)
                    else:
                        info_text = f"frame {frame_count}"

                    text_artist = ax.text(
                        0.01, 0.99, info_text,
                        transform=ax.transAxes, va="top", ha="left",
                        fontsize=9, family="monospace",
                        bbox=dict(
                            boxstyle="round,pad=0.3",
                            fc="black", ec="gray", alpha=0.7,
                        ),
                        color="white",
                    )

                plt.pause(0.001)

            time.sleep(POLL_S)

    except KeyboardInterrupt:
        pass

    print("[semantic_viewer] exiting")
    return 0


if __name__ == "__main__":
    sys.exit(main())
