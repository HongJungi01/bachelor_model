"""
Quick smoke-test: encode any local image as base64, POST to /detect,
print the result, and open the annotated debug image in your default viewer.

Usage:
    python test_detect.py [path/to/image.jpg]
    python test_detect.py          <- uses a generated synthetic test image
"""
from __future__ import annotations
import base64, io, sys, webbrowser, tempfile, os
import requests
from PIL import Image, ImageDraw

BASE = "http://127.0.0.1:7788"


def make_synthetic_image() -> bytes:
    """Create a fake parking-lot image with white lines on gray."""
    img = Image.new("RGB", (640, 480), color=(80, 80, 80))
    draw = ImageDraw.Draw(img)
    for x in range(80, 600, 120):
        draw.rectangle([x, 60, x + 8, 420], fill=(230, 230, 230))
    draw.rectangle([60, 400, 580, 420], fill=(230, 230, 230))
    draw.rectangle([60, 60, 580, 80], fill=(230, 230, 230))
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=90)
    return buf.getvalue()


def main():
    if len(sys.argv) > 1:
        path = sys.argv[1]
        raw = open(path, "rb").read()
        print(f"[test_detect] using image: {path}")
    else:
        raw = make_synthetic_image()
        print("[test_detect] using synthetic parking-lot image")

    b64 = base64.b64encode(raw).decode()

    print(f"[test_detect] POSTing to {BASE}/detect ...")
    r = requests.post(f"{BASE}/detect", json={
        "image_jpeg_b64": b64,
        "prompt": "parking space line . lane marking",
        "box_threshold": 0.25,
        "text_threshold": 0.20,
    }, timeout=30)

    if r.status_code != 200:
        print(f"[test_detect] ERROR {r.status_code}: {r.text}")
        return

    data = r.json()
    print(f"[test_detect] inference_ms={data['inference_ms']}")
    print(f"[test_detect] detections={len(data['detections'])}")
    for d in data["detections"]:
        print(f"  [{d['label']}]  score={d['score']:.3f}  box={[round(v) for v in d['box']]}")
    mb = data.get("mask_png_b64")
    print(f"[test_detect] mask_png_b64={'present (' + str(len(mb)) + ' chars)' if mb else 'absent'}")

    # Fetch annotated debug image and open it
    dbg = requests.get(f"{BASE}/debug_image", timeout=5)
    if dbg.status_code == 200:
        tmp = tempfile.NamedTemporaryFile(suffix=".jpg", delete=False)
        tmp.write(dbg.content)
        tmp.close()
        print(f"[test_detect] debug image saved to {tmp.name}")
        webbrowser.open(tmp.name)
    else:
        print("[test_detect] could not fetch debug image")


if __name__ == "__main__":
    main()
