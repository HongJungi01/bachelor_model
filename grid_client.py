"""
grid_client.py - Live visualizer for the RTABMap GUI's TCP grid stream.

Connects to 127.0.0.1:7777 and renders the occupancy grid + robot pose
in real time. Semantic-tagged cells are drawn so YOLO-detected
centerLine / parkingLine surfaces appear layered on the structural map.

Cell value legend (matches SemanticMaskStore.h):
    -1     unknown          -> gray   (128, 128, 128)
     0     free             -> white  (255, 255, 255)
     1     wall-like        -> red    BGR ( 40,  40, 220)
                               centerLine / parkingLine
   100     obstacle (RTABMap structural) -> black (  0,   0,   0)

Usage:
    pip install opencv-python numpy
    python grid_client.py                 # default: 2 px per cell
    python grid_client.py --scale 4       # zoom in
    python grid_client.py --scale 1       # shrink (large maps)
    (q or ESC to quit)
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import sys
import time
from dataclasses import dataclass

import cv2
import numpy as np

HOST = "127.0.0.1"
PORT = 7777
HEADER_FMT = "<iiffffff"       # width, height, xMin, yMin, cellSize, poseX, poseY, poseYaw
HEADER_SIZE = struct.calcsize(HEADER_FMT)  # = 32

WIN = "RTABMap occupancy grid  [q / ESC = quit]"


@dataclass
class GridFrame:
    width: int
    height: int
    x_min: float
    y_min: float
    cell_size: float
    pose_x: float
    pose_y: float
    pose_yaw: float
    grid: np.ndarray   # shape (H, W), int8


def recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("server closed connection")
        buf.extend(chunk)
    return bytes(buf)


def read_frame(sock: socket.socket) -> GridFrame:
    hdr = recv_exact(sock, HEADER_SIZE)
    w, h, xmin, ymin, cs, px, py, yaw = struct.unpack(HEADER_FMT, hdr)
    body = recv_exact(sock, w * h)
    grid = np.frombuffer(body, dtype=np.int8).reshape(h, w)
    return GridFrame(w, h, xmin, ymin, cs, px, py, yaw, grid)


def grid_to_bgr(grid: np.ndarray) -> np.ndarray:
    bgr = np.full((*grid.shape, 3), 128, dtype=np.uint8)  # unknown = gray
    bgr[grid == 0]   = (255, 255, 255)   # free
    bgr[grid == 1]   = ( 40,  40, 220)   # wall-like semantic (red in BGR)
    bgr[grid == 100] = (  0,   0,   0)   # obstacle
    return bgr


def world_to_px(wx: float, wy: float, f: GridFrame) -> tuple[int, int]:
    """World coord (m) → pixel (col, row) in the displayed image."""
    col = int((wx - f.x_min) / f.cell_size)
    row = f.height - 1 - int((wy - f.y_min) / f.cell_size)  # y-flip
    return col, row


def draw_overlay(img: np.ndarray, f: GridFrame, n_sem: int, n_obs: int,
                 frame_count: int, scale: int) -> None:
    arrow_m = 0.8  # arrow length in metres
    tip_x = f.pose_x + arrow_m * math.cos(f.pose_yaw)
    tip_y = f.pose_y + arrow_m * math.sin(f.pose_yaw)
    bx, by = world_to_px(f.pose_x, f.pose_y, f)
    tx, ty = world_to_px(tip_x,    tip_y,    f)
    base_px = (bx * scale, by * scale)
    tip_px  = (tx * scale, ty * scale)

    cv2.circle(img, base_px, max(3, 2 * scale), (0, 0, 255), -1)
    cv2.arrowedLine(img, base_px, tip_px, (0, 0, 255),
                    max(2, scale), tipLength=0.4)

    lines = [
        f"frame {frame_count}",
        f"pose ({f.pose_x:+.2f}, {f.pose_y:+.2f})  yaw {math.degrees(f.pose_yaw):+.1f}deg",
        f"grid {f.width}x{f.height}  cell {f.cell_size:.3f}m",
        f"semantic={n_sem}  obstacle={n_obs}",
    ]
    font, scale, thick = cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1
    pad, lh = 5, 16
    box_w = max(cv2.getTextSize(l, font, scale, thick)[0][0] for l in lines) + pad * 2
    box_h = lh * len(lines) + pad * 2
    cv2.rectangle(img, (4, 4), (4 + box_w, 4 + box_h), (255, 255, 255), -1)
    cv2.rectangle(img, (4, 4), (4 + box_w, 4 + box_h), (180, 180, 180),  1)
    for i, line in enumerate(lines):
        cv2.putText(img, line, (4 + pad, 4 + pad + lh * (i + 1) - 3),
                    font, scale, (30, 30, 30), thick, cv2.LINE_AA)


def main() -> int:
    ap = argparse.ArgumentParser(description="Live RTABMap occupancy grid viewer")
    ap.add_argument("--scale", type=int, default=2,
                    help="screen pixels per grid cell (default: 2). "
                         "Keeps zoom constant as the map grows.")
    ap.add_argument("--host", default=HOST)
    ap.add_argument("--port", type=int, default=PORT)
    args = ap.parse_args()
    if args.scale < 1:
        args.scale = 1

    print(f"[grid_client] connecting to {args.host}:{args.port} ...")
    while True:
        try:
            sock = socket.create_connection((args.host, args.port), timeout=5.0)
            break
        except (ConnectionRefusedError, OSError) as e:
            print(f"[grid_client]   waiting for server ({e}); retry in 1s")
            time.sleep(1.0)

    sock.settimeout(None)
    print(f"[grid_client] connected  -- scale={args.scale}px/cell -- press q or ESC to quit")

    # AUTOSIZE: window snaps to the rendered image size each frame, so
    # cells stay scale×scale pixels regardless of how the map grows.
    cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)

    frame_count = 0
    n_sem = n_obs = 0
    t_last_log = time.time()

    try:
        while True:
            f = read_frame(sock)
            frame_count += 1

            img = grid_to_bgr(np.flipud(f.grid))
            if args.scale > 1:
                img = cv2.resize(
                    img,
                    (img.shape[1] * args.scale, img.shape[0] * args.scale),
                    interpolation=cv2.INTER_NEAREST,
                )
            n_sem = int(np.count_nonzero(f.grid == 1))
            n_obs = int(np.count_nonzero(f.grid == 100))
            draw_overlay(img, f, n_sem, n_obs, frame_count, args.scale)

            cv2.imshow(WIN, img)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):  # q or ESC
                break

            now = time.time()
            if now - t_last_log >= 2.0:
                elapsed = now - t_last_log
                print(f"[grid_client] {frame_count} frames  "
                      f"{frame_count / elapsed:.1f} fps  "
                      f"semantic={n_sem}  obstacle={n_obs}")
                frame_count = 0
                t_last_log = now

    except (ConnectionError, KeyboardInterrupt) as e:
        print(f"[grid_client] stopped: {e}")
    finally:
        sock.close()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    sys.exit(main())
