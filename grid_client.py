"""
grid_client.py - Live visualizer for the RTABMap GUI's TCP grid stream.

Connects to 127.0.0.1:7777 and renders the occupancy grid + robot pose
in real time. Semantic-tagged cells (value 80) are drawn in red so you
can see VLM detections (parking lines etc.) layered on top of the
structural map.

Cell value legend:
    -1  unknown    -> gray
     0  free       -> white
    80  semantic   -> red       (added by RTABMap GUI when RTABMAP_SEMANTIC_URL is set)
   100  obstacle   -> black

Usage:
    pip install matplotlib numpy
    python grid_client.py
"""

from __future__ import annotations

import socket
import struct
import sys
import time
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap, BoundaryNorm

HOST = "127.0.0.1"
PORT = 7777
HEADER_FMT = "<iiffffff"        # width, height, xMin, yMin, cellSize, poseX, poseY, poseYaw
HEADER_SIZE = struct.calcsize(HEADER_FMT)  # = 32


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


def grid_to_rgb(grid: np.ndarray) -> np.ndarray:
    """Map cell values to an RGB image: unknown=gray, free=white,
    semantic(80)=red, obstacle(100)=black, anything else=blue (debug)."""
    h, w = grid.shape
    rgb = np.full((h, w, 3), 128, dtype=np.uint8)  # default: gray (unknown)
    rgb[grid == 0]   = (255, 255, 255)              # free
    rgb[grid == 80]  = (220,  40,  40)              # semantic
    rgb[grid == 100] = (  0,   0,   0)              # obstacle
    return rgb


def main() -> int:
    print(f"[grid_client] connecting to {HOST}:{PORT} ...")
    while True:
        try:
            sock = socket.create_connection((HOST, PORT), timeout=5.0)
            break
        except (ConnectionRefusedError, OSError) as e:
            print(f"[grid_client]   waiting for server ({e}); retry in 1s")
            time.sleep(1.0)

    sock.settimeout(None)
    print("[grid_client] connected")

    plt.ion()
    fig, ax = plt.subplots(figsize=(9, 7))
    ax.set_title("RTABMap occupancy grid + semantic (red)")
    img_artist = None
    arrow_artist = None
    text_artist = None

    try:
        frame_count = 0
        t_last_log = time.time()

        while True:
            f = read_frame(sock)
            frame_count += 1

            rgb = grid_to_rgb(f.grid)
            extent = (
                f.x_min,
                f.x_min + f.width  * f.cell_size,
                f.y_min,
                f.y_min + f.height * f.cell_size,
            )

            if img_artist is None:
                img_artist = ax.imshow(np.flipud(rgb), origin="lower", extent=extent)
                ax.set_xlabel("x (m)")
                ax.set_ylabel("y (m)")
                ax.set_aspect("equal")
            else:
                img_artist.set_data(np.flipud(rgb))
                img_artist.set_extent(extent)

            # robot pose: red dot + heading arrow
            if arrow_artist is not None:
                arrow_artist.remove()
            if text_artist is not None:
                text_artist.remove()
            arrow_len = 0.5
            arrow_artist = ax.arrow(
                f.pose_x, f.pose_y,
                arrow_len * np.cos(f.pose_yaw),
                arrow_len * np.sin(f.pose_yaw),
                head_width=0.15, head_length=0.15,
                fc="red", ec="red", length_includes_head=True,
            )
            n_sem = int(np.count_nonzero(f.grid == 80))
            n_obs = int(np.count_nonzero(f.grid == 100))
            text_artist = ax.text(
                0.02, 0.98,
                f"frame {frame_count}\n"
                f"pose=({f.pose_x:+.2f}, {f.pose_y:+.2f}, yaw={np.degrees(f.pose_yaw):+.1f}°)\n"
                f"grid {f.width}x{f.height}  cell={f.cell_size:.2f}m\n"
                f"semantic={n_sem}  obstacle={n_obs}",
                transform=ax.transAxes, va="top", ha="left",
                fontsize=9, family="monospace",
                bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="gray", alpha=0.85),
            )

            plt.pause(0.001)

            now = time.time()
            if now - t_last_log >= 2.0:
                print(f"[grid_client] {frame_count} frames received "
                      f"({frame_count/(now - t_last_log + 1e-6):.1f} fps last window), "
                      f"semantic_cells={n_sem}")
                frame_count = 0
                t_last_log = now

    except (ConnectionError, KeyboardInterrupt) as e:
        print(f"[grid_client] stopped: {e}")
        return 0
    finally:
        sock.close()


if __name__ == "__main__":
    sys.exit(main())
