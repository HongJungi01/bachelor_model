"""
test_client.py — TCP client for GridPublisher

Connects to 127.0.0.1:7777, receives 2D occupancy grid packets,
and visualises them with OpenCV.

Packet layout (little-endian):
  Header (32 bytes):
    width(i32), height(i32), xMin(f32), yMin(f32),
    cellSize(f32), poseX(f32), poseY(f32), poseYaw(f32)
  Body (width * height bytes):
    int8 per cell: -1=unknown, 0=free, 100=obstacle

Usage:
  python test_client.py [host] [port]
"""

import socket
import struct
import sys
import math
import numpy as np
import cv2

HEADER_FMT = "<iifffff f"  # 8 fields, 32 bytes
HEADER_SIZE = struct.calcsize(HEADER_FMT)
assert HEADER_SIZE == 32, f"Expected 32 bytes, got {HEADER_SIZE}"


def recv_exact(sock: socket.socket, n: int) -> bytes:
    """Receive exactly n bytes from socket."""
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Server disconnected")
        buf.extend(chunk)
    return bytes(buf)


def map_to_color(grid: np.ndarray) -> np.ndarray:
    """Convert signed int8 occupancy grid to BGR colour image.
       -1 → dark grey (unknown)
        0 → light grey (free)
      100 → black (obstacle)
    """
    h, w = grid.shape
    color = np.zeros((h, w, 3), dtype=np.uint8)

    unknown  = grid == -1
    free     = grid == 0
    obstacle = grid == 100

    color[unknown]  = (89, 89, 89)     # dark grey
    color[free]     = (178, 178, 178)   # light grey
    color[obstacle] = (0, 0, 0)         # black

    return color


def draw_pose(img: np.ndarray, px: int, py: int, yaw: float):
    """Draw robot position (red dot) and heading (arrow)."""
    h, w = img.shape[:2]
    if 0 <= px < w and 0 <= py < h:
        cv2.circle(img, (px, py), 5, (0, 0, 255), -1)
        arrow_len = 20
        dx = int(arrow_len * math.cos(yaw))
        dy = int(-arrow_len * math.sin(yaw))  # y-axis inverted
        cv2.arrowedLine(img, (px, py), (px + dx, py + dy),
                        (0, 0, 255), 2, cv2.LINE_AA, tipLength=0.3)


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 7777

    print(f"Connecting to {host}:{port}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    print("Connected!")

    try:
        while True:
            # 1. Read header
            hdr_bytes = recv_exact(sock, HEADER_SIZE)
            width, height, x_min, y_min, cell_size, pose_x, pose_y, pose_yaw = \
                struct.unpack(HEADER_FMT, hdr_bytes)

            # 2. Read body
            body_size = width * height
            body_bytes = recv_exact(sock, body_size)
            grid = np.frombuffer(body_bytes, dtype=np.int8).reshape((height, width))

            # 3. Visualise
            color = map_to_color(grid)

            # Robot position in pixel coordinates
            px = int((pose_x - x_min) / cell_size)
            py = int(height - 1 - (pose_y - y_min) / cell_size)
            draw_pose(color, px, py, pose_yaw)

            # HUD
            info = f"Grid {width}x{height}  cell={cell_size:.2f}m  pose=({pose_x:.2f},{pose_y:.2f})"
            cv2.putText(color, info, (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            cv2.imshow("Test Client - 2D Map", color)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:   # q or ESC
                break

    except ConnectionError as e:
        print(f"Connection lost: {e}")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        sock.close()
        cv2.destroyAllWindows()
        print("Closed.")


if __name__ == "__main__":
    main()
