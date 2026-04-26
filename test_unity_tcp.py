"""
test_unity_tcp.py — Unity → rtabmap TCP 패킷 검증 서버

rtabmap을 대신해서 7778 포트에서 listen하고, Unity (RTABMapStreamer)가
보내는 Calib/IMU/RGBD 패킷을 파싱·검증·표시합니다.

사용:
    python test_unity_tcp.py [--port 7778] [--show]

  --show 옵션: RGB와 Depth를 OpenCV 창으로 실시간 표시 (numpy/opencv 필요)
  옵션 없이 실행: 콘솔에 패킷 정보만 출력

프로토콜 (little-endian):
  Header (5B):  [type:u8][payloadSize:u32]
  Type 1 Calib (136B):  w(u32), h(u32), fx/fy/cx/cy(f64), localTransform[12](f64)
  Type 2 IMU   (56B):   stamp(f64), gx,gy,gz(f64), ax,ay,az(f64)
  Type 3 RGBD:           stamp(f64), w(u32), h(u32), rgb(W*H*3), depth(W*H*2 mm uint16)
"""

import argparse
import socket
import struct
import sys
import time

PKT_CALIB = 1
PKT_IMU = 2
PKT_RGBD = 3


def recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("client disconnected")
        buf.extend(chunk)
    return bytes(buf)


def parse_calib(payload):
    if len(payload) != 136:
        raise ValueError(f"Calib: expected 136B, got {len(payload)}")
    w, h = struct.unpack_from("<II", payload, 0)
    fx, fy, cx, cy = struct.unpack_from("<dddd", payload, 8)
    lt = struct.unpack_from("<12d", payload, 8 + 32)
    return {
        "w": w, "h": h,
        "fx": fx, "fy": fy, "cx": cx, "cy": cy,
        "localTransform": lt,
    }


def parse_imu(payload):
    if len(payload) != 56:
        raise ValueError(f"IMU: expected 56B, got {len(payload)}")
    stamp, gx, gy, gz, ax, ay, az = struct.unpack("<7d", payload)
    return {"t": stamp, "gyro": (gx, gy, gz), "accel": (ax, ay, az)}


def parse_rgbd_header(payload_head):
    stamp = struct.unpack_from("<d", payload_head, 0)[0]
    w, h = struct.unpack_from("<II", payload_head, 8)
    return stamp, w, h


def fmt_calib(c):
    lt = c["localTransform"]
    lt_str = "\n    ".join(
        f"[{lt[i]:6.2f} {lt[i+1]:6.2f} {lt[i+2]:6.2f} {lt[i+3]:6.2f}]"
        for i in (0, 4, 8)
    )
    return (f"\n  size = {c['w']} x {c['h']}"
            f"\n  fx={c['fx']:.2f}  fy={c['fy']:.2f}  cx={c['cx']:.2f}  cy={c['cy']:.2f}"
            f"\n  localTransform (3x4 row-major, optical→base_link):\n    {lt_str}")


def serve(port, show):
    np = cv2 = None
    if show:
        try:
            import numpy as _np
            import cv2 as _cv2
            np, cv2 = _np, _cv2
        except ImportError:
            print("[WARN] numpy/opencv not installed; --show disabled", file=sys.stderr)
            show = False

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", port))
    srv.listen(1)
    print(f"[server] listening on 0.0.0.0:{port} (Ctrl+C to stop)")

    while True:
        client, addr = srv.accept()
        print(f"\n[server] client connected from {addr}")
        counts = {PKT_CALIB: 0, PKT_IMU: 0, PKT_RGBD: 0, "unknown": 0}
        bytes_total = 0
        t0 = time.time()
        last_calib = None
        last_print = t0

        try:
            while True:
                hdr = recv_exact(client, 5)
                pkt_type = hdr[0]
                payload_size = struct.unpack_from("<I", hdr, 1)[0]
                bytes_total += 5 + payload_size

                if pkt_type == PKT_CALIB:
                    payload = recv_exact(client, payload_size)
                    c = parse_calib(payload)
                    last_calib = c
                    counts[PKT_CALIB] += 1
                    print(f"[Calib #{counts[PKT_CALIB]}]" + fmt_calib(c))

                elif pkt_type == PKT_IMU:
                    payload = recv_exact(client, payload_size)
                    s = parse_imu(payload)
                    counts[PKT_IMU] += 1
                    if counts[PKT_IMU] <= 3 or counts[PKT_IMU] % 100 == 0:
                        gx, gy, gz = s["gyro"]
                        ax, ay, az = s["accel"]
                        a_mag = (ax * ax + ay * ay + az * az) ** 0.5
                        print(f"[IMU #{counts[PKT_IMU]:5d}] t={s['t']:7.3f}s "
                              f"gyro=({gx:+6.3f},{gy:+6.3f},{gz:+6.3f}) rad/s  "
                              f"accel=({ax:+6.2f},{ay:+6.2f},{az:+6.2f}) |a|={a_mag:.2f} m/s²")

                elif pkt_type == PKT_RGBD:
                    head = recv_exact(client, 16)
                    stamp, w, h = parse_rgbd_header(head)
                    rgb = recv_exact(client, w * h * 3)
                    depth = recv_exact(client, w * h * 2)
                    expected = 16 + w * h * 5
                    counts[PKT_RGBD] += 1

                    if counts[PKT_RGBD] <= 3 or counts[PKT_RGBD] % 30 == 0:
                        size_ok = "OK" if payload_size == expected else f"MISMATCH (exp {expected})"
                        print(f"[RGBD #{counts[PKT_RGBD]:4d}] t={stamp:7.3f}s  {w}x{h}  "
                              f"payload={payload_size}B [{size_ok}]")

                    if show:
                        img = np.frombuffer(rgb, dtype=np.uint8).reshape(h, w, 3)
                        bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                        d16 = np.frombuffer(depth, dtype=np.uint16).reshape(h, w)
                        d_min = int(d16[d16 > 0].min()) if (d16 > 0).any() else 0
                        d_max = int(d16.max())
                        d_vis = np.clip(d16.astype(np.float32) / 6000.0 * 255.0, 0, 255).astype(np.uint8)
                        d_color = cv2.applyColorMap(d_vis, cv2.COLORMAP_JET)
                        d_color[d16 == 0] = (0, 0, 0)
                        cv2.putText(bgr, f"RGB {w}x{h}", (10, 25),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        cv2.putText(d_color, f"Depth mm  range [{d_min},{d_max}]", (10, 25),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                        cv2.imshow("RGB", bgr)
                        cv2.imshow("Depth", d_color)
                        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                            print("\n[server] quit by user")
                            return

                else:
                    counts["unknown"] += 1
                    skipped = recv_exact(client, payload_size)
                    print(f"[WARN] unknown packet type={pkt_type} size={payload_size} (skipped)")

                now = time.time()
                if now - last_print >= 5.0:
                    elapsed = now - t0
                    mb = bytes_total / 1e6
                    rgbd_fps = counts[PKT_RGBD] / elapsed if elapsed > 0 else 0
                    imu_hz = counts[PKT_IMU] / elapsed if elapsed > 0 else 0
                    print(f"--- stats: {elapsed:.0f}s  "
                          f"Calib={counts[PKT_CALIB]}  IMU={counts[PKT_IMU]} ({imu_hz:.1f} Hz)  "
                          f"RGBD={counts[PKT_RGBD]} ({rgbd_fps:.1f} fps)  "
                          f"total={mb:.1f} MB ---")
                    last_print = now

        except ConnectionError as e:
            print(f"\n[server] {e}")
        except struct.error as e:
            print(f"\n[ERROR] parse failed: {e}")
        except Exception as e:
            print(f"\n[ERROR] {type(e).__name__}: {e}")
        finally:
            client.close()
            print(f"[server] session ended.  totals: {counts}")
            if show:
                cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=7778, help="listen port (default 7778)")
    ap.add_argument("--show", action="store_true", help="show RGB/Depth in OpenCV windows")
    args = ap.parse_args()
    try:
        serve(args.port, args.show)
    except KeyboardInterrupt:
        print("\n[server] interrupted.")


if __name__ == "__main__":
    main()
