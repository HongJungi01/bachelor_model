"""
prototype_prd3.py — RTABMap 맵 기반 순수 경로 생성기 (+ 확인용 최소 뷰어).

rtab이 TCP 7777로 주는 occupancy grid + 로봇 pose를 받아, 맵에 박힌 **출구 태그
(grid 값 10 = exit_area)** 를 목표로 잡고 거기까지의 경로를 D* Lite(dstar_core3)로
생성한다. 출력은 **월드 waypoint(미터)** 리스트. 경로 생성 알고리즘 자체(dstar_core3)는
손대지 않는다.

게임형 기능(자율 추종, 안개 자동 공개, 프론티어 탐색, 한 칸씩 이동)은 모두 제거했다.
로봇은 직접 움직이지 않으며, 매 프레임 "현재 pose → 출구" 경로만 다시 생성한다.

RTABMap 셀 값(int8, 와이어)            →  플래너 레전드(int32, local_grid)
    -1  미탐색(unknown)                 → 255  (안개 / 약한 미탐색 페널티)
     0  자유공간(free)                  →   0  (이동 가능)
     1  시맨틱 벽(centerLine/parkingLine) → 100  (장애물 — 통과 불가)
    10  출구(exit_area)                 →   0  (이동 가능; 위치는 '목표'로 따로 사용)
   100  구조 장애물(obstacle)           → 100  (장애물)
  그 외(미래 11..18 / 21..28 방향 태그) →   0  (이번 범위 보류: 이동 가능 처리)

조작:
    좌클릭        임시 목표 지정 (출구 태그가 아직 없을 때의 테스트용 폴백)
    R            재계획(강제 리셋)
    ESC / Q      종료

실행:
    pip install pygame numpy numba
    python prototype_prd3.py
    python prototype_prd3.py --host 127.0.0.1 --port 7777
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import sys
import threading
import time

import numpy as np
import pygame

from numba import types
from numba.typed import List

from dstar_core3 import (
    reset_and_replan,
    calc_edge_cost,
    update_dist_wall_map,
)

# --- 네트워크 / 와이어 포맷 (grid_client.py 와 동일) ------------------------
HOST = "127.0.0.1"
PORT = 7777
HEADER_FMT = "<iiffffff"   # width, height, xMin, yMin, cellSize, poseX, poseY, poseYaw
HEADER_SIZE = struct.calcsize(HEADER_FMT)   # = 32

# --- 셀 값 ------------------------------------------------------------------
V_UNKNOWN = -1
V_FREE = 0
V_SEMANTIC_WALL = 1
V_EXIT = 10          # 목표 지점 (rtab 출구 태그)
V_OBSTACLE = 100

# 플래너 레전드
P_FREE = 0
P_OBSTACLE = 100
P_UNKNOWN = 255

# --- 화면 레이아웃 ----------------------------------------------------------
UI_WIDTH = 260
MAX_CANVAS_W, MAX_CANVAS_H = 1280, 820   # 맵 렌더 영역 상한 (셀 크기 자동 산출)
MAX_CELL = 40                            # 셀이 너무 커지지 않게 상한
MIN_SCREEN_H = 360

# --- 색상 -------------------------------------------------------------------
C_BG = (30, 30, 30)
C_PANEL = (40, 40, 40)
C_UNKNOWN = (60, 60, 60)      # 안개 (-1)
C_FREE = (210, 210, 210)      # 자유 (0)
C_OBSTACLE = (20, 20, 20)     # 구조 장애물 (100)
C_SEMANTIC = (200, 60, 60)    # 시맨틱 벽 (1) — 계획상 장애물, 색만 구분
C_EXIT = (50, 220, 70)        # 출구 (10)
C_PATH = (255, 200, 0)
C_AGENT = (60, 120, 255)
C_GOAL = (50, 230, 50)
C_TEXT = (235, 235, 235)


# ===========================================================================
# 좌표 / 변환 유틸 (순수 함수)
# ===========================================================================
def to_planner_grid(raw: np.ndarray) -> np.ndarray:
    """RTABMap int8 occupancy(flipud 정렬) → 플래너 레전드 int32 (같은 모양).

    -1→255, 1→100(시맨틱 벽=장애물), 100→100, 그 외(0/10/방향태그)→0.
    출구(10)는 '이동 가능'으로 두고, 목표 '위치'는 find_exit_cell 로 따로 찾는다.
    """
    out = np.zeros(raw.shape, dtype=np.int32)        # 기본 = 자유(0)
    out[raw == V_UNKNOWN] = P_UNKNOWN
    out[raw == V_SEMANTIC_WALL] = P_OBSTACLE
    out[raw == V_OBSTACLE] = P_OBSTACLE
    return out


def find_exit_cell(raw: np.ndarray):
    """raw(flipud 정렬)에서 출구 태그(값 10) 셀들의 중심에 가장 가까운 출구 셀 → (col, row).
    출구 태그가 없으면 None."""
    ys, xs = np.where(raw == V_EXIT)
    if len(xs) == 0:
        return None
    cx, cy = xs.mean(), ys.mean()
    i = int(np.argmin((xs - cx) ** 2 + (ys - cy) ** 2))
    return int(xs[i]), int(ys[i])


def world_to_cell(wx: float, wy: float, meta: dict) -> tuple[int, int]:
    """월드 좌표(m) → (col, row). 화면은 북쪽이 위가 되도록 row를 뒤집는다(flipud 정렬)."""
    col = int((wx - meta["x_min"]) / meta["cell"])
    row = (meta["h"] - 1) - int((wy - meta["y_min"]) / meta["cell"])
    return col, row


def cell_to_world(col: int, row: int, meta: dict) -> tuple[float, float]:
    """(col, row) 셀 중심 → 월드 좌표(m). world_to_cell 의 역변환."""
    wx = meta["x_min"] + (col + 0.5) * meta["cell"]
    wy = meta["y_min"] + ((meta["h"] - 1 - row) + 0.5) * meta["cell"]
    return wx, wy


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


# ===========================================================================
# 경로 생성기 (rtab 프레임 → 월드 waypoint). dstar_core3 코어 재사용, 무변경.
# ===========================================================================
class PathGenerator:
    def __init__(self):
        self.gw = self.gh = 0
        self.g_map = self.rhs_map = self.local_grid = self.dist_wall_map = None
        self.pq = None
        # 마지막 계산 결과 (뷰어가 그리기에 사용)
        self.start_cell = None
        self.goal_cell = None
        self.goal_src = "none"     # "exit" | "manual" | "none"
        self.path_cells: list[tuple[int, int]] = []

    def _ensure(self, w: int, h: int):
        if (w, h) == (self.gw, self.gh):
            return
        self.gw, self.gh = w, h
        self.g_map = np.full((h, w), np.inf, dtype=np.float64)
        self.rhs_map = np.full((h, w), np.inf, dtype=np.float64)
        self.local_grid = np.zeros((h, w), dtype=np.int32)
        self.dist_wall_map = np.full((h, w), 30.0, dtype=np.float64)
        self.pq = List.empty_list(
            types.Tuple((types.float64, types.float64, types.int64, types.int64))
        )

    def _extract_path(self, start, goal):
        """현재 g_map 을 따라 start→goal 경로를 그리디 하강으로 추출 (셀 리스트)."""
        g, lg, dw = self.g_map, self.local_grid, self.dist_wall_map
        path = [start]
        cur = start
        seen = {start}
        for _ in range(self.gw * self.gh):
            if cur == goal:
                break
            cx, cy = cur
            best, nxt = np.inf, cur
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < self.gw and 0 <= ny < self.gh:
                        c = calc_edge_cost(cx, cy, nx, ny, lg, dw, 3) + g[ny, nx]
                        if c < best:
                            best, nxt = c, (nx, ny)
            if nxt == cur or best == np.inf or nxt in seen:
                break
            seen.add(nxt)
            path.append(nxt)
            cur = nxt
        return path

    def compute(self, meta: dict, raw_i8: np.ndarray, goal_world=None):
        """한 프레임에 대해 pose→출구 경로를 생성한다.

        반환: 월드 waypoint 리스트 [(wx, wy), ...] (미터). 목표가 없으면 빈 리스트.
        goal_world: 출구 태그가 없을 때만 쓰는 임시 목표(월드 좌표) 폴백.
        """
        raw = np.flipud(raw_i8)                 # 북쪽이 위로 오도록 정렬
        h, w = raw.shape
        self._ensure(w, h)
        self.local_grid[:, :] = to_planner_grid(raw)
        update_dist_wall_map(self.local_grid, self.dist_wall_map, 3.0)

        # 시작 = 로봇 pose
        sx, sy = world_to_cell(meta["px"], meta["py"], meta)
        sx, sy = _clamp(sx, 0, w - 1), _clamp(sy, 0, h - 1)
        self.start_cell = (sx, sy)

        # 목표 = 출구 태그(값 10) 우선, 없으면 임시 목표(클릭) 폴백
        exit_cell = find_exit_cell(raw)
        if exit_cell is not None:
            gx, gy = exit_cell
            self.goal_src = "exit"
        elif goal_world is not None:
            gx, gy = world_to_cell(goal_world[0], goal_world[1], meta)
            gx, gy = _clamp(gx, 0, w - 1), _clamp(gy, 0, h - 1)
            self.goal_src = "manual"
        else:
            self.goal_cell = None
            self.goal_src = "none"
            self.path_cells = []
            return []

        self.goal_cell = (gx, gy)
        reset_and_replan(
            sx, sy, gx, gy,
            self.g_map, self.rhs_map, self.local_grid, self.dist_wall_map,
            self.pq, 0.0, 3,
        )
        self.path_cells = self._extract_path((sx, sy), (gx, gy))
        return [cell_to_world(c, r, meta) for c, r in self.path_cells]


def generate_waypoints(meta: dict, raw_i8: np.ndarray, goal_world=None):
    """일회성 호출용 편의 함수: 한 프레임 → 월드 waypoint 리스트."""
    return PathGenerator().compute(meta, raw_i8, goal_world)


# ===========================================================================
# RTABMap TCP 클라이언트 (백그라운드 수신)
# ===========================================================================
class RtabmapClient:
    """RTABMap TCP grid 스트림(포트 7777)을 백그라운드 스레드에서 읽어 최신 프레임만 보관한다."""

    def __init__(self, host: str = HOST, port: int = PORT):
        self.host, self.port = host, port
        self._lock = threading.Lock()
        self._frame = None        # (meta: dict, grid: np.ndarray int8 (H, W))
        self._connected = False
        self._stop = False
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop = True

    @property
    def connected(self) -> bool:
        return self._connected

    def latest(self):
        with self._lock:
            return self._frame

    @staticmethod
    def _recv_exact(sock: socket.socket, n: int) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("server closed connection")
            buf.extend(chunk)
        return bytes(buf)

    def _loop(self):
        sock = None
        while not self._stop:
            try:
                sock = socket.create_connection((self.host, self.port), timeout=5.0)
                sock.settimeout(None)
                self._connected = True
                print(f"[rtabmap] connected to {self.host}:{self.port}")
                while not self._stop:
                    hdr = self._recv_exact(sock, HEADER_SIZE)
                    w, h, xmin, ymin, cs, px, py, yaw = struct.unpack(HEADER_FMT, hdr)
                    body = self._recv_exact(sock, w * h)
                    grid = np.frombuffer(body, dtype=np.int8).reshape(h, w).copy()
                    meta = {
                        "w": w, "h": h, "x_min": xmin, "y_min": ymin,
                        "cell": cs, "px": px, "py": py, "yaw": yaw,
                    }
                    with self._lock:
                        self._frame = (meta, grid)
            except (ConnectionError, OSError) as e:
                self._connected = False
                if not self._stop:
                    print(f"[rtabmap] waiting for server ({e}); retry in 1s")
                    time.sleep(1.0)
            finally:
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass
                    sock = None


# ===========================================================================
# 확인용 최소 뷰어
# ===========================================================================
class Viewer:
    def __init__(self, host: str, port: int):
        pygame.init()
        self.font = pygame.font.SysFont("malgungothic", 14)

        self.client = RtabmapClient(host, port)
        self.client.start()
        self.gen = PathGenerator()

        self.cell = MAX_CELL
        self.gw = self.gh = 0
        self.meta = None
        self.raw = None                 # flipud 된 원본 프레임 (색칠용)
        self.waypoints: list[tuple[float, float]] = []
        self.goal_world = None          # 클릭 임시 목표 (월드 좌표, 맵이 커져도 고정)
        self._last_logged = None
        self._t_log = 0.0

        self.screen = pygame.display.set_mode((640, MIN_SCREEN_H))
        pygame.display.set_caption("D* Lite × RTABMap — 경로 생성기")
        self.clock = pygame.time.Clock()
        self.running = True

    def _fit(self, w: int, h: int):
        if (w, h) == (self.gw, self.gh):
            return
        self.gw, self.gh = w, h
        self.cell = max(1, min(MAX_CELL, MAX_CANVAS_W // w, MAX_CANVAS_H // h))
        sw = w * self.cell + UI_WIDTH
        sh = max(h * self.cell, MIN_SCREEN_H)
        self.screen = pygame.display.set_mode((sw, sh))

    def step(self):
        frame = self.client.latest()
        if frame is None:
            return
        meta, raw_i8 = frame
        self._fit(meta["w"], meta["h"])
        self.meta = meta
        self.raw = np.flipud(raw_i8)

        # 경로 생성 (출구 태그 우선, 없으면 클릭 임시 목표)
        self.waypoints = self.gen.compute(meta, raw_i8, goal_world=self.goal_world)

        # 변경 시 throttled 콘솔 출력
        now = time.time()
        sig = (self.gen.goal_src, self.gen.goal_cell, len(self.waypoints))
        if sig != self._last_logged and now - self._t_log > 0.3:
            self._last_logged, self._t_log = sig, now
            if self.waypoints:
                s, g = self.waypoints[0], self.waypoints[-1]
                print(f"[path] {len(self.waypoints)} waypoints  "
                      f"start({s[0]:+.2f},{s[1]:+.2f}) -> "
                      f"goal({g[0]:+.2f},{g[1]:+.2f})  [{self.gen.goal_src}]")
            else:
                print(f"[path] 목표 없음 (출구 태그/클릭 대기)  [{self.gen.goal_src}]")

    # ----- 입력 ------------------------------------------------------------
    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    self.running = False
                elif event.key == pygame.K_r:
                    self.gen = PathGenerator()   # 강제 리셋
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = event.pos
                if self.meta and mx < self.gw * self.cell and my < self.gh * self.cell:
                    col, row = mx // self.cell, my // self.cell
                    self.goal_world = cell_to_world(col, row, self.meta)

    # ----- 렌더 ------------------------------------------------------------
    def _cell_center(self, col, row):
        return col * self.cell + self.cell // 2, row * self.cell + self.cell // 2

    def _draw_map(self):
        if self.raw is None:
            return
        raw = self.raw
        disp = np.empty((raw.shape[0], raw.shape[1], 3), dtype=np.uint8)
        disp[:] = C_UNKNOWN
        disp[raw == V_FREE] = C_FREE
        disp[raw == V_SEMANTIC_WALL] = C_SEMANTIC
        disp[raw == V_EXIT] = C_EXIT
        disp[raw == V_OBSTACLE] = C_OBSTACLE
        surf = pygame.surfarray.make_surface(np.transpose(disp, (1, 0, 2)))
        surf = pygame.transform.scale(surf, (self.gw * self.cell, self.gh * self.cell))
        self.screen.blit(surf, (0, 0))

    def _draw_overlays(self):
        gen = self.gen
        if len(gen.path_cells) > 1:
            pts = [self._cell_center(c, r) for c, r in gen.path_cells]
            pygame.draw.lines(self.screen, C_PATH, False, pts, max(2, self.cell // 10))
            pygame.draw.circle(self.screen, C_PATH, pts[-1], max(4, self.cell // 4))

        if gen.goal_cell is not None:
            gx, gy = self._cell_center(*gen.goal_cell)
            pygame.draw.circle(self.screen, C_GOAL, (gx, gy), max(5, self.cell // 3), 2)

        if gen.start_cell is not None and self.meta is not None:
            ax, ay = self._cell_center(*gen.start_cell)
            pygame.draw.circle(self.screen, C_AGENT, (ax, ay), max(4, self.cell // 3))
            yaw = self.meta["yaw"]
            ln = self.cell  # 화면 y는 아래로 증가 → sin 부호 반전
            tip = (int(ax + ln * math.cos(yaw)), int(ay - ln * math.sin(yaw)))
            pygame.draw.line(self.screen, C_AGENT, (ax, ay), tip, max(2, self.cell // 12))

    def _draw_panel(self):
        x0 = self.gw * self.cell
        sw, sh = self.screen.get_size()
        pygame.draw.rect(self.screen, C_PANEL, (x0, 0, UI_WIDTH, sh))

        connected = self.client.connected
        m = self.meta
        goal_label = {"exit": "출구 태그(10)", "manual": "클릭 임시목표", "none": "없음"}[self.gen.goal_src]
        lines = [
            ("RTABMap", C_GOAL if connected else C_SEMANTIC),
            (f"  {'연결됨' if connected else '연결 대기중...'}", C_TEXT),
        ]
        if m is not None:
            lines += [
                (f"격자 {m['w']}x{m['h']}  셀 {m['cell']:.3f}m", C_TEXT),
                (f"pose ({m['px']:+.2f}, {m['py']:+.2f})", C_TEXT),
                (f"yaw {math.degrees(m['yaw']):+.1f}deg", C_TEXT),
            ]
        lines += [
            ("", C_TEXT),
            (f"목표: {goal_label}", C_GOAL if self.gen.goal_src != "none" else C_SEMANTIC),
            (f"경로: {len(self.waypoints)} waypoint", C_TEXT),
        ]
        if self.waypoints:
            g = self.waypoints[-1]
            lines.append((f"  도착 ({g[0]:+.2f}, {g[1]:+.2f}) m", C_TEXT))
        lines += [
            ("", C_TEXT),
            ("[조작]", C_TEXT),
            (" 좌클릭  임시 목표(폴백)", C_TEXT),
            (" R      재계획", C_TEXT),
            (" ESC/Q  종료", C_TEXT),
            ("", C_TEXT),
            ("[범례]", C_TEXT),
            (" 회색 미탐색 / 흰 자유", C_FREE),
            (" 검정 구조벽 / 빨강 시맨틱벽", C_SEMANTIC),
            (" 초록 출구 / 노랑 경로", C_EXIT),
        ]
        y = 10
        for text, color in lines:
            if text:
                self.screen.blit(self.font.render(text, True, color), (x0 + 10, y))
            y += 20

    def draw(self):
        self.screen.fill(C_BG)
        self._draw_map()
        self._draw_overlays()
        self._draw_panel()
        pygame.display.flip()

    def run(self):
        while self.running:
            self.handle_events()
            self.step()
            self.draw()
            self.clock.tick(15)
        self.client.stop()
        pygame.quit()


def main() -> int:
    ap = argparse.ArgumentParser(description="RTABMap 맵 기반 경로 생성기 (+ 최소 뷰어)")
    ap.add_argument("--host", default=HOST)
    ap.add_argument("--port", type=int, default=PORT)
    args = ap.parse_args()
    Viewer(args.host, args.port).run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
