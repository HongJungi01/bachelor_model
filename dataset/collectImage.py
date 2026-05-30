import pyrealsense2 as rs
import numpy as np
import cv2
import os
import time

# ── 설정 ──────────────────────────────────────────
SAVE_DIR    = "rgb_images"
INTERVAL    = 0.5   # 저장 간격 (초)
WIDTH, HEIGHT = 848, 480
FPS         = 30
# ──────────────────────────────────────────────────

os.makedirs(SAVE_DIR, exist_ok=True)

pipeline = rs.pipeline()
config   = rs.config()
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
pipeline.start(config)

count     = 0
last_save = time.time()
saving    = True   # 스페이스바로 토글

print("=== RealSense RGB 수집 ===")
print(" SPACE : 자동저장 일시정지/재개")
print(" S     : 수동 즉시 저장")
print(" Q     : 종료")
print("=========================")

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        img = np.asanyarray(color_frame.get_data())
        now = time.time()

        # 인터벌 자동 저장
        saved_this_frame = False
        if saving and (now - last_save >= INTERVAL):
            timestamp = int(now * 1000)
            cv2.imwrite(os.path.join(SAVE_DIR, f"{timestamp}.png"), img)
            last_save = now
            count += 1
            saved_this_frame = True
            print(f"[AUTO] {count:04d} | {timestamp}.png")

        # ── OSD 오버레이 ──────────────────────────
        display = img.copy()

        status_color = (0, 255, 0) if saving else (0, 100, 255)
        status_text  = f"{'REC' if saving else 'PAUSE'}  #{count:04d}"
        cv2.rectangle(display, (0, 0), (260, 36), (0, 0, 0), -1)
        cv2.putText(display, status_text, (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

        # 저장 순간 플래시 효과
        if saved_this_frame:
            cv2.rectangle(display, (0, 0), (WIDTH, HEIGHT), (255, 255, 255), 4)

        # 인터벌 프로그레스 바
        if saving:
            elapsed  = min(now - last_save, INTERVAL)
            bar_w    = int((elapsed / INTERVAL) * WIDTH)
            cv2.rectangle(display, (0, HEIGHT - 6), (WIDTH, HEIGHT), (50, 50, 50), -1)
            cv2.rectangle(display, (0, HEIGHT - 6), (bar_w, HEIGHT), (0, 220, 0), -1)

        # 저장 경로 표시
        cv2.rectangle(display, (0, HEIGHT - 28), (WIDTH, HEIGHT - 8), (0, 0, 0), -1)
        cv2.putText(display, f"-> {SAVE_DIR}/", (6, HEIGHT - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
        # ──────────────────────────────────────────

        cv2.imshow("RealSense RGB Collector", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord(' '):
            saving = not saving
            print(f"[{'재개' if saving else '일시정지'}] 자동 저장")
            last_save = time.time()
        elif key == ord('s'):
            timestamp = int(now * 1000)
            cv2.imwrite(os.path.join(SAVE_DIR, f"{timestamp}.png"), img)
            count += 1
            print(f"[MANUAL] {count:04d} | {timestamp}.png")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    print(f"\n총 {count}장 저장 완료 → {SAVE_DIR}/")