import pyrealsense2 as rs
import numpy as np
import cv2, os, time

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)

save_dir = "dataset"
os.makedirs(save_dir, exist_ok=True)

INTERVAL = 0.5  # 0.5초마다 저장
last_save = time.time()
count = 0

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        img = np.asanyarray(color_frame.get_data())
        now = time.time()

        if now - last_save >= INTERVAL:
            timestamp = int(now * 1000)
            cv2.imwrite(f"{save_dir}/{timestamp}.png", img)
            last_save = now
            count += 1
            print(f"[{count}] Saved")

        cv2.imshow("RGB", img)
        if cv2.waitKey(1) == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()