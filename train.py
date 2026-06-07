"""
merged.yolov11 데이터셋으로 YOLO11-seg 학습.
결과물은 c:\\dev\\test1 폴더에 저장된다 (가중치: test1/weights/best.pt).
"""
from ultralytics import YOLO


def main():
    # 사전학습 가중치에서 시작 (n=nano. 더 큰 모델은 yolo11s-seg / yolo11m-seg 등)
    model = YOLO("yolo11n-seg.pt")

    model.train(
        data="c:/dev/merged.yolov11/data.yaml",
        epochs=100,
        imgsz=640,
        batch=16,
        project="c:/dev",   # 저장 부모 폴더
        name="test1",        # 결과 폴더명 -> c:/dev/test1
        exist_ok=True,       # 기존 test1 폴더에 덮어쓰기 (False면 test12, test13... 새로 생성)
        device=0,            # GPU 0번 사용. CPU만 있으면 "cpu" 로 변경
    )


if __name__ == "__main__":
    main()
