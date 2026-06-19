"""
두 YOLO11-seg 데이터셋(msde_gp.yolov11, --yolo.yolov11)을 merged.yolov11로 병합.

통합 클래스 (Arrow 제외 3-클래스):
    0=centerLine, 1=parkingLine, 2=sign

- msde_gp: ID가 이미 통합 기준과 동일 -> 그대로 복사
- --yolo : Arrow(0) 줄 삭제, 1->0, 2->1, 3->2 재매핑

원본 폴더는 수정하지 않고, merged.yolov11는 매 실행 시 새로 만든다(멱등).
"""
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "merged.yolov11"
OUT_IMG = OUT / "train" / "images"
OUT_LBL = OUT / "train" / "labels"

NAMES = ["centerLine", "parkingLine", "sign"]
IMG_EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".webp"}

# --yolo 클래스 재매핑: 원본ID -> 통합ID (None = 삭제)
YOLO_REMAP = {0: None, 1: 0, 2: 1, 3: 2}


def reset_output():
    if OUT.exists():
        shutil.rmtree(OUT)
    OUT_IMG.mkdir(parents=True)
    OUT_LBL.mkdir(parents=True)


def find_image(images_dir: Path, stem: str):
    for ext in IMG_EXTS:
        p = images_dir / f"{stem}{ext}"
        if p.exists():
            return p
    return None


def copy_image(src: Path):
    dst = OUT_IMG / src.name
    if dst.exists():
        print(f"  [WARN] 이미지 파일명 충돌: {src.name}")
    shutil.copy2(src, dst)


def remap_label_lines(lines, remap):
    """remap=None 이면 그대로 둠. 아니면 첫 토큰 재매핑/삭제."""
    out = []
    for line in lines:
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        cid = int(parts[0])
        if remap is not None:
            new_id = remap.get(cid, cid)
            if new_id is None:  # 삭제 (Arrow)
                continue
            parts[0] = str(new_id)
        out.append(" ".join(parts))
    return out


def process_dataset(name: str, src_dir: Path, remap):
    img_dir = src_dir / "train" / "images"
    lbl_dir = src_dir / "train" / "labels"
    n_img = n_empty = n_dropped = 0

    for lbl in sorted(lbl_dir.glob("*.txt")):
        img = find_image(img_dir, lbl.stem)
        if img is None:
            print(f"  [WARN] 이미지 없음, 라벨 건너뜀: {lbl.name}")
            continue

        with open(lbl, "r", encoding="utf-8") as f:
            orig = f.readlines()
        new_lines = remap_label_lines(orig, remap)
        n_dropped += len([l for l in orig if l.strip()]) - len(new_lines)
        if not new_lines:
            n_empty += 1

        copy_image(img)
        with open(OUT_LBL / lbl.name, "w", encoding="utf-8") as f:
            f.write("\n".join(new_lines) + ("\n" if new_lines else ""))
        n_img += 1

    print(f"[{name}] 처리 {n_img}장, 삭제된 인스턴스 {n_dropped}개, 빈 라벨(배경) {n_empty}개")
    return n_img


def write_yaml():
    yaml_path = OUT / "data.yaml"
    # `path:` 의도적으로 생략 -> ultralytics가 이 yaml 폴더 기준으로 train/val을
    # 해석하므로 merged.yolov11을 그대로 zip해서 RunPod /workspace에 풀어도 동작(이식성).
    content = (
        "train: train/images\n"
        "val: train/images   # 별도 검증셋 없음 - train과 동일 경로\n"
        f"nc: {len(NAMES)}\n"
        f"names: {NAMES}\n"
    )
    yaml_path.write_text(content, encoding="utf-8")
    print(f"[yaml] {yaml_path}")


def main():
    reset_output()
    total = 0
    total += process_dataset("msde_gp", ROOT / "msde_gp.yolov11", remap=None)
    total += process_dataset("--yolo", ROOT / "--yolo.yolov11", YOLO_REMAP)
    write_yaml()

    n_out_img = len(list(OUT_IMG.glob("*")))
    n_out_lbl = len(list(OUT_LBL.glob("*.txt")))
    print(f"\n완료: 총 {total}장 병합 -> images={n_out_img}, labels={n_out_lbl}")


if __name__ == "__main__":
    main()
