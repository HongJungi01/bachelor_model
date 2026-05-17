"""
Train YOLO11-seg on the simulationyolo dataset.

Roboflow exports drop everything into train/ with no val split, so this
script:
  1. Reads dataset root from --data (default: ../simulationyolo.yolov11)
  2. If valid/ doesn't exist, deterministically splits train/ into
     train+valid (--val-frac, default 0.20) by moving files. Idempotent —
     re-running is a no-op once the split is done.
  3. Writes data_split.yaml with absolute paths next to the dataset's
     data.yaml (leaves the original untouched).
  4. Launches ultralytics segmentation training with defaults tuned for a
     small (~80 image) dataset: yolo11n-seg base, strong augmentation,
     longer epochs, generous early-stop patience.

Usage (from semantic_service/):
    .venv\\Scripts\\activate
    python train.py
    # tweak hyperparams via flags:
    python train.py --epochs 300 --imgsz 1280 --model yolo11s-seg.pt

The trained best.pt lands in runs/segment/train/weights/best.pt; copy it
to semantic_service/models/best.pt to serve via app.py.
"""

from __future__ import annotations

import argparse
import random
import shutil
import sys
from pathlib import Path

import yaml


def split_train_val(train_dir: Path, valid_dir: Path, val_frac: float, seed: int) -> tuple[int, int]:
    """Move val_frac of train/{images,labels} to valid/{images,labels}.
    Returns (n_train_remaining, n_val_moved). Idempotent."""
    train_images = train_dir / "images"
    train_labels = train_dir / "labels"
    valid_images = valid_dir / "images"
    valid_labels = valid_dir / "labels"

    if valid_images.exists() and any(valid_images.iterdir()):
        n_t = sum(1 for _ in train_images.glob("*"))
        n_v = sum(1 for _ in valid_images.glob("*"))
        print(f"[train] valid/ already populated ({n_v} val, {n_t} train) — skipping split")
        return n_t, n_v

    valid_images.mkdir(parents=True, exist_ok=True)
    valid_labels.mkdir(parents=True, exist_ok=True)

    image_exts = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
    images = sorted([p for p in train_images.iterdir() if p.suffix.lower() in image_exts])
    rng = random.Random(seed)
    rng.shuffle(images)

    n_val = max(1, int(round(len(images) * val_frac)))
    val_set = images[:n_val]

    moved = 0
    for img in val_set:
        label = train_labels / (img.stem + ".txt")
        shutil.move(str(img), str(valid_images / img.name))
        if label.exists():
            shutil.move(str(label), str(valid_labels / label.name))
        moved += 1

    n_train = sum(1 for _ in train_images.glob("*") if _.suffix.lower() in image_exts)
    print(f"[train] split: {n_train} train, {moved} val (seed={seed})")
    return n_train, moved


def write_split_yaml(dataset_root: Path, names: list[str]) -> Path:
    """Write data_split.yaml with absolute paths so ultralytics resolves
    images/labels independently of cwd."""
    split_yaml = dataset_root / "data_split.yaml"
    payload = {
        "path": str(dataset_root.resolve()),
        "train": "train/images",
        "val": "valid/images",
        "names": {i: n for i, n in enumerate(names)},
        "nc": len(names),
    }
    split_yaml.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    print(f"[train] wrote {split_yaml}")
    return split_yaml


def main() -> int:
    here = Path(__file__).resolve().parent
    default_data = (here.parent / "simulationyolo.yolov11").resolve()

    ap = argparse.ArgumentParser(description="Train YOLO11-seg on simulationyolo dataset")
    ap.add_argument("--data", type=Path, default=default_data,
                    help=f"Dataset root containing data.yaml (default: {default_data})")
    ap.add_argument("--model", default="yolo11n-seg.pt",
                    help="Base weights (yolo11n-seg.pt / yolo11s-seg.pt / yolo11m-seg.pt)")
    ap.add_argument("--epochs", type=int, default=200)
    ap.add_argument("--imgsz", type=int, default=1280)
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--patience", type=int, default=50)
    ap.add_argument("--val-frac", type=float, default=0.20)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--device", default=0,
                    help="CUDA device index or 'cpu' (default: 0)")
    ap.add_argument("--project", default="runs/segment",
                    help="Ultralytics output project dir")
    ap.add_argument("--name", default="train",
                    help="Run name (output goes to <project>/<name>)")
    args = ap.parse_args()

    if not args.data.exists():
        print(f"[train] ERROR: dataset not found at {args.data}")
        return 1
    data_yaml = args.data / "data.yaml"
    if not data_yaml.exists():
        print(f"[train] ERROR: data.yaml not found in {args.data}")
        return 1

    cfg = yaml.safe_load(data_yaml.read_text(encoding="utf-8"))
    names = cfg.get("names")
    if isinstance(names, dict):
        names = [names[i] for i in sorted(names)]
    if not names:
        print(f"[train] ERROR: data.yaml has no 'names' field")
        return 1
    print(f"[train] dataset: {args.data}  classes: {names}")

    split_train_val(
        train_dir=args.data / "train",
        valid_dir=args.data / "valid",
        val_frac=args.val_frac,
        seed=args.seed,
    )

    split_yaml = write_split_yaml(args.data, names)

    from ultralytics import YOLO

    print(f"[train] loading base weights: {args.model}")
    model = YOLO(args.model)

    print(f"[train] starting training "
          f"(epochs={args.epochs}, imgsz={args.imgsz}, batch={args.batch}, "
          f"device={args.device})")

    # Augmentation tuned for small datasets: stronger geometric + color jitter
    # to compensate for limited samples. mosaic is on by default; mixup and
    # copy_paste give the seg model more polygon variety.
    model.train(
        data=str(split_yaml),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        patience=args.patience,
        device=args.device,
        project=args.project,
        name=args.name,
        seed=args.seed,
        # augmentation
        hsv_h=0.015, hsv_s=0.7, hsv_v=0.4,
        degrees=10.0, translate=0.1, scale=0.5, shear=2.0,
        perspective=0.0, flipud=0.0, fliplr=0.5,
        mosaic=1.0, mixup=0.15, copy_paste=0.30,
        close_mosaic=20,
        # loss / training
        cos_lr=True,
        warmup_epochs=3.0,
    )

    best = Path(args.project) / args.name / "weights" / "best.pt"
    if best.exists():
        print(f"\n[train] done. best weights: {best.resolve()}")
        print(f"        copy to:            {here / 'models' / 'best.pt'}")
        print(f"        then run:           run_semantic.bat")
    else:
        print(f"\n[train] done but best.pt not at expected path: {best}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
