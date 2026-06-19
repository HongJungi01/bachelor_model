#!/usr/bin/env bash
# Linux/RunPod launcher for the semantic_service sidecar.
#
# Mirrors run_semantic.bat but assumes torch+CUDA already come from the
# RunPod PyTorch template (do NOT reinstall torch here). On first run it
# installs the non-torch deps from requirements.txt.
#
# Usage (inside the pod, e.g. under tmux so it survives terminal close):
#   bash start_semantic.sh
#
# Env overrides:
#   HOST (default 0.0.0.0)  PORT (default 7788)
#   YOLO_WEIGHTS (default models/best.pt), YOLO_IMGSZ/CONF/IOU
set -euo pipefail

cd "$(dirname "$0")"

python -c "import torch; print(f'[start_semantic] torch {torch.__version__}  cuda={torch.cuda.is_available()}')"

# Install app deps once (torch is intentionally absent from requirements.txt).
if ! python -c "import fastapi, uvicorn, ultralytics" >/dev/null 2>&1; then
    echo "[start_semantic] installing requirements ..."
    pip install -r requirements.txt
fi

if [ ! -f "${YOLO_WEIGHTS:-models/best.pt}" ]; then
    echo "[start_semantic] WARNING: weights '${YOLO_WEIGHTS:-models/best.pt}' not found"
fi

HOST="${HOST:-0.0.0.0}"
PORT="${PORT:-7788}"
echo "[start_semantic] uvicorn listening on ${HOST}:${PORT}"
exec python -m uvicorn app:app --host "${HOST}" --port "${PORT}"
