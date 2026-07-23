#!/usr/bin/env bash
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT}/scripts/piper_env.sh"

LOG_DIR="${ROOT}/.ros/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/yolo_smoke_$(date +%Y%m%d_%H%M%S).log"

exec > >(tee -a "${LOG_FILE}") 2>&1

echo "Python: $(command -v python3)"
python3 --version
echo "PIPER_PYTHON: ${PIPER_PYTHON}"
echo "Log: ${LOG_FILE}"

python3 - <<'PY'
from ultralytics import YOLO

model = YOLO("yolo11n.pt")
result = model.predict("bus.jpg", verbose=False, device="cpu")[0]
print("detections", len(result.boxes))
PY
