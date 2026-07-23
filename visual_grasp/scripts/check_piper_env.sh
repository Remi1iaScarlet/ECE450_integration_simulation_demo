#!/usr/bin/env bash
set -e

source "$(dirname "$0")/piper_env.sh"

python3 - <<'PY'
import importlib

modules = [
    "rospy",
    "tf",
    "sensor_msgs",
    "geometry_msgs",
    "visualization_msgs",
    "pyrealsense2",
    "ultralytics",
    "piper_sdk",
    "can",
    "cv2",
    "numpy",
    "scipy",
]

for name in modules:
    importlib.import_module(name)
    print(f"OK {name}")
PY

rospack find piper_description
echo "OK piper_description"
