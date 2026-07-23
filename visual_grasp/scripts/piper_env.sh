#!/usr/bin/env bash
# Source this file in every terminal before running the real Piper deployment.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export VISUAL_GRASP_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
export VISUAL_GRASP_CONDA_ENV="${VISUAL_GRASP_CONDA_ENV:-${VISUAL_GRASP_ROOT}/.conda_env}"

if [ -f /home/zhou/miniconda3/etc/profile.d/conda.sh ]; then
  # shellcheck disable=SC1091
  source /home/zhou/miniconda3/etc/profile.d/conda.sh
  if ! conda activate "${VISUAL_GRASP_CONDA_ENV}"; then
    echo "Failed to activate conda env: ${VISUAL_GRASP_CONDA_ENV}" >&2
    return 1 2>/dev/null || exit 1
  fi
else
  echo "Missing conda setup: /home/zhou/miniconda3/etc/profile.d/conda.sh" >&2
  return 1 2>/dev/null || exit 1
fi

if [ -f /opt/ros/noetic/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi

if [ -f /home/zhou/pika_ros/install/setup.bash ]; then
  # shellcheck disable=SC1091
  source /home/zhou/pika_ros/install/setup.bash
elif [ -f /home/zhou/pika_ros/devel/setup.bash ]; then
  # shellcheck disable=SC1091
  source /home/zhou/pika_ros/devel/setup.bash
fi

mkdir -p "${VISUAL_GRASP_ROOT}/.ros" \
         "${VISUAL_GRASP_ROOT}/.cache/matplotlib" \
         "${VISUAL_GRASP_ROOT}/.cache/ultralytics"

export ROS_HOME="${VISUAL_GRASP_ROOT}/.ros"
export ROS_LOG_DIR="${VISUAL_GRASP_ROOT}/.ros/log"
export MPLCONFIGDIR="${VISUAL_GRASP_ROOT}/.cache/matplotlib"
export YOLO_CONFIG_DIR="${VISUAL_GRASP_ROOT}/.cache/ultralytics"
export XDG_CONFIG_HOME="${VISUAL_GRASP_ROOT}/.cache"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:--1}"

export PIPER_PYTHON="$(command -v python3)"
export PYTHONPATH="${VISUAL_GRASP_ROOT}/.python_deps:${VISUAL_GRASP_ROOT}:/opt/ros/noetic/lib/python3/dist-packages:/home/zhou/pika_ros/install/lib/python3/dist-packages:/home/zhou/.local/lib/python3.8/site-packages:/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
hash -r 2>/dev/null || true

cd "${VISUAL_GRASP_ROOT}"
