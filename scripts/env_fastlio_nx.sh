#!/usr/bin/env bash

_env_fastlio_restore_nounset=0
case $- in
  *u*) _env_fastlio_restore_nounset=1 ;;
esac

set +u

_fastlio_source_if_exists() {
  local path="$1"
  if [[ -f "${path}" ]]; then
    # shellcheck disable=SC1090
    source "${path}"
  fi
}

_fastlio_prepend_path() {
  local var_name="$1"
  local entry="$2"
  local current_value="${!var_name:-}"

  if [[ -z "${entry}" || ! -e "${entry}" ]]; then
    return 0
  fi

  if [[ -z "${current_value}" ]]; then
    printf -v "${var_name}" '%s' "${entry}"
    export "${var_name}"
    return 0
  fi

  case ":${current_value}:" in
    *":${entry}:"*) ;;
    *)
      printf -v "${var_name}" '%s:%s' "${entry}" "${current_value}"
      export "${var_name}"
      ;;
  esac
}

: "${AMENT_TRACE_SETUP_FILES:=false}"
: "${AMENT_PYTHON_EXECUTABLE:=/usr/bin/python3}"
: "${COLCON_TRACE:=false}"
: "${COLCON_PYTHON_EXECUTABLE:=/usr/bin/python3}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
export FASTLIO_ENV_ROLE="nx"

_fastlio_source_if_exists "/opt/ros/foxy/setup.bash"
_fastlio_source_if_exists "${WORKSPACE_DIR}/install/setup.bash"

_fastlio_prepend_path LD_PRELOAD "/lib/aarch64-linux-gnu/libgomp.so.1"

export ONNXRUNTIME_ROOT="${ONNXRUNTIME_ROOT:-$HOME/opt/onnxruntime}"
_fastlio_prepend_path LD_LIBRARY_PATH "${ONNXRUNTIME_ROOT}/lib"

export ROI_LIDAR_CORNER_ENABLE_LIVOX_DRIVER="${ROI_LIDAR_CORNER_ENABLE_LIVOX_DRIVER:-true}"
export ROI_LIDAR_CORNER_LIVOX_DRIVER_LAUNCH="${ROI_LIDAR_CORNER_LIVOX_DRIVER_LAUNCH:-msg_MID360_launch.py}"
export ROI_LIDAR_CORNER_ENABLE_D435I="${ROI_LIDAR_CORNER_ENABLE_D435I:-false}"
export ROI_LIDAR_CORNER_IMAGE_TOPIC="${ROI_LIDAR_CORNER_IMAGE_TOPIC:-/camera/color/image_raw}"
export ROI_LIDAR_CORNER_CAMERA_INFO_TOPIC="${ROI_LIDAR_CORNER_CAMERA_INFO_TOPIC:-/camera/color/camera_info}"
export ROI_LIDAR_CORNER_SEMANTIC_MAPPING_EN="${ROI_LIDAR_CORNER_SEMANTIC_MAPPING_EN:-false}"
export ROI_LIDAR_CORNER_DETECTOR_BACKEND="${ROI_LIDAR_CORNER_DETECTOR_BACKEND:-pt}"
export ROI_LIDAR_CORNER_DETECTOR_USE_GPU="${ROI_LIDAR_CORNER_DETECTOR_USE_GPU:-false}"
export ROI_LIDAR_CORNER_DETECTOR_CONF_THRESHOLD="${ROI_LIDAR_CORNER_DETECTOR_CONF_THRESHOLD:-0.25}"
export ROI_LIDAR_CORNER_DETECTOR_IOU_THRESHOLD="${ROI_LIDAR_CORNER_DETECTOR_IOU_THRESHOLD:-0.45}"
export ROI_LIDAR_CORNER_DETECTOR_INPUT_SIZE="${ROI_LIDAR_CORNER_DETECTOR_INPUT_SIZE:-640}"
export ROI_LIDAR_CORNER_DETECTOR_CLASS_FILTER="${ROI_LIDAR_CORNER_DETECTOR_CLASS_FILTER:-[]}"
export ROI_LIDAR_CORNER_PUBLISH_DEBUG_IMAGE="${ROI_LIDAR_CORNER_PUBLISH_DEBUG_IMAGE:-true}"
export ROI_LIDAR_CORNER_OPEN_DEBUG_WINDOW="${ROI_LIDAR_CORNER_OPEN_DEBUG_WINDOW:-true}"
export ROI_LIDAR_CORNER_ENABLE_RVIZ="${ROI_LIDAR_CORNER_ENABLE_RVIZ:-true}"
export ROI_LIDAR_CORNER_MIN_POINTS="${ROI_LIDAR_CORNER_MIN_POINTS:-5}"
export ROI_LIDAR_CORNER_MAX_TIME_DIFF_CLOUD="${ROI_LIDAR_CORNER_MAX_TIME_DIFF_CLOUD:-0.25}"
export ROI_LIDAR_CORNER_MAX_TIME_DIFF_ODOM="${ROI_LIDAR_CORNER_MAX_TIME_DIFF_ODOM:-0.08}"
export ROI_LIDAR_CORNER_MIN_RANGE="${ROI_LIDAR_CORNER_MIN_RANGE:-0.2}"
export ROI_LIDAR_CORNER_MAX_RANGE="${ROI_LIDAR_CORNER_MAX_RANGE:-30.0}"
export ROI_LIDAR_CORNER_POINTCLOUD_TOPIC="${ROI_LIDAR_CORNER_POINTCLOUD_TOPIC:-/cloud_registered}"
export ROI_LIDAR_CORNER_ODOM_TOPIC="${ROI_LIDAR_CORNER_ODOM_TOPIC:-/Odometry}"
export ROI_LIDAR_CORNER_HISTORY_WINDOW_SEC="${ROI_LIDAR_CORNER_HISTORY_WINDOW_SEC:-0.8}"
export ROI_LIDAR_CORNER_MAX_WINDOW_FRAMES="${ROI_LIDAR_CORNER_MAX_WINDOW_FRAMES:-8}"
export ROI_LIDAR_CORNER_CACHE_VOXEL_SIZE="${ROI_LIDAR_CORNER_CACHE_VOXEL_SIZE:-0.1}"
export ROI_LIDAR_CORNER_BBOX_EXPAND_RATIO="${ROI_LIDAR_CORNER_BBOX_EXPAND_RATIO:-0.15}"
export ROI_LIDAR_CORNER_CORNER_TARGET_POINTS="${ROI_LIDAR_CORNER_CORNER_TARGET_POINTS:-24}"
export ROI_LIDAR_CORNER_CORNER_TARGET_FRAMES="${ROI_LIDAR_CORNER_CORNER_TARGET_FRAMES:-2}"
export ROI_LIDAR_CORNER_CORNER_CAP_POINTS="${ROI_LIDAR_CORNER_CORNER_CAP_POINTS:-96}"
export ROI_LIDAR_CORNER_NEO_CANNY_LOW="${ROI_LIDAR_CORNER_NEO_CANNY_LOW:-50}"
export ROI_LIDAR_CORNER_NEO_CANNY_HIGH="${ROI_LIDAR_CORNER_NEO_CANNY_HIGH:-200}"
export ROI_LIDAR_CORNER_NEO_HOUGH_THRESHOLD="${ROI_LIDAR_CORNER_NEO_HOUGH_THRESHOLD:-50}"
export ROI_LIDAR_CORNER_NEO_MIN_LINE_LENGTH="${ROI_LIDAR_CORNER_NEO_MIN_LINE_LENGTH:-50}"
export ROI_LIDAR_CORNER_NEO_MAX_LINE_GAP="${ROI_LIDAR_CORNER_NEO_MAX_LINE_GAP:-50}"
export ROI_LIDAR_CORNER_NEO_BLUR_KERNEL_SIZE="${ROI_LIDAR_CORNER_NEO_BLUR_KERNEL_SIZE:-5}"
export ROI_LIDAR_CORNER_NEO_BORDER_RATIO="${ROI_LIDAR_CORNER_NEO_BORDER_RATIO:-0.15}"

unset -f _fastlio_source_if_exists
unset -f _fastlio_prepend_path

if [[ "${_env_fastlio_restore_nounset}" -eq 1 ]]; then
  set -u
else
  set +u
fi
unset _env_fastlio_restore_nounset
