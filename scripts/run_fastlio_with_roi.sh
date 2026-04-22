#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR_DEFAULT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ENV_SCRIPT="${FASTLIO_ENV_SCRIPT:-${SCRIPT_DIR}/env_fastlio.sh}"

if [[ ! -f "${ENV_SCRIPT}" ]]; then
  echo "找不到环境脚本: ${ENV_SCRIPT}" >&2
  exit 1
fi

source "${ENV_SCRIPT}"

WORKSPACE_DIR="${WORKSPACE_DIR_DEFAULT}"
PACKAGE_NAME="roi_lidar_corner"
LAUNCH_FILE_NAME="fastlio_with_roi.launch.py"
SOURCE_LAUNCH_FILE="${WORKSPACE_DIR}/src/fast_lio_lx/roi_lidar_corner/launch/${LAUNCH_FILE_NAME}"
FASTLIO_CONFIG_FILE="${WORKSPACE_DIR}/src/fast_lio_lx/config/mid360.yaml"
FASTLIO_CONFIG_PATH="$(dirname "${FASTLIO_CONFIG_FILE}")"
FASTLIO_CONFIG_NAME="$(basename "${FASTLIO_CONFIG_FILE}")"

DEFAULT_LAUNCH_ARGS=(
  "fastlio_config_path:=${FASTLIO_CONFIG_PATH}"
  "fastlio_config_file:=${FASTLIO_CONFIG_NAME}"
  "semantic_mapping_en:=${ROI_LIDAR_CORNER_SEMANTIC_MAPPING_EN:-false}"
  "enable_livox_driver:=${ROI_LIDAR_CORNER_ENABLE_LIVOX_DRIVER:-true}"
  "livox_driver_launch:=${ROI_LIDAR_CORNER_LIVOX_DRIVER_LAUNCH:-msg_MID360_launch.py}"
  "enable_d435i:=${ROI_LIDAR_CORNER_ENABLE_D435I:-true}"
  "image_topic:=${ROI_LIDAR_CORNER_IMAGE_TOPIC:-/camera/color/image_raw}"
  "camera_info_topic:=${ROI_LIDAR_CORNER_CAMERA_INFO_TOPIC:-/camera/color/camera_info}"
  "detector_backend:=${ROI_LIDAR_CORNER_DETECTOR_BACKEND:-pt}"
  "detector_use_gpu:=${ROI_LIDAR_CORNER_DETECTOR_USE_GPU:-true}"
  "detector_conf_threshold:=${ROI_LIDAR_CORNER_DETECTOR_CONF_THRESHOLD:-0.25}"
  "detector_iou_threshold:=${ROI_LIDAR_CORNER_DETECTOR_IOU_THRESHOLD:-0.45}"
  "detector_input_size:=${ROI_LIDAR_CORNER_DETECTOR_INPUT_SIZE:-640}"
  "detector_class_filter:=${ROI_LIDAR_CORNER_DETECTOR_CLASS_FILTER:-[]}"
  "pointcloud_topic:=${ROI_LIDAR_CORNER_POINTCLOUD_TOPIC:-/cloud_registered}"
  "odom_topic:=${ROI_LIDAR_CORNER_ODOM_TOPIC:-/Odometry}"
  "min_points:=${ROI_LIDAR_CORNER_MIN_POINTS:-2}"
  "max_time_diff_cloud:=${ROI_LIDAR_CORNER_MAX_TIME_DIFF_CLOUD:-1.0}"
  "max_time_diff_odom:=${ROI_LIDAR_CORNER_MAX_TIME_DIFF_ODOM:-0.5}"
  "min_range:=${ROI_LIDAR_CORNER_MIN_RANGE:-0.2}"
  "max_range:=${ROI_LIDAR_CORNER_MAX_RANGE:-30.0}"
  "history_window_sec:=${ROI_LIDAR_CORNER_HISTORY_WINDOW_SEC:-3.0}"
  "max_window_frames:=${ROI_LIDAR_CORNER_MAX_WINDOW_FRAMES:-30}"
  "cache_voxel_size:=${ROI_LIDAR_CORNER_CACHE_VOXEL_SIZE:-0.1}"
  "bbox_expand_ratio:=${ROI_LIDAR_CORNER_BBOX_EXPAND_RATIO:-0.15}"
  "corner_target_points:=${ROI_LIDAR_CORNER_CORNER_TARGET_POINTS:-6}"
  "corner_target_frames:=${ROI_LIDAR_CORNER_CORNER_TARGET_FRAMES:-2}"
  "corner_cap_points:=${ROI_LIDAR_CORNER_CORNER_CAP_POINTS:-96}"
  "publish_debug_image:=${ROI_LIDAR_CORNER_PUBLISH_DEBUG_IMAGE:-true}"
  "open_debug_window:=${ROI_LIDAR_CORNER_OPEN_DEBUG_WINDOW:-false}"
  "rviz:=${ROI_LIDAR_CORNER_ENABLE_RVIZ:-false}"
  "neo_canny_low:=${ROI_LIDAR_CORNER_NEO_CANNY_LOW:-50}"
  "neo_canny_high:=${ROI_LIDAR_CORNER_NEO_CANNY_HIGH:-200}"
  "neo_hough_threshold:=${ROI_LIDAR_CORNER_NEO_HOUGH_THRESHOLD:-50}"
  "neo_min_line_length:=${ROI_LIDAR_CORNER_NEO_MIN_LINE_LENGTH:-50}"
  "neo_max_line_gap:=${ROI_LIDAR_CORNER_NEO_MAX_LINE_GAP:-50}"
  "neo_blur_kernel_size:=${ROI_LIDAR_CORNER_NEO_BLUR_KERNEL_SIZE:-5}"
  "neo_border_ratio:=${ROI_LIDAR_CORNER_NEO_BORDER_RATIO:-0.15}"
)

if ! command -v ros2 >/dev/null 2>&1; then
  echo "未找到 ros2 命令，请先 source ROS2 环境" >&2
  exit 1
fi

if ros2 pkg prefix "${PACKAGE_NAME}" >/dev/null 2>&1; then
  echo "[run] 使用已安装包启动: ros2 launch ${PACKAGE_NAME} ${LAUNCH_FILE_NAME}"
  exec ros2 launch "${PACKAGE_NAME}" "${LAUNCH_FILE_NAME}" "${DEFAULT_LAUNCH_ARGS[@]}" "$@"
fi

if [[ -f "${SOURCE_LAUNCH_FILE}" ]]; then
  echo "[run] 使用源码启动: ${SOURCE_LAUNCH_FILE}"
  exec ros2 launch "${SOURCE_LAUNCH_FILE}" "${DEFAULT_LAUNCH_ARGS[@]}" "$@"
fi

echo "找不到联合启动入口，请先在 ${WORKSPACE_DIR} 执行 colcon build" >&2
exit 1
