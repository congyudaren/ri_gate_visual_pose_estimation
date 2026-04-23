#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROI_PYTHON_ROOT="${WORKSPACE_ROOT}/src/fast_lio_lx/roi_lidar_corner"
WORKSPACE_SETUP="${WORKSPACE_ROOT}/install/setup.bash"
ROS_SETUP="/opt/ros/${ROS_DISTRO:-foxy}/setup.bash"
FASTLIO_CONFIG_PATH="${ROI_LIDAR_CORNER_FASTLIO_CONFIG_PATH:-${WORKSPACE_ROOT}/src/fast_lio_lx/config}"
FASTLIO_CONFIG_FILE="${ROI_LIDAR_CORNER_FASTLIO_CONFIG_FILE:-mid360.yaml}"
DEFAULT_CAPTURE_ARGS=(
    "--fastlio-config-path" "${FASTLIO_CONFIG_PATH}"
    "--fastlio-config-file" "${FASTLIO_CONFIG_FILE}"
)

if [[ -f "${ROS_SETUP}" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "${ROS_SETUP}"
    set -u
fi

if [[ -f "${WORKSPACE_SETUP}" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "${WORKSPACE_SETUP}"
    set -u
fi

if python3 -c "import roi_lidar_corner.lidar_projection_exposure_capture" >/dev/null 2>&1; then
    python3 -m roi_lidar_corner.lidar_projection_exposure_capture "${DEFAULT_CAPTURE_ARGS[@]}" "$@"
else
    export PYTHONPATH="${ROI_PYTHON_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
    python3 -m roi_lidar_corner.lidar_projection_exposure_capture "${DEFAULT_CAPTURE_ARGS[@]}" "$@"
fi
