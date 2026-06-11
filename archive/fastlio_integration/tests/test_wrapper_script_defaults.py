from __future__ import annotations

from pathlib import Path
import re


WORKSPACE_ROOT = Path(__file__).resolve().parents[4]
DEV_ENV_PATH = WORKSPACE_ROOT / "scripts" / "env_fastlio.sh"
NX_ENV_PATH = WORKSPACE_ROOT / "scripts" / "env_fastlio_nx.sh"
RUN_SCRIPT_PATH = WORKSPACE_ROOT / "scripts" / "run_fastlio_with_roi.sh"
CAPTURE_SCRIPT_PATH = WORKSPACE_ROOT / "scripts" / "capture_lidar_projection_exposure.sh"

ENV_DEFAULT_PATTERN = re.compile(
    r'export\s+(ROI_LIDAR_CORNER_[A-Z0-9_]+)="\$\{[A-Z0-9_]+:-([^}]*)\}"'
)
RUN_FALLBACK_PATTERN = re.compile(
    r'"[^"]+:=\$\{(ROI_LIDAR_CORNER_[A-Z0-9_]+):-([^}]*)\}"'
)

EXPECTED_DEFAULTS = {
    "ROI_LIDAR_CORNER_ENABLE_LIVOX_DRIVER": "true",
    "ROI_LIDAR_CORNER_LIVOX_DRIVER_LAUNCH": "msg_MID360_launch.py",
    "ROI_LIDAR_CORNER_ENABLE_D435I": "true",
    "ROI_LIDAR_CORNER_IMAGE_TOPIC": "/camera/color/image_raw",
    "ROI_LIDAR_CORNER_CAMERA_INFO_TOPIC": "/camera/color/camera_info",
    "ROI_LIDAR_CORNER_CORNER_RADIUS": "5",
    "ROI_LIDAR_CORNER_SEMANTIC_MAPPING_EN": "false",
    "ROI_LIDAR_CORNER_DETECTOR_BACKEND": "pt",
    "ROI_LIDAR_CORNER_DETECTOR_USE_GPU": "true",
    "ROI_LIDAR_CORNER_DETECTOR_CONF_THRESHOLD": "0.25",
    "ROI_LIDAR_CORNER_DETECTOR_IOU_THRESHOLD": "0.45",
    "ROI_LIDAR_CORNER_DETECTOR_INPUT_SIZE": "640",
    "ROI_LIDAR_CORNER_DETECTOR_CLASS_FILTER": "[]",
    "ROI_LIDAR_CORNER_PUBLISH_DEBUG_IMAGE": "true",
    "ROI_LIDAR_CORNER_OPEN_DEBUG_WINDOW": "false",
    "ROI_LIDAR_CORNER_ENABLE_RVIZ": "false",
    "ROI_LIDAR_CORNER_MIN_POINTS": "2",
    "ROI_LIDAR_CORNER_MAX_TIME_DIFF_CLOUD": "1.0",
    "ROI_LIDAR_CORNER_MAX_TIME_DIFF_ODOM": "0.5",
    "ROI_LIDAR_CORNER_MIN_RANGE": "0.2",
    "ROI_LIDAR_CORNER_MAX_RANGE": "30.0",
    "ROI_LIDAR_CORNER_POINTCLOUD_TOPIC": "/cloud_registered",
    "ROI_LIDAR_CORNER_ODOM_TOPIC": "/Odometry",
    "ROI_LIDAR_CORNER_HISTORY_WINDOW_SEC": "1.0",
    "ROI_LIDAR_CORNER_MAX_WINDOW_FRAMES": "30",
    "ROI_LIDAR_CORNER_CACHE_VOXEL_SIZE": "0.1",
    "ROI_LIDAR_CORNER_BBOX_EXPAND_RATIO": "0.15",
    "ROI_LIDAR_CORNER_CORNER_TARGET_POINTS": "6",
    "ROI_LIDAR_CORNER_CORNER_TARGET_FRAMES": "2",
    "ROI_LIDAR_CORNER_CORNER_CAP_POINTS": "96",
    "ROI_LIDAR_CORNER_POST_MAX_Z_JUMP_M": "0.8",
    "ROI_LIDAR_CORNER_NEO_CANNY_LOW": "50",
    "ROI_LIDAR_CORNER_NEO_CANNY_HIGH": "200",
    "ROI_LIDAR_CORNER_NEO_HOUGH_THRESHOLD": "50",
    "ROI_LIDAR_CORNER_NEO_MIN_LINE_LENGTH": "50",
    "ROI_LIDAR_CORNER_NEO_MAX_LINE_GAP": "50",
    "ROI_LIDAR_CORNER_NEO_BLUR_KERNEL_SIZE": "5",
    "ROI_LIDAR_CORNER_NEO_BORDER_RATIO": "0.15",
}

EXPECTED_RUN_FALLBACKS = {
    **EXPECTED_DEFAULTS,
    "ROI_LIDAR_CORNER_PUBLISH_DEBUG_UV": "true",
    "ROI_LIDAR_CORNER_DEBUG_OVERLAY_FRAME_COUNT": "1",
    "ROI_LIDAR_CORNER_SUBSCRIBE_CORNER3D_DEBUG": "true",
    "ROI_LIDAR_CORNER_SUBSCRIBE_SOLVER_DEBUG_UV": "true",
    "ROI_LIDAR_CORNER_ENABLE_DEBUG_MARKERS": "true",
}


def _parse_env_defaults(path: Path) -> dict[str, str]:
    return {
        match.group(1): match.group(2)
        for match in ENV_DEFAULT_PATTERN.finditer(path.read_text(encoding="utf-8"))
    }


def _parse_run_script_fallbacks(path: Path) -> dict[str, str]:
    return {
        match.group(1): match.group(2)
        for match in RUN_FALLBACK_PATTERN.finditer(path.read_text(encoding="utf-8"))
    }


def test_dev_and_nx_env_scripts_share_identical_wrapper_defaults() -> None:
    assert _parse_env_defaults(DEV_ENV_PATH) == _parse_env_defaults(NX_ENV_PATH)


def test_wrapper_env_scripts_match_agreed_defaults() -> None:
    assert _parse_env_defaults(DEV_ENV_PATH) == EXPECTED_DEFAULTS
    assert _parse_env_defaults(NX_ENV_PATH) == EXPECTED_DEFAULTS


def test_run_script_fallbacks_match_wrapper_defaults() -> None:
    assert _parse_run_script_fallbacks(RUN_SCRIPT_PATH) == EXPECTED_RUN_FALLBACKS


def test_projection_exposure_wrapper_supplies_fastlio_config_defaults() -> None:
    script = CAPTURE_SCRIPT_PATH.read_text(encoding="utf-8")

    assert (
        'FASTLIO_CONFIG_PATH="${ROI_LIDAR_CORNER_FASTLIO_CONFIG_PATH:-${WORKSPACE_ROOT}/src/fast_lio_lx/config}"'
        in script
    )
    assert 'FASTLIO_CONFIG_FILE="${ROI_LIDAR_CORNER_FASTLIO_CONFIG_FILE:-mid360.yaml}"' in script
    assert '"--fastlio-config-path" "${FASTLIO_CONFIG_PATH}"' in script
    assert '"--fastlio-config-file" "${FASTLIO_CONFIG_FILE}"' in script
