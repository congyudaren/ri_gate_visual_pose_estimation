from __future__ import annotations

import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
STANDALONE_LAUNCH_PATH = PACKAGE_ROOT / "launch" / "roi_lidar_corner.launch.py"
GENERATOR_NODE_PATH = PACKAGE_ROOT / "roi_lidar_corner" / "roi_generator_node.py"
SOLVER_NODE_PATH = PACKAGE_ROOT / "roi_lidar_corner" / "corner_lidar_solver_node.py"


def _literal_value(node: ast.AST):
    if isinstance(node, ast.Constant):
        return node.value
    if isinstance(node, ast.List):
        return [_literal_value(element) for element in node.elts]
    if isinstance(node, ast.Tuple):
        return tuple(_literal_value(element) for element in node.elts)
    raise TypeError(f"Unsupported literal node: {ast.dump(node)}")


def _collect_declare_launch_defaults(path: Path) -> dict[str, object]:
    module = ast.parse(path.read_text(encoding="utf-8"))
    defaults: dict[str, object] = {}
    for node in ast.walk(module):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != "DeclareLaunchArgument":
            continue
        if not node.args or not isinstance(node.args[0], ast.Constant) or not isinstance(node.args[0].value, str):
            continue
        for keyword in node.keywords:
            if keyword.arg == "default_value":
                try:
                    defaults[node.args[0].value] = _literal_value(keyword.value)
                except TypeError:
                    pass
    return defaults


def _collect_declared_node_defaults(path: Path) -> dict[str, object]:
    module = ast.parse(path.read_text(encoding="utf-8"))
    defaults: dict[str, object] = {}
    for node in ast.walk(module):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Attribute) or node.func.attr != "declare_parameter":
            continue
        if not node.args or len(node.args) < 2:
            continue
        if not isinstance(node.args[0], ast.Constant) or not isinstance(node.args[0].value, str):
            continue
        try:
            defaults[node.args[0].value] = _literal_value(node.args[1])
        except TypeError:
            pass
    return defaults


def test_standalone_launch_defaults_match_current_wrapper_values() -> None:
    defaults = _collect_declare_launch_defaults(STANDALONE_LAUNCH_PATH)

    assert defaults["image_topic"] == "/camera/color/image_raw"
    assert defaults["publish_debug_image"] == "true"
    assert defaults["corner_radius"] == "5"
    assert defaults["publish_debug_uv"] == "true"
    assert defaults["debug_overlay_frame_count"] == "1"
    assert defaults["subscribe_corner3d_debug"] == "true"
    assert defaults["subscribe_solver_debug_uv"] == "true"
    assert defaults["pointcloud_topic"] == "/points"
    assert defaults["odom_topic"] == "/odom"
    assert defaults["camera_info_topic"] == "/camera/color/camera_info"
    assert defaults["min_points"] == "2"
    assert defaults["max_time_diff_cloud"] == "1.0"
    assert defaults["max_time_diff_odom"] == "0.5"
    assert defaults["min_range"] == "0.2"
    assert defaults["max_range"] == "30.0"
    assert defaults["history_window_sec"] == "1.0"
    assert defaults["max_window_frames"] == "30"
    assert defaults["corner_target_points"] == "6"
    assert defaults["post_max_z_jump_m"] == "0.8"
    assert defaults["detector_use_gpu"] == "true"


def test_roi_generator_node_declares_current_wrapper_defaults() -> None:
    defaults = _collect_declared_node_defaults(GENERATOR_NODE_PATH)

    assert defaults["image_topic"] == "/camera/color/image_raw"
    assert defaults["publish_debug_image"] is True
    assert defaults["corner_radius"] == 5
    assert defaults["detector_backend"] == "pt"
    assert defaults["detector_use_gpu"] is True
    assert defaults["detector_class_filter"] == "[]"


def test_corner_solver_node_declares_current_wrapper_defaults() -> None:
    defaults = _collect_declared_node_defaults(SOLVER_NODE_PATH)

    assert defaults["camera_info_topic"] == "/camera/color/camera_info"
    assert defaults["image_topic"] == "/camera/color/image_raw"
    assert defaults["min_points"] == 2
    assert defaults["max_time_diff_cloud"] == 1.0
    assert defaults["max_time_diff_odom"] == 0.5
    assert defaults["history_window_sec"] == 1.0
    assert defaults["max_window_frames"] == 30
    assert defaults["corner_target_points"] == 6
    assert defaults["post_max_z_jump_m"] == 0.8
