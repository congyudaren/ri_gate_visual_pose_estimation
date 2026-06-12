from __future__ import annotations

import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
STANDALONE_LAUNCH_PATH = PACKAGE_ROOT / "launch" / "roi_lidar_corner.launch.py"
GENERATOR_NODE_PATH = PACKAGE_ROOT / "roi_lidar_corner" / "roi_generator_node.py"
SOLVER_NODE_PATH = PACKAGE_ROOT / "roi_lidar_corner" / "corner_lidar_solver_node.py"
DEBUG_MARKERS_NODE_PATH = PACKAGE_ROOT / "roi_lidar_corner" / "roi_lidar_debug_markers.py"
CONFIG_PATH = PACKAGE_ROOT / "config" / "roi_lidar_corner.yaml"


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
        if not isinstance(node.func, ast.Name):
            continue
        if node.func.id == "DeclareLaunchArgument":
            default_value_node = next(
                (keyword.value for keyword in node.keywords if keyword.arg == "default_value"),
                None,
            )
        elif node.func.id == "_arg" and len(node.args) >= 2:
            default_value_node = node.args[1]
        else:
            continue
        if not node.args or not isinstance(node.args[0], ast.Constant) or not isinstance(node.args[0].value, str):
            continue
        if default_value_node is None:
            continue
        try:
            defaults[node.args[0].value] = _literal_value(default_value_node)
        except TypeError:
            pass
    return defaults


def _collect_launch_argument_descriptions(path: Path) -> dict[str, str]:
    module = ast.parse(path.read_text(encoding="utf-8"))
    descriptions: dict[str, str] = {}
    for node in ast.walk(module):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name):
            continue
        if node.func.id == "DeclareLaunchArgument":
            description_node = next(
                (keyword.value for keyword in node.keywords if keyword.arg == "description"),
                None,
            )
        elif node.func.id == "_arg" and len(node.args) >= 3:
            description_node = node.args[2]
        else:
            continue
        if not node.args or not isinstance(node.args[0], ast.Constant) or not isinstance(node.args[0].value, str):
            continue
        if isinstance(description_node, ast.Constant) and isinstance(description_node.value, str):
            descriptions[node.args[0].value] = description_node.value
    return descriptions


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
    assert defaults["pointcloud_topic"] == "/cloud_registered"
    assert defaults["odom_topic"] == "/Odometry"
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


def test_standalone_launch_arguments_have_visible_categories() -> None:
    descriptions = _collect_launch_argument_descriptions(STANDALONE_LAUNCH_PATH)

    assert descriptions["image_topic"].startswith("[输入]")
    assert descriptions["detector_backend"].startswith("[检测器]")
    assert descriptions["roi_output_topic"].startswith("[输出]")
    assert descriptions["min_points"].startswith("[求解/过滤]")
    assert descriptions["history_window_sec"].startswith("[求解/跟踪]")
    assert descriptions["output_frame_id"].startswith("[变换]")


def test_roi_generator_node_declares_current_wrapper_defaults() -> None:
    defaults = _collect_declared_node_defaults(GENERATOR_NODE_PATH)
    text = GENERATOR_NODE_PATH.read_text(encoding="utf-8")

    assert defaults["image_topic"] == "/camera/color/image_raw"
    assert defaults["publish_debug_image"] is True
    assert defaults["corner_radius"] == 5
    assert defaults["detector_backend"] == "pt"
    assert 'declare_parameter("detector_model_path", _default_share_file("models", "best.pt"))' in text
    assert 'declare_parameter("detector_names_path", _default_share_file("models", "detect.names"))' in text
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
    assert defaults["output_frame_id"] == "body"
    assert defaults["output_extrinsic_r_00"] == 0.0
    assert defaults["output_extrinsic_r_02"] == 1.0
    assert defaults["output_extrinsic_r_10"] == 1.0
    assert defaults["output_extrinsic_r_21"] == 1.0
    assert defaults["output_extrinsic_r_22"] == 0.0
    assert defaults["camera_extrinsic_t"] == [0.049, 0.29671, 0.01812]
    assert defaults["camera_extrinsic_r"] == [
        0.0,
        0.0,
        1.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
    ]


def test_debug_marker_node_declares_launch_defaults() -> None:
    defaults = _collect_declared_node_defaults(DEBUG_MARKERS_NODE_PATH)

    assert defaults["corner_topic"] == "/roi_lidar_corner/front_face_corners"
    assert defaults["marker_topic"] == "/roi_lidar_corner/front_face_markers"
    assert defaults["show_invalid"] is False
    assert defaults["point_scale"] == 0.08
    assert defaults["text_scale"] == 0.08
    assert defaults["frame_id_fallback"] == "map"
    assert defaults["use_text"] is True


def test_config_keeps_legacy_corner3d_debug_topic_separate_from_front_face_corners() -> None:
    text = CONFIG_PATH.read_text(encoding="utf-8")

    assert "corner3d_topic: /roi_lidar_corner/corners3d" in text
    assert "point_output_topic: /roi_lidar_corner/front_face_corners" in text
