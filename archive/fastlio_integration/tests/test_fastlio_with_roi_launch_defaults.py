from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import sys
import types


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "fastlio_with_roi.launch.py"


class FakeLaunchDescription:
    def __init__(self, actions):
        self.entities = list(actions)


class FakeDeclareLaunchArgument:
    def __init__(self, name, default_value=None, description=None):
        self.name = name
        self.default_value = default_value
        self.description = description


class FakeLaunchConfiguration:
    def __init__(self, name: str):
        self.name = name

    def __repr__(self) -> str:
        return f"LaunchConfiguration({self.name!r})"


class FakePathJoinSubstitution:
    def __init__(self, parts):
        self.parts = list(parts)


class FakePythonLaunchDescriptionSource:
    def __init__(self, location):
        self.location = location


class FakeIncludeLaunchDescription:
    def __init__(self, launch_description_source, launch_arguments=None, condition=None):
        self.launch_description_source = launch_description_source
        self.launch_arguments = dict(launch_arguments or [])
        self.condition = condition


class FakeIfCondition:
    def __init__(self, predicate):
        self.predicate = predicate


class FakeNode:
    def __init__(self, *, package, executable, name, output, parameters, condition=None):
        self.package = package
        self.executable = executable
        self.name = name
        self.output = output
        self.parameters = list(parameters)
        self.condition = condition


class FakeParameterValue:
    def __init__(self, value, *, value_type=None):
        self.value = value
        self.value_type = value_type


def _install_module(name: str, module: types.ModuleType) -> None:
    sys.modules[name] = module


def _load_launch_module():
    for name in [
        "fastlio_with_roi_launch_under_test",
        "ament_index_python",
        "ament_index_python.packages",
        "launch",
        "launch.actions",
        "launch.conditions",
        "launch.launch_description_sources",
        "launch.substitutions",
        "launch_ros",
        "launch_ros.actions",
        "launch_ros.parameter_descriptions",
    ]:
        sys.modules.pop(name, None)

    ament_index_python = types.ModuleType("ament_index_python")
    ament_packages = types.ModuleType("ament_index_python.packages")

    def fake_get_package_share_directory(package_name: str) -> str:
        return f"/fake/share/{package_name}"

    ament_packages.get_package_share_directory = fake_get_package_share_directory
    _install_module("ament_index_python", ament_index_python)
    _install_module("ament_index_python.packages", ament_packages)

    launch_module = types.ModuleType("launch")
    launch_module.LaunchDescription = FakeLaunchDescription
    _install_module("launch", launch_module)

    launch_actions = types.ModuleType("launch.actions")
    launch_actions.DeclareLaunchArgument = FakeDeclareLaunchArgument
    launch_actions.IncludeLaunchDescription = FakeIncludeLaunchDescription
    _install_module("launch.actions", launch_actions)

    launch_conditions = types.ModuleType("launch.conditions")
    launch_conditions.IfCondition = FakeIfCondition
    _install_module("launch.conditions", launch_conditions)

    launch_sources = types.ModuleType("launch.launch_description_sources")
    launch_sources.PythonLaunchDescriptionSource = FakePythonLaunchDescriptionSource
    _install_module("launch.launch_description_sources", launch_sources)

    launch_substitutions = types.ModuleType("launch.substitutions")
    launch_substitutions.LaunchConfiguration = FakeLaunchConfiguration
    launch_substitutions.PathJoinSubstitution = FakePathJoinSubstitution
    _install_module("launch.substitutions", launch_substitutions)

    launch_ros = types.ModuleType("launch_ros")
    launch_ros_actions = types.ModuleType("launch_ros.actions")
    launch_ros_actions.Node = FakeNode
    launch_ros_parameter_descriptions = types.ModuleType("launch_ros.parameter_descriptions")
    launch_ros_parameter_descriptions.ParameterValue = FakeParameterValue
    _install_module("launch_ros", launch_ros)
    _install_module("launch_ros.actions", launch_ros_actions)
    _install_module("launch_ros.parameter_descriptions", launch_ros_parameter_descriptions)

    spec = importlib.util.spec_from_file_location("fastlio_with_roi_launch_under_test", LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    _install_module("fastlio_with_roi_launch_under_test", module)

    original_env = os.environ.pop("ROI_LIDAR_CORNER_FASTLIO_CONFIG_FILE", None)
    try:
        spec.loader.exec_module(module)
    finally:
        if original_env is not None:
            os.environ["ROI_LIDAR_CORNER_FASTLIO_CONFIG_FILE"] = original_env
    return module


def _declare_argument_defaults(description: FakeLaunchDescription) -> dict[str, object]:
    defaults = {}
    for entity in description.entities:
        if isinstance(entity, FakeDeclareLaunchArgument):
            defaults[entity.name] = entity.default_value
    return defaults


def _find_node(description: FakeLaunchDescription, *, executable: str) -> FakeNode:
    for entity in description.entities:
        if isinstance(entity, FakeNode) and entity.executable == executable:
            return entity
    raise AssertionError(f"Node with executable={executable!r} not found")


def _find_include(description: FakeLaunchDescription, *, location_suffix: str) -> FakeIncludeLaunchDescription:
    for entity in description.entities:
        if not isinstance(entity, FakeIncludeLaunchDescription):
            continue
        location = getattr(entity.launch_description_source, "location", "")
        if isinstance(location, str) and location.endswith(location_suffix):
            return entity
    raise AssertionError(f"Include with suffix={location_suffix!r} not found")


def test_launch_defaults_use_fast_lio_mid360_yaml() -> None:
    module = _load_launch_module()

    description = module.generate_launch_description()
    defaults = _declare_argument_defaults(description)

    assert defaults["fastlio_config_path"] == "/fake/share/fast_lio/config"
    assert defaults["fastlio_config_file"] == "mid360.yaml"
    assert defaults["semantic_mapping_en"] == "false"


def test_launch_defaults_match_current_wrapper_defaults() -> None:
    module = _load_launch_module()

    description = module.generate_launch_description()
    defaults = _declare_argument_defaults(description)

    assert defaults["enable_d435i"] == "true"
    assert defaults["detector_use_gpu"] == "true"
    assert defaults["min_points"] == "2"
    assert defaults["max_time_diff_cloud"] == "1.0"
    assert defaults["max_time_diff_odom"] == "0.5"
    assert defaults["history_window_sec"] == "1.0"
    assert defaults["max_window_frames"] == "30"
    assert defaults["corner_target_points"] == "6"
    assert defaults["post_max_z_jump_m"] == "0.8"
    assert defaults["corner_radius"] == "5"
    assert defaults["publish_debug_image"] == "true"
    assert defaults["publish_debug_uv"] == "true"
    assert defaults["debug_overlay_frame_count"] == "1"
    assert defaults["subscribe_corner3d_debug"] == "true"
    assert defaults["subscribe_solver_debug_uv"] == "true"
    assert defaults["enable_debug_markers"] == "true"
    assert defaults["open_debug_window"] == "false"
    assert defaults["rviz"] == "false"


def test_launch_defaults_use_front_face_topics() -> None:
    text = LAUNCH_PATH.read_text(encoding="utf-8")

    assert '"/roi_lidar_corner/front_face_rois"' in text
    assert '"/roi_lidar_corner/front_face_corners"' in text
    assert '"/roi_lidar_corner/front_face_debug"' in text


def test_launch_passes_fast_lio_semantic_switch_through_include() -> None:
    module = _load_launch_module()

    description = module.generate_launch_description()
    fastlio_include = _find_include(description, location_suffix="fast_lio/launch/mapping.launch.py")

    semantic_mapping_en = fastlio_include.launch_arguments["semantic_mapping_en"]
    assert isinstance(semantic_mapping_en, FakeLaunchConfiguration)
    assert semantic_mapping_en.name == "semantic_mapping_en"


def test_launch_passes_selected_fast_lio_config_through_to_solver() -> None:
    module = _load_launch_module()
    description = module.generate_launch_description()

    solver = _find_node(description, executable="corner_lidar_solver_node.py")
    params = solver.parameters[0]

    assert isinstance(params["fastlio_config_path"], FakeLaunchConfiguration)
    assert params["fastlio_config_path"].name == "fastlio_config_path"
    assert isinstance(params["fastlio_config_file"], FakeLaunchConfiguration)
    assert params["fastlio_config_file"].name == "fastlio_config_file"


def test_launch_surface_drops_legacy_args_and_exposes_current_tuning_args() -> None:
    module = _load_launch_module()
    description = module.generate_launch_description()
    defaults = _declare_argument_defaults(description)

    for removed_name in [
        "enable_yolo",
        "yolo_weights",
        "yolo_conf",
        "yolo_imgsz",
        "camera_fx",
        "camera_fy",
        "camera_cx",
        "camera_cy",
    ]:
        assert removed_name not in defaults

    for expected_name in [
        "detector_class_filter",
        "corner_radius",
        "neo_canny_low",
        "neo_canny_high",
        "neo_hough_threshold",
        "neo_min_line_length",
        "neo_max_line_gap",
        "neo_blur_kernel_size",
        "neo_border_ratio",
        "debug_uv_output_topic",
    ]:
        assert expected_name in defaults


def test_launch_wires_current_generator_and_solver_parameters() -> None:
    module = _load_launch_module()
    description = module.generate_launch_description()

    generator = _find_node(description, executable="roi_generator_node.py")
    generator_params = generator.parameters[0]

    detector_class_filter = generator_params["detector_class_filter"]
    assert isinstance(detector_class_filter, FakeParameterValue)
    assert detector_class_filter.value_type is str
    assert isinstance(detector_class_filter.value, FakeLaunchConfiguration)
    assert detector_class_filter.value.name == "detector_class_filter"
    assert isinstance(generator_params["neo_canny_low"], FakeLaunchConfiguration)
    assert generator_params["neo_canny_low"].name == "neo_canny_low"
    assert isinstance(generator_params["neo_canny_high"], FakeLaunchConfiguration)
    assert generator_params["neo_canny_high"].name == "neo_canny_high"
    assert isinstance(generator_params["neo_hough_threshold"], FakeLaunchConfiguration)
    assert generator_params["neo_hough_threshold"].name == "neo_hough_threshold"
    assert isinstance(generator_params["neo_min_line_length"], FakeLaunchConfiguration)
    assert generator_params["neo_min_line_length"].name == "neo_min_line_length"
    assert isinstance(generator_params["neo_max_line_gap"], FakeLaunchConfiguration)
    assert generator_params["neo_max_line_gap"].name == "neo_max_line_gap"
    assert isinstance(generator_params["neo_blur_kernel_size"], FakeLaunchConfiguration)
    assert generator_params["neo_blur_kernel_size"].name == "neo_blur_kernel_size"
    assert isinstance(generator_params["neo_border_ratio"], FakeLaunchConfiguration)
    assert generator_params["neo_border_ratio"].name == "neo_border_ratio"
    assert isinstance(generator_params["corner_radius"], FakeLaunchConfiguration)
    assert generator_params["corner_radius"].name == "corner_radius"
    assert isinstance(generator_params["corner3d_topic"], FakeLaunchConfiguration)
    assert generator_params["corner3d_topic"].name == "point_output_topic"
    assert isinstance(generator_params["solver_debug_uv_topic"], FakeLaunchConfiguration)
    assert generator_params["solver_debug_uv_topic"].name == "debug_uv_output_topic"
    assert isinstance(generator_params["subscribe_corner3d_debug"], FakeLaunchConfiguration)
    assert generator_params["subscribe_corner3d_debug"].name == "subscribe_corner3d_debug"
    assert isinstance(generator_params["subscribe_solver_debug_uv"], FakeLaunchConfiguration)
    assert generator_params["subscribe_solver_debug_uv"].name == "subscribe_solver_debug_uv"

    solver = _find_node(description, executable="corner_lidar_solver_node.py")
    solver_params = solver.parameters[0]

    assert isinstance(solver_params["publish_debug_uv"], FakeLaunchConfiguration)
    assert solver_params["publish_debug_uv"].name == "publish_debug_uv"
    assert isinstance(solver_params["debug_overlay_frame_count"], FakeLaunchConfiguration)
    assert solver_params["debug_overlay_frame_count"].name == "debug_overlay_frame_count"


def test_launch_forces_detector_class_filter_through_string_parameter_value() -> None:
    module = _load_launch_module()
    description = module.generate_launch_description()

    generator = _find_node(description, executable="roi_generator_node.py")
    detector_class_filter = generator.parameters[0]["detector_class_filter"]

    assert isinstance(detector_class_filter, FakeParameterValue)
    assert detector_class_filter.value_type is str
    assert isinstance(detector_class_filter.value, FakeLaunchConfiguration)
    assert detector_class_filter.value.name == "detector_class_filter"

    solver = _find_node(description, executable="corner_lidar_solver_node.py")
    solver_params = solver.parameters[0]

    for removed_name in ["camera_k", "camera_fx", "camera_fy", "camera_cx", "camera_cy"]:
        assert removed_name not in solver_params

    assert isinstance(solver_params["debug_uv_output_topic"], FakeLaunchConfiguration)
    assert solver_params["debug_uv_output_topic"].name == "debug_uv_output_topic"


def test_launch_gates_debug_markers_node_with_launch_argument() -> None:
    module = _load_launch_module()
    description = module.generate_launch_description()

    debug_markers = _find_node(description, executable="roi_lidar_debug_markers.py")

    assert isinstance(debug_markers.condition, FakeIfCondition)
    assert isinstance(debug_markers.condition.predicate, FakeLaunchConfiguration)
    assert debug_markers.condition.predicate.name == "enable_debug_markers"
