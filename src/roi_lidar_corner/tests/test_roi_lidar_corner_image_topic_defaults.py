from __future__ import annotations

import importlib.util
from pathlib import Path
import re
import sys
import types


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "roi_lidar_corner.launch.py"
GENERATOR_PATH = PACKAGE_ROOT / "roi_lidar_corner" / "roi_generator_node.py"
SOLVER_PATH = PACKAGE_ROOT / "roi_lidar_corner" / "corner_lidar_solver_node.py"


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
        "roi_lidar_corner_launch_under_test",
        "ament_index_python",
        "ament_index_python.packages",
        "launch",
        "launch.actions",
        "launch.conditions",
        "launch.substitutions",
        "launch_ros",
        "launch_ros.actions",
        "launch_ros.parameter_descriptions",
    ]:
        sys.modules.pop(name, None)

    ament_index_python = types.ModuleType("ament_index_python")
    ament_packages = types.ModuleType("ament_index_python.packages")
    ament_packages.get_package_share_directory = lambda package_name: f"/fake/share/{package_name}"
    _install_module("ament_index_python", ament_index_python)
    _install_module("ament_index_python.packages", ament_packages)

    launch_module = types.ModuleType("launch")
    launch_module.LaunchDescription = FakeLaunchDescription
    _install_module("launch", launch_module)

    launch_actions = types.ModuleType("launch.actions")
    launch_actions.DeclareLaunchArgument = FakeDeclareLaunchArgument
    _install_module("launch.actions", launch_actions)

    launch_conditions = types.ModuleType("launch.conditions")
    launch_conditions.IfCondition = FakeIfCondition
    _install_module("launch.conditions", launch_conditions)

    launch_substitutions = types.ModuleType("launch.substitutions")
    launch_substitutions.LaunchConfiguration = FakeLaunchConfiguration
    _install_module("launch.substitutions", launch_substitutions)

    launch_ros = types.ModuleType("launch_ros")
    launch_ros_actions = types.ModuleType("launch_ros.actions")
    launch_ros_actions.Node = FakeNode
    launch_ros_parameter_descriptions = types.ModuleType("launch_ros.parameter_descriptions")
    launch_ros_parameter_descriptions.ParameterValue = FakeParameterValue
    _install_module("launch_ros", launch_ros)
    _install_module("launch_ros.actions", launch_ros_actions)
    _install_module("launch_ros.parameter_descriptions", launch_ros_parameter_descriptions)

    spec = importlib.util.spec_from_file_location("roi_lidar_corner_launch_under_test", LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    _install_module("roi_lidar_corner_launch_under_test", module)
    spec.loader.exec_module(module)
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


def test_standalone_launch_defaults_to_color_image_topic() -> None:
    module = _load_launch_module()

    description = module.generate_launch_description()
    defaults = _declare_argument_defaults(description)

    assert defaults["image_topic"] == "/camera/color/image_raw"


def test_nodes_declare_color_image_topic_defaults() -> None:
    expected_pattern = re.compile(r'declare_parameter\("image_topic", "/camera/color/image_raw"\)')

    assert expected_pattern.search(GENERATOR_PATH.read_text(encoding="utf-8"))
    assert expected_pattern.search(SOLVER_PATH.read_text(encoding="utf-8"))


def test_standalone_launch_forces_detector_class_filter_through_string_parameter_value() -> None:
    module = _load_launch_module()
    description = module.generate_launch_description()

    generator = _find_node(description, executable="roi_generator_node.py")
    detector_class_filter = generator.parameters[0]["detector_class_filter"]

    assert isinstance(detector_class_filter, FakeParameterValue)
    assert detector_class_filter.value_type is str
    assert isinstance(detector_class_filter.value, FakeLaunchConfiguration)
    assert detector_class_filter.value.name == "detector_class_filter"
