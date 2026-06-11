from __future__ import annotations

from dataclasses import dataclass, field
import importlib.util
import json
from pathlib import Path
import struct
import sys
import types

import numpy as np
import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = PACKAGE_ROOT / "roi_lidar_corner" / "corner_lidar_solver_node.py"
LOOKBACK_PATH = PACKAGE_ROOT / "roi_lidar_corner" / "lookback_solver.py"


@dataclass
class Stamp:
    sec: int = 0
    nanosec: int = 0


@dataclass
class Header:
    stamp: Stamp = field(default_factory=Stamp)
    frame_id: str = ""


@dataclass
class PointField:
    name: str = ""
    offset: int = 0
    datatype: int = 0
    count: int = 1

    INT8 = 1
    UINT8 = 2
    INT16 = 3
    UINT16 = 4
    INT32 = 5
    UINT32 = 6
    FLOAT32 = 7
    FLOAT64 = 8


@dataclass
class PointCloud2:
    header: Header = field(default_factory=Header)
    fields: list[PointField] = field(default_factory=list)
    is_bigendian: bool = False
    point_step: int = 0
    data: bytes = b""


@dataclass
class CameraInfo:
    width: int = 0
    height: int = 0
    k: list[float] = field(default_factory=lambda: [0.0] * 9)


@dataclass
class Image:
    width: int = 0
    height: int = 0


@dataclass
class Quaternion:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class Position:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Pose:
    position: Position = field(default_factory=Position)
    orientation: Quaternion = field(default_factory=Quaternion)


@dataclass
class PoseWithCovariance:
    pose: Pose = field(default_factory=Pose)


@dataclass
class Odometry:
    header: Header = field(default_factory=Header)
    pose: PoseWithCovariance = field(default_factory=PoseWithCovariance)


@dataclass
class String:
    data: str = ""


@dataclass
class Point:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class CornerROI:
    corner_label: int
    mask_origin_x: int
    mask_origin_y: int
    mask_width: int
    mask_height: int
    roi_mask: list[int]
    valid: bool = True


@dataclass
class ObjectROI:
    object_id: int
    class_id: int
    conf: float = 1.0
    corner_rois: list[CornerROI] = field(default_factory=list)


@dataclass
class ObjectROIArray:
    header: Header = field(default_factory=Header)
    objects: list[ObjectROI] = field(default_factory=list)


@dataclass
class StructureROI:
    header: Header = field(default_factory=Header)
    object_id: int = 0
    class_id: int = 0
    conf: float = 1.0
    structure_label: int = 0
    mask_origin_x: int = 0
    mask_origin_y: int = 0
    mask_width: int = 0
    mask_height: int = 0
    roi_mask: list[int] = field(default_factory=list)
    line_u0: float = 0.0
    line_v0: float = 0.0
    line_u1: float = 0.0
    line_v1: float = 0.0
    valid: bool = True
    structure_conf: float = 1.0
    source: str = "test"

    LEFT_POST = 0
    RIGHT_POST = 1
    TOP_BEAM = 2


@dataclass
class FrontFaceROI:
    header: Header = field(default_factory=Header)
    object_id: int = 0
    class_id: int = 0
    conf: float = 1.0
    bbox_xyxy: list[float] = field(default_factory=list)
    structures: list[StructureROI] = field(default_factory=list)


@dataclass
class Corner3D:
    header: Header = field(default_factory=Header)
    object_id: int = 0
    class_id: int = 0
    corner_label: int = 0
    support_point_count: int = 0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    fit_error: float = 0.0
    valid: bool = False


@dataclass
class Corner3DArray:
    header: Header = field(default_factory=Header)
    corners: list[Corner3D] = field(default_factory=list)


@dataclass
class FrontFaceROIArray:
    header: Header = field(default_factory=Header)
    objects: list = field(default_factory=list)


@dataclass
class FrontFaceCorners:
    header: Header = field(default_factory=Header)
    solution_state: int = 0
    valid: bool = False
    tracking_confidence: float = 0.0
    top_source: str = ""
    top_left: Point = field(default_factory=Point)
    top_right: Point = field(default_factory=Point)
    bottom_left: Point = field(default_factory=Point)
    bottom_right: Point = field(default_factory=Point)
    top_left_status: int = 0
    top_right_status: int = 0
    bottom_left_status: int = 0
    bottom_right_status: int = 0

    SOLUTION_INVALID = 0
    SOLUTION_TRACKING = 1
    SOLUTION_LOST = 2
    CORNER_OBSERVED = 0
    CORNER_INFERRED = 1
    CORNER_INVALID = 2


@dataclass
class FrontFaceDebug:
    header: Header = field(default_factory=Header)
    valid: bool = False
    direct_top_left: Point = field(default_factory=Point)
    direct_top_right: Point = field(default_factory=Point)
    direct_top_left_valid: bool = False
    direct_top_right_valid: bool = False
    left_post_valid: bool = False
    left_post_x: float = 0.0
    left_post_z: float = 0.0
    left_post_confidence: float = 0.0
    right_post_valid: bool = False
    right_post_x: float = 0.0
    right_post_z: float = 0.0
    right_post_confidence: float = 0.0
    top_beam_valid: bool = False
    top_beam_y: float = 0.0
    top_beam_confidence: float = 0.0
    tracking_confidence: float = 0.0
    top_source: str = ""


class FakeParameterValue:
    def __init__(self, value):
        self.bool_value = bool(value) if isinstance(value, bool) else False
        self.string_value = value if isinstance(value, str) else ""
        self.integer_value = int(value) if isinstance(value, (int, bool)) else 0
        self.double_value = float(value) if isinstance(value, (int, float)) else 0.0
        self.double_array_value = [float(item) for item in value] if isinstance(value, (list, tuple)) else []


class FakeParameter:
    def __init__(self, value):
        self._value = value

    def get_parameter_value(self) -> FakeParameterValue:
        return FakeParameterValue(self._value)


class FakePublisher:
    def __init__(self, topic: str):
        self.topic = topic
        self.messages = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


class FakeLogger:
    def __init__(self):
        self.records = []

    def info(self, msg: str) -> None:
        self.records.append(msg)

    def warning(self, msg: str) -> None:
        self.records.append(msg)

    def warn(self, msg: str) -> None:
        self.records.append(msg)


class FakeNode:
    parameter_overrides = {}

    def __init__(self, name: str):
        self._node_name = name
        self._params = dict(self.parameter_overrides)
        self._logger = FakeLogger()

    def declare_parameter(self, name: str, default_value):
        if name in self.parameter_overrides:
            self._params[name] = self.parameter_overrides[name]
        else:
            self._params[name] = default_value

    def get_parameter(self, name: str) -> FakeParameter:
        return FakeParameter(self._params[name])

    def create_publisher(self, _msg_type, topic: str, _queue_size: int) -> FakePublisher:
        return FakePublisher(topic)

    def create_subscription(self, _msg_type, _topic: str, _callback, _queue_size: int):
        return object()

    def get_logger(self) -> FakeLogger:
        return self._logger

    def destroy_node(self) -> None:
        return None


def _install_module(name: str, module: types.ModuleType) -> None:
    sys.modules[name] = module


def load_solver_node_module():
    FakeNode.parameter_overrides = {}
    for name in [
        "corner_lidar_solver_node_under_test",
        "roi_lidar_corner",
        "roi_lidar_corner.msg",
        "roi_lidar_corner.lookback_solver",
        "roi_lidar_corner.structure_point_filter",
        "roi_lidar_corner.structure_tracker",
        "roi_lidar_corner.front_face_restorer",
        "rotor_swarm_msgs",
        "rotor_swarm_msgs.msg",
        "rclpy",
        "rclpy.node",
        "sensor_msgs",
        "sensor_msgs.msg",
        "geometry_msgs",
        "geometry_msgs.msg",
        "nav_msgs",
        "nav_msgs.msg",
        "std_msgs",
        "std_msgs.msg",
    ]:
        sys.modules.pop(name, None)

    package_module = types.ModuleType("roi_lidar_corner")
    package_module.__path__ = [str(PACKAGE_ROOT / "roi_lidar_corner")]
    _install_module("roi_lidar_corner", package_module)

    msg_module = types.ModuleType("roi_lidar_corner.msg")
    msg_module.Corner3D = Corner3D
    msg_module.Corner3DArray = Corner3DArray
    msg_module.ObjectROIArray = ObjectROIArray
    msg_module.StructureROI = StructureROI
    msg_module.FrontFaceROI = FrontFaceROI
    msg_module.FrontFaceROIArray = FrontFaceROIArray
    msg_module.FrontFaceDebug = FrontFaceDebug
    _install_module("roi_lidar_corner.msg", msg_module)

    rotor_swarm_msgs_module = types.ModuleType("rotor_swarm_msgs")
    rotor_swarm_msgs_msg_module = types.ModuleType("rotor_swarm_msgs.msg")
    rotor_swarm_msgs_msg_module.FrontFaceCorners = FrontFaceCorners
    _install_module("rotor_swarm_msgs", rotor_swarm_msgs_module)
    _install_module("rotor_swarm_msgs.msg", rotor_swarm_msgs_msg_module)

    rclpy_module = types.ModuleType("rclpy")
    rclpy_module.init = lambda: None
    rclpy_module.shutdown = lambda: None
    rclpy_module.spin = lambda _node: None
    _install_module("rclpy", rclpy_module)

    rclpy_node_module = types.ModuleType("rclpy.node")
    rclpy_node_module.Node = FakeNode
    _install_module("rclpy.node", rclpy_node_module)

    sensor_msgs_module = types.ModuleType("sensor_msgs")
    sensor_msgs_msg_module = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg_module.PointField = PointField
    sensor_msgs_msg_module.PointCloud2 = PointCloud2
    sensor_msgs_msg_module.CameraInfo = CameraInfo
    sensor_msgs_msg_module.Image = Image
    _install_module("sensor_msgs", sensor_msgs_module)
    _install_module("sensor_msgs.msg", sensor_msgs_msg_module)

    geometry_msgs_module = types.ModuleType("geometry_msgs")
    geometry_msgs_msg_module = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg_module.Quaternion = Quaternion
    _install_module("geometry_msgs", geometry_msgs_module)
    _install_module("geometry_msgs.msg", geometry_msgs_msg_module)

    nav_msgs_module = types.ModuleType("nav_msgs")
    nav_msgs_msg_module = types.ModuleType("nav_msgs.msg")
    nav_msgs_msg_module.Odometry = Odometry
    _install_module("nav_msgs", nav_msgs_module)
    _install_module("nav_msgs.msg", nav_msgs_msg_module)

    std_msgs_module = types.ModuleType("std_msgs")
    std_msgs_msg_module = types.ModuleType("std_msgs.msg")
    std_msgs_msg_module.String = String
    std_msgs_msg_module.Header = Header
    _install_module("std_msgs", std_msgs_module)
    _install_module("std_msgs.msg", std_msgs_msg_module)

    lookback_spec = importlib.util.spec_from_file_location("roi_lidar_corner.lookback_solver", LOOKBACK_PATH)
    lookback_module = importlib.util.module_from_spec(lookback_spec)
    assert lookback_spec.loader is not None
    _install_module("roi_lidar_corner.lookback_solver", lookback_module)
    lookback_spec.loader.exec_module(lookback_module)

    node_spec = importlib.util.spec_from_file_location("corner_lidar_solver_node_under_test", MODULE_PATH)
    node_module = importlib.util.module_from_spec(node_spec)
    assert node_spec.loader is not None
    _install_module("corner_lidar_solver_node_under_test", node_module)
    node_spec.loader.exec_module(node_module)
    return node_module, lookback_module


def make_pointcloud(points: list[tuple[float, float, float]], *, stamp_sec: int = 1, frame_id: str = "map") -> PointCloud2:
    data = b"".join(struct.pack("<fff", *point) for point in points)
    return PointCloud2(
        header=Header(stamp=Stamp(sec=stamp_sec), frame_id=frame_id),
        fields=[
            PointField(name="x", offset=0, datatype=PointField.FLOAT32),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32),
        ],
        is_bigendian=False,
        point_step=12,
        data=data,
    )


def make_roi_message(*, stamp_sec: int = 2) -> FrontFaceROIArray:
    structures = []
    for label in (StructureROI.LEFT_POST, StructureROI.RIGHT_POST, StructureROI.TOP_BEAM):
        structures.append(
            StructureROI(
                object_id=7,
                class_id=3,
                structure_label=label,
                mask_origin_x=8,
                mask_origin_y=18,
                mask_width=8,
                mask_height=8,
                roi_mask=[255] * 64,
                line_u0=8.0,
                line_v0=18.0,
                line_u1=16.0,
                line_v1=26.0,
            )
        )
    return FrontFaceROIArray(
        header=Header(stamp=Stamp(sec=stamp_sec), frame_id="camera"),
        objects=[
            FrontFaceROI(
                object_id=7,
                class_id=3,
                bbox_xyxy=[8.0, 18.0, 16.0, 26.0],
                structures=structures,
            )
        ],
    )


def drive_solver_callbacks(node) -> None:
    node.t_cb = np.zeros(3, dtype=np.float64)
    node.R_cb = np.eye(3, dtype=np.float64)
    node.min_points = 1

    pointcloud = make_pointcloud([(0.0, 0.0, 5.0), (0.04, 0.04, 5.0)], stamp_sec=2, frame_id="map")
    odom = Odometry(header=Header(stamp=Stamp(sec=2), frame_id="map"))
    camera_info = CameraInfo(width=640, height=480, k=[120.0, 0.0, 10.0, 0.0, 130.0, 20.0, 0.0, 0.0, 1.0])
    image = Image(width=640, height=480)

    node.cloud_callback(pointcloud)
    node.odom_callback(odom)
    node.camera_info_callback(camera_info)
    node.image_callback(image)
    node.roi_callback(make_roi_message(stamp_sec=2))


def test_node_loads_camera_offsets_from_roi_parameters() -> None:
    module, _lookback_module = load_solver_node_module()
    FakeNode.parameter_overrides = {
        "camera_extrinsic_t": [0.11, 0.22, 0.33],
        "camera_extrinsic_r": [
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            -1.0,
            0.0,
            1.0,
            0.0,
        ],
    }

    try:
        node = module.CornerLidarSolverNode()
    finally:
        FakeNode.parameter_overrides = {}

    np.testing.assert_allclose(node.t_cb, np.asarray([0.11, 0.22, 0.33], dtype=np.float64))
    np.testing.assert_allclose(
        node.R_cb,
        np.asarray(
            [
                [1.0, 0.0, 0.0],
                [0.0, 0.0, -1.0],
                [0.0, 1.0, 0.0],
            ],
            dtype=np.float64,
        ),
    )
    assert any("Loaded solver camera extrinsics from ROI parameters" in record for record in node.get_logger().records)


def test_node_declares_lookback_window_parameters() -> None:
    module, _ = load_solver_node_module()

    node = module.CornerLidarSolverNode()

    assert node.get_parameter("history_window_sec").get_parameter_value().double_value == 1.0
    assert node.get_parameter("max_window_frames").get_parameter_value().integer_value == 30
    assert node.get_parameter("cache_voxel_size").get_parameter_value().double_value == 0.1
    assert node.get_parameter("bbox_expand_ratio").get_parameter_value().double_value == 0.15
    assert node.get_parameter("corner_target_points").get_parameter_value().integer_value == 6
    assert node.get_parameter("corner_target_frames").get_parameter_value().integer_value == 2
    assert node.get_parameter("corner_cap_points").get_parameter_value().integer_value == 96
    assert node.get_parameter("post_max_z_jump_m").get_parameter_value().double_value == 0.8


def test_solver_node_publishes_front_face_outputs() -> None:
    module, _lookback_module = load_solver_node_module()
    node = module.CornerLidarSolverNode()
    module.restore_front_face = lambda track, width_m, height_m: types.SimpleNamespace(
        valid=True,
        solution_state="tracking",
        top_left=(-0.5, -0.2, 2.0),
        top_right=(0.5, -0.2, 2.1),
        bottom_left=(-0.5, 1.8, 2.0),
        bottom_right=(0.5, 1.8, 2.1),
        top_source="beam",
        tracking_confidence=0.7,
    )
    node.point_pub = FakePublisher("/roi_lidar_corner/front_face_corners")
    node.debug_pub = FakePublisher("/roi_lidar_corner/front_face_debug")
    node.track.left_post.initialized = True
    node.track.left_post.x_state = 0.31
    node.track.left_post.z_state = 3.39
    node.track.left_post.confidence = 0.8
    node.track.right_post.initialized = True
    node.track.right_post.x_state = -0.76
    node.track.right_post.z_state = 3.44
    node.track.right_post.confidence = 0.9
    node.track.top_beam.initialized = True
    node.track.top_beam.y_top_state = 1.24
    node.track.top_beam.confidence = 0.7

    node._publish_solution(Header(stamp=Stamp(sec=2), frame_id="camera"))

    assert node.point_pub.messages[-1].valid is True
    assert node.point_pub.messages[-1].solution_state == node.point_pub.messages[-1].SOLUTION_TRACKING
    assert node.point_pub.messages[-1].top_left.x == -0.5
    assert node.point_pub.messages[-1].bottom_right.y == 1.8
    debug = node.debug_pub.messages[-1]
    assert debug.top_source == "beam"
    assert debug.left_post_valid is True
    assert debug.left_post_x == 0.31
    assert debug.left_post_z == 3.39
    assert debug.left_post_confidence == 0.8
    assert debug.right_post_valid is True
    assert debug.right_post_x == -0.76
    assert debug.right_post_z == 3.44
    assert debug.right_post_confidence == 0.9
    assert debug.top_beam_valid is True
    assert debug.top_beam_y == 1.24
    assert debug.top_beam_confidence == 0.7


def test_solver_node_transforms_output_points_into_body_frame() -> None:
    module, _lookback_module = load_solver_node_module()
    FakeNode.parameter_overrides = {
        "output_frame_id": "body",
        "output_extrinsic_t_x": 1.0,
        "output_extrinsic_t_y": 2.0,
        "output_extrinsic_t_z": 3.0,
        "output_extrinsic_r_00": 0.0,
        "output_extrinsic_r_01": -1.0,
        "output_extrinsic_r_02": 0.0,
        "output_extrinsic_r_10": 1.0,
        "output_extrinsic_r_11": 0.0,
        "output_extrinsic_r_12": 0.0,
        "output_extrinsic_r_20": 0.0,
        "output_extrinsic_r_21": 0.0,
        "output_extrinsic_r_22": 1.0,
    }
    node = module.CornerLidarSolverNode()
    module.restore_front_face = lambda track, width_m, height_m: types.SimpleNamespace(
        valid=True,
        solution_state="tracking",
        top_left=(1.0, 0.0, 0.0),
        top_right=(0.0, 1.0, 0.0),
        bottom_left=(1.0, 2.0, 0.0),
        bottom_right=(0.0, 3.0, 0.0),
        top_source="beam",
        tracking_confidence=1.0,
    )
    node.point_pub = FakePublisher("/roi_lidar_corner/front_face_corners")
    node.debug_pub = FakePublisher("/roi_lidar_corner/front_face_debug")

    node._publish_solution(Header(stamp=Stamp(sec=2), frame_id="camera"))

    assert node.point_pub.messages[-1].header.frame_id == "body"
    assert node.point_pub.messages[-1].top_left.x == 1.0
    assert node.point_pub.messages[-1].top_left.y == 3.0
    assert node.point_pub.messages[-1].top_left.z == 3.0
    assert node.point_pub.messages[-1].top_right.x == 0.0
    assert node.point_pub.messages[-1].top_right.y == 2.0
    assert node.point_pub.messages[-1].top_right.z == 3.0


def test_cloud_callback_caches_decoded_cloud_frame() -> None:
    module, lookback_module = load_solver_node_module()
    node = module.CornerLidarSolverNode()

    pointcloud = make_pointcloud([(0.1, 0.0, 0.0), (1.0, 0.0, 0.0)], frame_id="base_link")

    node.cloud_callback(pointcloud)

    cached = node.cloud_buffer[-1]
    assert isinstance(cached, lookback_module.DecodedCloudFrame)
    assert cached.stamp == 1.0
    assert cached.frame_id == "base_link"
    np.testing.assert_allclose(cached.points_world_f32[:, :3], np.array([[1.0, 0.0, 0.0]], dtype=np.float32))


def test_cloud_callback_skips_origin_range_filter_for_world_frames() -> None:
    module, lookback_module = load_solver_node_module()
    node = module.CornerLidarSolverNode()
    node.min_range = 0.0
    node.max_range = 10.0
    node.cache_voxel_size = 0.0

    pointcloud = make_pointcloud([(100.0, 0.0, 5.0)], frame_id="map")

    node.cloud_callback(pointcloud)

    cached = node.cloud_buffer[-1]
    assert isinstance(cached, lookback_module.DecodedCloudFrame)
    np.testing.assert_allclose(cached.points_world_f32[:, :3], np.array([[100.0, 0.0, 5.0]], dtype=np.float32))


@pytest.mark.skip(reason="legacy Corner3DArray contract replaced by FrontFaceCorners")
def test_roi_callback_uses_lookback_window_and_publishes_outputs() -> None:
    module, lookback_module = load_solver_node_module()
    node = module.CornerLidarSolverNode()
    node.t_cb = np.zeros(3, dtype=np.float64)
    node.R_cb = np.eye(3, dtype=np.float64)
    node.fx = 120.0
    node.fy = 130.0
    node.cx = 40.0
    node.cy = 50.0
    node.camera_width = 640
    node.camera_height = 480
    node.image_width = 640
    node.image_height = 480
    node.camera_info_ready = True
    node.debug_projected_cloud_stride = 1
    node.min_points = 1
    node.history_window_sec = 0.8
    node.max_window_frames = 8

    pose = module._BufferedPose(
        stamp=2.0,
        t=np.zeros(3, dtype=np.float64),
        R=np.eye(3, dtype=np.float64),
    )
    node.odom_buffer.append(pose)
    frame_a = lookback_module.DecodedCloudFrame(
        stamp=1.7,
        points_world_f32=np.array([[1.0, 0.0, 5.0]], dtype=np.float32),
        frame_id="map",
    )
    frame_b = lookback_module.DecodedCloudFrame(
        stamp=2.0,
        points_world_f32=np.array([[0.0, 1.0, 5.0]], dtype=np.float32),
        frame_id="map",
    )
    node.cloud_buffer.extend([frame_a, frame_b])

    selected_calls = {}
    solve_calls = {}

    def fake_select(frames, stamp, history_window_sec, max_window_frames):
        selected_calls["frames"] = list(frames)
        selected_calls["stamp"] = stamp
        selected_calls["history_window_sec"] = history_window_sec
        selected_calls["max_window_frames"] = max_window_frames
        return [frame_b, frame_a]

    def fake_solve(**kwargs):
        solve_calls.update(kwargs)
        return {
            "corners": {
                (7, 0): np.array([1.0, 2.0, 3.0], dtype=np.float32),
                (7, 1): None,
            },
            "corner_states": {
                (7, 0): types.SimpleNamespace(
                    points_cam_current=np.array([[1.0, 2.0, 3.0], [1.1, 2.1, 3.1]], dtype=np.float32),
                    uv_hits=np.array([[11.0, 21.0], [12.0, 22.0]], dtype=np.float32),
                    support_frames=2,
                ),
                (7, 1): types.SimpleNamespace(
                    points_cam_current=np.zeros((0, 3), dtype=np.float32),
                    uv_hits=np.zeros((0, 2), dtype=np.float32),
                    support_frames=0,
                ),
            },
            "stats": {
                "frames_seen": 2,
                "frames_used": 2,
                "stop_reason": "all_corners_frozen",
                "points_raw_sum": 2,
                "points_after_cache_filter_sum": 2,
                "points_after_bbox_sum": 2,
                "points_after_mask_sum": 2,
                "solve_ms": 0.3,
            },
        }

    module.select_lookback_frames = fake_select
    module.solve_lookback_window = fake_solve

    roi_msg = make_roi_message()
    node.roi_callback(roi_msg)

    assert selected_calls["frames"] == [frame_a, frame_b]
    assert selected_calls["stamp"] == 2.0
    assert selected_calls["history_window_sec"] == 0.8
    assert selected_calls["max_window_frames"] == 8
    assert [frame.stamp for frame in solve_calls["frames"]] == [2.0, 1.7]
    assert [frame.frame_id for frame in solve_calls["frames"]] == ["world", "world"]
    assert solve_calls["pose"] is pose

    point_msg = node.point_pub.messages[-1]
    assert len(point_msg.corners) == 4
    valid_corner = next(corner for corner in point_msg.corners if corner.corner_label == 0)
    assert valid_corner.valid is True
    assert valid_corner.support_point_count == 2
    assert (valid_corner.x, valid_corner.y, valid_corner.z) == (1.0, 2.0, 3.0)

    debug_msg = node.debug_uv_pub.messages[-1]
    debug_payload = json.loads(debug_msg.data)
    assert set(debug_payload) >= {"stamp", "objects", "cloud_uv", "cloud_uv_depth", "stats"}
    assert debug_payload["objects"][0]["corners"][0] == {
        "corner_label": 0,
        "uv": [[11, 21], [12, 22]],
        "valid": True,
        "failure_reason": "",
    }
    assert debug_payload["cloud_uv"] == [[40, 76]]
    assert debug_payload["cloud_uv_depth"] == [5.0]
    assert debug_payload["stats"]["frames_used"] == 2

    diag_msg = node.diag_pub.messages[-1]
    assert "published_corners" in diag_msg.data


def test_roi_callback_keeps_solver_outputs_when_debug_uv_publishing_disabled() -> None:
    module, _lookback_module = load_solver_node_module()
    FakeNode.parameter_overrides = {"publish_debug_uv": False}
    node = module.CornerLidarSolverNode()

    drive_solver_callbacks(node)

    assert node.point_pub.messages
    assert node.diag_pub.messages
    assert node.debug_uv_pub.messages == []


def test_roi_callback_preserves_default_debug_uv_payload_shape() -> None:
    module, _lookback_module = load_solver_node_module()
    node = module.CornerLidarSolverNode()
    node.debug_projected_cloud_stride = 1

    drive_solver_callbacks(node)

    debug_payload = json.loads(node.debug_uv_pub.messages[-1].data)
    assert "stamp" in debug_payload
    assert "objects" in debug_payload
    assert "cloud_uv" in debug_payload
    assert "cloud_uv_depth" in debug_payload
    assert "stats" in debug_payload


@pytest.mark.skip(reason="legacy Corner3DArray contract replaced by FrontFaceCorners")
def test_roi_callback_marks_corner_invalid_when_support_points_below_min_points() -> None:
    module, lookback_module = load_solver_node_module()
    node = module.CornerLidarSolverNode()
    node.fx = 120.0
    node.fy = 130.0
    node.cx = 40.0
    node.cy = 50.0
    node.camera_width = 640
    node.camera_height = 480
    node.image_width = 640
    node.image_height = 480
    node.camera_info_ready = True
    node.min_points = 3

    pose = module._BufferedPose(
        stamp=2.0,
        t=np.zeros(3, dtype=np.float64),
        R=np.eye(3, dtype=np.float64),
    )
    node.odom_buffer.append(pose)
    frame_a = lookback_module.DecodedCloudFrame(
        stamp=2.0,
        points_world_f32=np.array([[1.0, 0.0, 5.0]], dtype=np.float32),
        frame_id="map",
    )
    node.cloud_buffer.append(frame_a)

    def fake_solve(**_kwargs):
        return {
            "corners": {
                (7, 0): np.array([1.0, 2.0, 3.0], dtype=np.float32),
            },
            "corner_states": {
                (7, 0): types.SimpleNamespace(
                    points_cam_current=np.array([[1.0, 2.0, 3.0], [1.1, 2.1, 3.1]], dtype=np.float32),
                    uv_hits=np.array([[11.0, 21.0], [12.0, 22.0]], dtype=np.float32),
                    support_frames=1,
                ),
            },
            "stats": {
                "frames_seen": 1,
                "frames_used": 1,
                "stop_reason": "window_exhausted",
                "points_raw_sum": 1,
                "points_after_cache_filter_sum": 1,
                "points_after_bbox_sum": 1,
                "points_after_mask_sum": 1,
                "solve_ms": 0.2,
            },
        }

    module.solve_lookback_window = fake_solve

    roi_msg = make_roi_message()
    node.roi_callback(roi_msg)

    point_msg = node.point_pub.messages[-1]
    corner0 = next(corner for corner in point_msg.corners if corner.corner_label == 0)
    assert corner0.valid is False
    assert corner0.support_point_count == 0

    debug_payload = json.loads(node.debug_uv_pub.messages[-1].data)
    assert debug_payload["objects"][0]["corners"][0] == {
        "corner_label": 0,
        "uv": [[11, 21], [12, 22]],
        "valid": False,
        "failure_reason": "below_min_points",
    }


@pytest.mark.skip(reason="legacy Corner3DArray contract replaced by FrontFaceCorners")
def test_roi_callback_uses_trimmed_corner_support_metadata_from_helper() -> None:
    module, lookback_module = load_solver_node_module()
    node = module.CornerLidarSolverNode()
    node.fx = 120.0
    node.fy = 130.0
    node.cx = 40.0
    node.cy = 50.0
    node.camera_width = 640
    node.camera_height = 480
    node.image_width = 640
    node.image_height = 480
    node.camera_info_ready = True
    node.min_points = 1

    pose = module._BufferedPose(
        stamp=2.0,
        t=np.zeros(3, dtype=np.float64),
        R=np.eye(3, dtype=np.float64),
    )
    node.odom_buffer.append(pose)
    node.cloud_buffer.append(
        lookback_module.DecodedCloudFrame(
            stamp=2.0,
            points_world_f32=np.array([[1.0, 0.0, 5.0]], dtype=np.float32),
            frame_id="map",
        )
    )

    def fake_solve(**_kwargs):
        return {
            "corners": {(7, 0): np.array([1.1, 2.1, 3.1], dtype=np.float32)},
            "corner_states": {
                (7, 0): types.SimpleNamespace(
                    points_cam_current=np.array(
                        [[1.0, 2.0, 3.0], [1.2, 2.2, 3.2], [30.0, 40.0, 50.0]],
                        dtype=np.float32,
                    ),
                    uv_hits=np.array([[11.0, 21.0], [12.0, 22.0], [99.0, 88.0]], dtype=np.float32),
                    support_frames=1,
                ),
            },
            "corner_support": {
                (7, 0): types.SimpleNamespace(
                    points_cam_current=np.array([[1.0, 2.0, 3.0], [1.2, 2.2, 3.2]], dtype=np.float32),
                    uv_hits=np.array([[11.0, 21.0], [12.0, 22.0]], dtype=np.float32),
                    support_frames=1,
                ),
            },
            "stats": {
                "frames_seen": 1,
                "frames_used": 1,
                "stop_reason": "all_corners_frozen",
                "points_raw_sum": 1,
                "points_after_cache_filter_sum": 1,
                "points_after_bbox_sum": 1,
                "points_after_mask_sum": 1,
                "solve_ms": 0.1,
            },
        }

    module.solve_lookback_window = fake_solve

    node.roi_callback(make_roi_message())

    point_msg = node.point_pub.messages[-1]
    corner0 = next(corner for corner in point_msg.corners if corner.corner_label == 0)
    assert corner0.valid is True
    assert corner0.support_point_count == 2

    debug_payload = json.loads(node.debug_uv_pub.messages[-1].data)
    assert debug_payload["objects"][0]["corners"][0] == {
        "corner_label": 0,
        "uv": [[11, 21], [12, 22]],
        "valid": True,
        "failure_reason": "",
    }


@pytest.mark.skip(reason="legacy Corner3DArray contract replaced by FrontFaceCorners")
def test_roi_callback_includes_per_corner_failure_reason_when_support_state_is_missing() -> None:
    module, lookback_module = load_solver_node_module()
    node = module.CornerLidarSolverNode()
    node.fx = 120.0
    node.fy = 130.0
    node.cx = 40.0
    node.cy = 50.0
    node.camera_width = 640
    node.camera_height = 480
    node.image_width = 640
    node.image_height = 480
    node.camera_info_ready = True
    node.min_points = 1

    pose = module._BufferedPose(
        stamp=2.0,
        t=np.zeros(3, dtype=np.float64),
        R=np.eye(3, dtype=np.float64),
    )
    node.odom_buffer.append(pose)
    node.cloud_buffer.append(
        lookback_module.DecodedCloudFrame(
            stamp=2.0,
            points_world_f32=np.array([[1.0, 0.0, 5.0]], dtype=np.float32),
            frame_id="map",
        )
    )

    def fake_solve(**_kwargs):
        return {
            "corners": {},
            "corner_states": {},
            "stats": {
                "frames_seen": 1,
                "frames_used": 1,
                "stop_reason": "window_exhausted",
                "points_raw_sum": 1,
                "points_after_cache_filter_sum": 1,
                "points_after_bbox_sum": 1,
                "points_after_mask_sum": 1,
                "solve_ms": 0.1,
            },
        }

    module.solve_lookback_window = fake_solve

    node.roi_callback(make_roi_message())

    debug_payload = json.loads(node.debug_uv_pub.messages[-1].data)
    assert debug_payload["objects"][0]["corners"][0] == {
        "corner_label": 0,
        "uv": [],
        "valid": False,
        "failure_reason": "no_state",
    }


@pytest.mark.skip(reason="legacy Corner3DArray contract replaced by FrontFaceCorners")
def test_roi_callback_includes_per_corner_failure_reason_when_support_is_insufficient() -> None:
    module, lookback_module = load_solver_node_module()
    node = module.CornerLidarSolverNode()
    node.fx = 120.0
    node.fy = 130.0
    node.cx = 40.0
    node.cy = 50.0
    node.camera_width = 640
    node.camera_height = 480
    node.image_width = 640
    node.image_height = 480
    node.camera_info_ready = True
    node.min_points = 1
    node.lookback_params = lookback_module.LookbackSolveParams(
        bbox_expand_ratio=node.lookback_params.bbox_expand_ratio,
        corner_target_points=3,
        corner_target_frames=2,
        corner_cap_points=node.lookback_params.corner_cap_points,
        min_range=node.lookback_params.min_range,
        max_range=node.lookback_params.max_range,
    )

    pose = module._BufferedPose(
        stamp=2.0,
        t=np.zeros(3, dtype=np.float64),
        R=np.eye(3, dtype=np.float64),
    )
    node.odom_buffer.append(pose)
    node.cloud_buffer.append(
        lookback_module.DecodedCloudFrame(
            stamp=2.0,
            points_world_f32=np.array([[1.0, 0.0, 5.0]], dtype=np.float32),
            frame_id="map",
        )
    )

    def fake_solve(**_kwargs):
        return {
            "corners": {},
            "corner_states": {
                (7, 0): types.SimpleNamespace(
                    points_cam_current=np.array([[1.0, 2.0, 3.0], [1.1, 2.1, 3.1]], dtype=np.float32),
                    uv_hits=np.array([[11.0, 21.0], [12.0, 22.0]], dtype=np.float32),
                    support_frames=1,
                ),
            },
            "stats": {
                "frames_seen": 1,
                "frames_used": 1,
                "stop_reason": "window_exhausted",
                "points_raw_sum": 1,
                "points_after_cache_filter_sum": 1,
                "points_after_bbox_sum": 1,
                "points_after_mask_sum": 1,
                "solve_ms": 0.1,
            },
        }

    module.solve_lookback_window = fake_solve

    node.roi_callback(make_roi_message())

    debug_payload = json.loads(node.debug_uv_pub.messages[-1].data)
    assert debug_payload["objects"][0]["corners"][0] == {
        "corner_label": 0,
        "uv": [[11, 21], [12, 22]],
        "valid": False,
        "failure_reason": "insufficient_support_points_and_frames",
    }


@pytest.mark.skip(reason="legacy lookback helper contract replaced by structure tracking")
def test_roi_callback_treats_camera_init_as_world_frame_in_auto_mode() -> None:
    module, lookback_module = load_solver_node_module()
    node = module.CornerLidarSolverNode()
    node.t_cb = np.zeros(3, dtype=np.float64)
    node.R_cb = np.eye(3, dtype=np.float64)
    node.cloud_frame_mode = "auto"
    node.fx = 120.0
    node.fy = 130.0
    node.cx = 40.0
    node.cy = 50.0
    node.camera_width = 640
    node.camera_height = 480
    node.image_width = 640
    node.image_height = 480
    node.camera_info_ready = True
    node.debug_projected_cloud_stride = 1
    node.min_points = 1

    pose = module._BufferedPose(
        stamp=2.0,
        t=np.array([1.0, 0.0, 0.0], dtype=np.float64),
        R=np.eye(3, dtype=np.float64),
    )
    node.odom_buffer.append(pose)
    frame = lookback_module.DecodedCloudFrame(
        stamp=2.0,
        points_world_f32=np.array([[1.0, 0.0, 5.0]], dtype=np.float32),
        frame_id="camera_init",
    )
    node.cloud_buffer.append(frame)

    solve_calls = {}

    def fake_solve(**kwargs):
        solve_calls.update(kwargs)
        return {
            "corners": {(7, 0): np.array([0.0, 0.0, 5.0], dtype=np.float32)},
            "corner_states": {
                (7, 0): types.SimpleNamespace(
                    points_cam_current=np.array([[0.0, 0.0, 5.0]], dtype=np.float32),
                    uv_hits=np.array([[40.0, 50.0]], dtype=np.float32),
                    support_frames=1,
                )
            },
            "stats": {
                "frames_seen": 1,
                "frames_used": 1,
                "stop_reason": "all_corners_frozen",
                "points_raw_sum": 1,
                "points_after_cache_filter_sum": 1,
                "points_after_bbox_sum": 1,
                "points_after_mask_sum": 1,
                "solve_ms": 0.1,
            },
        }

    module.solve_lookback_window = fake_solve

    node.roi_callback(make_roi_message())

    assert solve_calls["frames"][0].frame_id == "world"
    debug_payload = json.loads(node.debug_uv_pub.messages[-1].data)
    assert debug_payload["cloud_uv"] == [[40, 50]]
    assert debug_payload["cloud_uv_depth"] == [5.0]


@pytest.mark.skip(reason="legacy Corner3DArray contract replaced by FrontFaceCorners")
def test_roi_callback_rejects_stale_cloud_window_when_latest_cloud_exceeds_threshold() -> None:
    module, lookback_module = load_solver_node_module()
    node = module.CornerLidarSolverNode()
    node.fx = 120.0
    node.fy = 130.0
    node.cx = 40.0
    node.cy = 50.0
    node.camera_width = 640
    node.camera_height = 480
    node.image_width = 640
    node.image_height = 480
    node.camera_info_ready = True
    node.history_window_sec = 2.0
    node.max_time_diff_cloud = 0.25
    node.min_points = 1

    pose = module._BufferedPose(
        stamp=2.0,
        t=np.zeros(3, dtype=np.float64),
        R=np.eye(3, dtype=np.float64),
    )
    node.odom_buffer.append(pose)
    node.cloud_buffer.append(
        lookback_module.DecodedCloudFrame(
            stamp=1.0,
            points_world_f32=np.array([[1.0, 0.0, 5.0]], dtype=np.float32),
            frame_id="map",
        )
    )

    solve_called = {"value": False}

    def fake_solve(**_kwargs):
        solve_called["value"] = True
        return {}

    module.solve_lookback_window = fake_solve

    node.roi_callback(make_roi_message())

    assert solve_called["value"] is False
    assert node.point_pub.messages[-1].corners == []
    assert "no_cloud_near_stamp" in node.diag_pub.messages[-1].data
    debug_payload = json.loads(node.debug_uv_pub.messages[-1].data)
    assert debug_payload["stats"]["stop_reason"] == "no_cloud_near_stamp"


@pytest.mark.skip(reason="legacy Corner3DArray contract replaced by FrontFaceCorners")
def test_roi_callback_early_return_publishes_empty_corner_array() -> None:
    module, lookback_module = load_solver_node_module()
    node = module.CornerLidarSolverNode()
    node.fx = 120.0
    node.fy = 130.0
    node.cx = 40.0
    node.cy = 50.0
    node.camera_width = 640
    node.camera_height = 480
    node.image_width = 640
    node.image_height = 480
    node.camera_info_ready = True
    node.min_points = 1

    pose = module._BufferedPose(
        stamp=2.0,
        t=np.zeros(3, dtype=np.float64),
        R=np.eye(3, dtype=np.float64),
    )
    node.odom_buffer.append(pose)
    node.cloud_buffer.append(
        lookback_module.DecodedCloudFrame(
            stamp=2.0,
            points_world_f32=np.array([[1.0, 0.0, 5.0]], dtype=np.float32),
            frame_id="map",
        )
    )

    def fake_solve(**_kwargs):
        return {
            "corners": {(7, 0): np.array([1.0, 2.0, 3.0], dtype=np.float32)},
            "corner_states": {
                (7, 0): types.SimpleNamespace(
                    points_cam_current=np.array([[1.0, 2.0, 3.0]], dtype=np.float32),
                    uv_hits=np.array([[11.0, 21.0]], dtype=np.float32),
                    support_frames=1,
                )
            },
            "stats": {
                "frames_seen": 1,
                "frames_used": 1,
                "stop_reason": "all_corners_frozen",
                "points_raw_sum": 1,
                "points_after_cache_filter_sum": 1,
                "points_after_bbox_sum": 1,
                "points_after_mask_sum": 1,
                "solve_ms": 0.1,
            },
        }

    module.solve_lookback_window = fake_solve

    roi_msg = make_roi_message()
    node.roi_callback(roi_msg)
    assert len(node.point_pub.messages) == 1
    assert any(corner.valid for corner in node.point_pub.messages[-1].corners)

    node.odom_buffer.clear()
    node.roi_callback(roi_msg)

    assert len(node.point_pub.messages) == 2
    assert node.point_pub.messages[-1].header == roi_msg.header
    assert node.point_pub.messages[-1].corners == []
