from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import types

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = PACKAGE_ROOT / "roi_lidar_corner" / "roi_generator_node.py"


def _install_module(name: str, module: types.ModuleType) -> None:
    sys.modules[name] = module


def _load_generator_module():
    for name in [
        "roi_generator_node_under_test",
        "ament_index_python",
        "ament_index_python.packages",
        "cv2",
        "rclpy",
        "rclpy.node",
        "rclpy.qos",
        "cv_bridge",
        "sensor_msgs",
        "sensor_msgs.msg",
        "std_msgs",
        "std_msgs.msg",
        "rotor_swarm_msgs",
        "rotor_swarm_msgs.msg",
        "roi_lidar_corner",
        "roi_lidar_corner.detection_runtime",
        "roi_lidar_corner.msg",
        "roi_lidar_corner.neo_roi_refiner",
        "roi_lidar_corner.structure_roi_builder",
    ]:
        sys.modules.pop(name, None)

    cv2 = types.ModuleType("cv2")
    cv2.FONT_HERSHEY_SIMPLEX = 0
    cv2.RETR_EXTERNAL = 0
    cv2.CHAIN_APPROX_SIMPLE = 0
    cv2.circle = lambda *_args, **_kwargs: None
    cv2.drawContours = lambda *_args, **_kwargs: None
    cv2.findContours = lambda *_args, **_kwargs: ([], None)
    def fake_line(image, start, end, value, thickness=1, *_args, **_kwargs):
        if not isinstance(image, np.ndarray):
            return None
        x0, y0 = start
        x1, y1 = end
        half = max(0, int(thickness) // 2)
        if x0 == x1:
            x = max(0, min(image.shape[1] - 1, int(x0)))
            ya = max(0, min(image.shape[0] - 1, min(int(y0), int(y1))))
            yb = max(0, min(image.shape[0] - 1, max(int(y0), int(y1))))
            image[ya : yb + 1, max(0, x - half) : min(image.shape[1], x + half + 1)] = value
        elif y0 == y1:
            y = max(0, min(image.shape[0] - 1, int(y0)))
            xa = max(0, min(image.shape[1] - 1, min(int(x0), int(x1))))
            xb = max(0, min(image.shape[1] - 1, max(int(x0), int(x1))))
            image[max(0, y - half) : min(image.shape[0], y + half + 1), xa : xb + 1] = value
        return None

    cv2.line = fake_line
    cv2.rectangle = lambda *_args, **_kwargs: None
    cv2.put_text_calls = []

    def fake_put_text(_image, text, *_args, **_kwargs):
        cv2.put_text_calls.append(text)

    cv2.putText = fake_put_text
    _install_module("cv2", cv2)

    ament_index_python = types.ModuleType("ament_index_python")
    ament_packages = types.ModuleType("ament_index_python.packages")
    ament_packages.get_package_share_directory = lambda package_name: f"/fake/share/{package_name}"
    _install_module("ament_index_python", ament_index_python)
    _install_module("ament_index_python.packages", ament_packages)

    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_qos = types.ModuleType("rclpy.qos")

    class FakeNode:
        parameter_overrides = {}

        def __init__(self, _name: str) -> None:
            self.parameters = {}
            self.publishers = []
            self.subscriptions = []
            self.logger = types.SimpleNamespace(
                error=lambda *_args, **_kwargs: None,
                info=lambda *_args, **_kwargs: None,
                warn=lambda *_args, **_kwargs: None,
            )

        def declare_parameter(self, name: str, default_value):
            self.parameters[name] = self.parameter_overrides.get(name, default_value)

        def get_parameter(self, name: str):
            value = self.parameters[name]

            class FakeParameter:
                def get_parameter_value(self):
                    return types.SimpleNamespace(
                        bool_value=bool(value),
                        double_value=float(value) if isinstance(value, (int, float)) else 0.0,
                        integer_value=int(value) if isinstance(value, (int, float)) else 0,
                        string_value=str(value),
                    )

            return FakeParameter()

        def create_publisher(self, msg_type, topic: str, qos):
            publisher = types.SimpleNamespace(msg_type=msg_type, topic=topic, qos=qos, published=[])
            publisher.publish = publisher.published.append
            self.publishers.append(publisher)
            return publisher

        def create_subscription(self, msg_type, topic: str, callback, qos):
            subscription = types.SimpleNamespace(
                msg_type=msg_type,
                topic=topic,
                callback=callback,
                qos=qos,
            )
            self.subscriptions.append(subscription)
            return subscription

        def get_logger(self):
            return self.logger

    rclpy_node.Node = FakeNode
    rclpy_qos.qos_profile_sensor_data = object()
    _install_module("rclpy", rclpy)
    _install_module("rclpy.node", rclpy_node)
    _install_module("rclpy.qos", rclpy_qos)

    cv_bridge = types.ModuleType("cv_bridge")

    class FakeCvBridge:
        def imgmsg_to_cv2(self, msg, desired_encoding="bgr8"):
            return msg.cv_image

        def cv2_to_imgmsg(self, cv_image, encoding="bgr8"):
            return types.SimpleNamespace(cv_image=cv_image, encoding=encoding)

    cv_bridge.CvBridge = FakeCvBridge
    _install_module("cv_bridge", cv_bridge)

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")

    class FakeImage:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=types.SimpleNamespace(sec=0, nanosec=0))

    sensor_msgs_msg.Image = FakeImage
    _install_module("sensor_msgs", sensor_msgs)
    _install_module("sensor_msgs.msg", sensor_msgs_msg)

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")

    class FakeHeader:
        def __init__(self):
            self.stamp = types.SimpleNamespace(sec=0, nanosec=0)

    class FakeString:
        def __init__(self):
            self.data = ""

    std_msgs_msg.Header = FakeHeader
    std_msgs_msg.String = FakeString
    _install_module("std_msgs", std_msgs)
    _install_module("std_msgs.msg", std_msgs_msg)

    roi_pkg = types.ModuleType("roi_lidar_corner")
    roi_pkg.__path__ = [str(PACKAGE_ROOT / "roi_lidar_corner")]
    detection_runtime = types.ModuleType("roi_lidar_corner.detection_runtime")

    class FakeDetection:
        pass

    detection_runtime.Detection = FakeDetection
    detection_runtime.create_detector = lambda **_: None
    detection_runtime.resolve_resource_path = lambda value, *_args, **_kwargs: value
    _install_module("roi_lidar_corner", roi_pkg)
    _install_module("roi_lidar_corner.detection_runtime", detection_runtime)

    roi_msg = types.ModuleType("roi_lidar_corner.msg")

    class FakeCorner3DArray:
        def __init__(self):
            self.header = FakeHeader()
            self.corners = []

    class FakeCornerROI:
        pass

    class FakeObjectROI:
        def __init__(self):
            self.corner_rois = []

    class FakeObjectROIArray:
        def __init__(self):
            self.objects = []

    class FakeStructureROI:
        LEFT_POST = 0
        RIGHT_POST = 1
        TOP_BEAM = 2

        def __init__(self):
            self.header = None
            self.object_id = 0
            self.class_id = 0
            self.conf = 0.0
            self.structure_label = 0
            self.mask_origin_x = 0
            self.mask_origin_y = 0
            self.mask_width = 0
            self.mask_height = 0
            self.roi_mask = []
            self.line_u0 = 0.0
            self.line_v0 = 0.0
            self.line_u1 = 0.0
            self.line_v1 = 0.0
            self.valid = False
            self.structure_conf = 0.0
            self.source = ""

    class FakeFrontFaceROI:
        def __init__(self):
            self.header = None
            self.object_id = 0
            self.class_id = 0
            self.conf = 0.0
            self.bbox_xyxy = []
            self.structures = []

    class FakeFrontFaceROIArray:
        def __init__(self):
            self.header = FakeHeader()
            self.objects = []

    roi_msg.Corner3DArray = FakeCorner3DArray
    roi_msg.CornerROI = FakeCornerROI
    roi_msg.ObjectROI = FakeObjectROI
    roi_msg.ObjectROIArray = FakeObjectROIArray
    roi_msg.StructureROI = FakeStructureROI
    roi_msg.FrontFaceROI = FakeFrontFaceROI
    roi_msg.FrontFaceROIArray = FakeFrontFaceROIArray
    _install_module("roi_lidar_corner.msg", roi_msg)

    rotor_swarm_msgs = types.ModuleType("rotor_swarm_msgs")
    rotor_swarm_msgs_msg = types.ModuleType("rotor_swarm_msgs.msg")

    class FakePoint:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class FakeFrontFaceCorners:
        CORNER_INVALID = 2
        SOLUTION_INVALID = 0

        def __init__(self):
            self.header = FakeHeader()
            self.valid = False
            self.solution_state = self.SOLUTION_INVALID
            self.tracking_confidence = 0.0
            self.top_source = ""
            self.top_left = FakePoint()
            self.top_right = FakePoint()
            self.bottom_left = FakePoint()
            self.bottom_right = FakePoint()
            self.top_left_status = self.CORNER_INVALID
            self.top_right_status = self.CORNER_INVALID
            self.bottom_left_status = self.CORNER_INVALID
            self.bottom_right_status = self.CORNER_INVALID

    rotor_swarm_msgs_msg.FrontFaceCorners = FakeFrontFaceCorners
    _install_module("rotor_swarm_msgs", rotor_swarm_msgs)
    _install_module("rotor_swarm_msgs.msg", rotor_swarm_msgs_msg)

    neo_roi_refiner = types.ModuleType("roi_lidar_corner.neo_roi_refiner")
    neo_roi_refiner.refine_corners_inside_bbox = lambda *_args, **_kwargs: []
    _install_module("roi_lidar_corner.neo_roi_refiner", neo_roi_refiner)

    spec = importlib.util.spec_from_file_location("roi_generator_node_under_test", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    _install_module("roi_generator_node_under_test", module)
    spec.loader.exec_module(module)
    return module


def test_scaled_corner_radius_allows_large_debug_radius_over_previous_cap() -> None:
    module = _load_generator_module()

    scaled = module._scaled_corner_radius((0.0, 0.0, 245.0, 245.0), 40)

    assert scaled == 40


def test_debug_subscriptions_are_not_created_when_debug_image_is_disabled() -> None:
    module = _load_generator_module()
    module.Node.parameter_overrides = {"publish_debug_image": False}

    node = module.RoiGeneratorNode()

    subscribed_topics = [subscription.topic for subscription in node.subscriptions]
    assert "/camera/color/image_raw" in subscribed_topics
    assert "/roi_lidar_corner/corners3d" not in subscribed_topics
    assert "/roi_lidar_corner/solver_debug_uv" not in subscribed_topics


def test_publishes_structure_rois_from_refined_corners() -> None:
    module = _load_generator_module()
    module.refine_corners_inside_bbox = lambda *_args, **_kwargs: [
        (10.0, 20.0),
        (30.0, 20.0),
        (10.0, 60.0),
        (30.0, 60.0),
    ]

    detection = types.SimpleNamespace(
        bbox=(10.0, 20.0, 30.0, 60.0),
        class_id=3,
        conf=0.9,
        class_name="frame",
    )
    node = module.RoiGeneratorNode()
    node.detector = types.SimpleNamespace(
        available=True,
        detect=lambda _image: types.SimpleNamespace(detections=[detection]),
    )
    image_msg = types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=1, nanosec=0)),
        cv_image=np.zeros((80, 80, 3), dtype=np.uint8),
    )

    node.image_callback(image_msg)

    published = node.publisher.published[-1]
    structures = published.objects[0].structures
    assert [item.structure_label for item in structures] == [0, 1, 2]
    assert structures[2].line_v0 == 20.0
    assert structures[2].line_v1 == 20.0
    assert structures[2].mask_width > 0
    assert structures[2].mask_height > 0


def test_corner3d_detail_text_uses_corner_freshness_without_solver_debug_freshness() -> None:
    module = _load_generator_module()
    module.Node.parameter_overrides = {}
    node = module.RoiGeneratorNode()
    image_msg = module.Image()
    image_msg.header.stamp.sec = 12
    image_msg.header.stamp.nanosec = 500_000_000

    corner_roi = module.CornerROI()
    corner_roi.corner_label = 0
    corner_roi.corner_u = 20.0
    corner_roi.corner_v = 30.0
    corner_roi.mask_origin_x = 20
    corner_roi.mask_origin_y = 30
    corner_roi.mask_width = 1
    corner_roi.mask_height = 1
    corner_roi.roi_mask = [255]
    obj = module.ObjectROI()
    obj.object_id = 0
    obj.corner_rois.append(corner_roi)
    corner3d = types.SimpleNamespace(
        valid=True,
        support_point_count=7,
        x=1.25,
        y=2.5,
        z=3.75,
    )
    node.latest_corner3d = {(0, 0): corner3d}
    node.latest_corner3d_stamp = 12.5
    node.latest_solver_debug_uv_stamp = None

    draw_solver_debug = node._is_solver_debug_fresh(image_msg)
    draw_corner3d_debug = node._is_corner3d_fresh(image_msg)
    node._draw_corner_rois(
        np.zeros((80, 80, 3), dtype=np.uint8),
        [obj],
        draw_solver_debug,
        draw_corner3d_debug,
    )

    assert any("xyz=(1.25, 2.50, 3.75)" in text for text in module.cv2.put_text_calls)
    assert not any("xyz=(?, ?, ?)" in text for text in module.cv2.put_text_calls)


def test_build_front_face_roi_marks_structure_source() -> None:
    module = _load_generator_module()
    node = module.RoiGeneratorNode()
    detection = types.SimpleNamespace(
        bbox=(10.0, 20.0, 110.0, 220.0),
        class_id=3,
        conf=0.9,
        class_name="gate",
    )

    roi = node._build_front_face_roi(
        header=module.Header(),
        object_id=0,
        detection=detection,
        corners=[(10.0, 20.0), (110.0, 20.0), (10.0, 220.0), (110.0, 220.0)],
        image_shape=(480, 640, 3),
        source="bbox_fallback",
    )

    assert roi is not None
    assert {structure.source for structure in roi.structures} == {"bbox_fallback"}


def test_invalid_refined_geometry_falls_back_to_bbox_structures() -> None:
    module = _load_generator_module()
    module.refine_corners_inside_bbox = lambda *_args, **_kwargs: [
        (10.0, 20.0),
        (110.0, 20.0),
        (10.0, 24.0),
        (110.0, 220.0),
    ]
    module.Node.parameter_overrides = {
        "roi_enable_geometry_prior": True,
        "roi_enable_temporal_prior": False,
    }
    detection = types.SimpleNamespace(
        bbox=(10.0, 20.0, 110.0, 220.0),
        class_id=3,
        conf=0.9,
        class_name="gate",
    )
    node = module.RoiGeneratorNode()
    node.detector = types.SimpleNamespace(
        available=True,
        detect=lambda _image: types.SimpleNamespace(detections=[detection]),
    )
    image_msg = types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=1, nanosec=0)),
        cv_image=np.zeros((480, 640, 3), dtype=np.uint8),
    )

    node.image_callback(image_msg)

    structures = node.publisher.published[-1].objects[0].structures
    assert {structure.source for structure in structures} == {"bbox_fallback"}
    left = next(item for item in structures if item.structure_label == 0)
    assert left.line_v0 == 20.0
    assert left.line_v1 == 220.0
