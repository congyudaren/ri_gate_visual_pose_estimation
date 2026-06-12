from __future__ import annotations

from pathlib import Path
from array import array

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header

from roi_lidar_fixture.static_scene_fixture_data import (
    DEFAULT_CAMERA_EXTRINSIC_R,
    DEFAULT_CAMERA_EXTRINSIC_T,
    DEFAULT_INTRINSICS,
    CameraIntrinsics,
    FixtureAssets,
    camera_points_to_body,
    default_assets,
    load_fixture_assets,
    points_to_xyz32_bytes,
    rotate_uv_180,
    uv_depth_to_xyz_cam,
)


def _header(stamp: Time, frame_id: str) -> Header:
    msg = Header()
    msg.stamp = stamp
    msg.frame_id = frame_id
    return msg


def _make_image_msg(image_bgr: np.ndarray, stamp: Time, frame_id: str) -> Image:
    image = np.asarray(image_bgr, dtype=np.uint8)
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f"expected BGR image with shape HxWx3, got {image.shape}")
    msg = Image()
    msg.header = _header(stamp, frame_id)
    msg.height = int(image.shape[0])
    msg.width = int(image.shape[1])
    msg.encoding = "bgr8"
    msg.is_bigendian = False
    msg.step = int(image.shape[1] * 3)
    msg.data = array("B", np.ascontiguousarray(image).tobytes())
    return msg


def _make_camera_info_msg(intrinsics: CameraIntrinsics, stamp: Time, frame_id: str) -> CameraInfo:
    msg = CameraInfo()
    msg.header = _header(stamp, frame_id)
    msg.width = int(intrinsics.width)
    msg.height = int(intrinsics.height)
    msg.distortion_model = "plumb_bob"
    msg.d = []
    msg.k = [
        float(intrinsics.fx),
        0.0,
        float(intrinsics.cx),
        0.0,
        float(intrinsics.fy),
        float(intrinsics.cy),
        0.0,
        0.0,
        1.0,
    ]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [
        float(intrinsics.fx),
        0.0,
        float(intrinsics.cx),
        0.0,
        0.0,
        float(intrinsics.fy),
        float(intrinsics.cy),
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
    ]
    return msg


def _make_pointcloud_msg(points_body: np.ndarray, stamp: Time, frame_id: str) -> PointCloud2:
    payload, point_count = points_to_xyz32_bytes(points_body)
    msg = PointCloud2()
    msg.header = _header(stamp, frame_id)
    msg.height = 1
    msg.width = int(point_count)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = int(point_count * msg.point_step)
    msg.data = payload
    msg.is_dense = True
    return msg


def _make_identity_odom_msg(stamp: Time, frame_id: str, child_frame_id: str) -> Odometry:
    msg = Odometry()
    msg.header = _header(stamp, frame_id)
    msg.child_frame_id = child_frame_id
    msg.pose.pose.orientation.w = 1.0
    return msg


class StaticSceneFixturePublisher(Node):
    def __init__(self) -> None:
        super().__init__("static_scene_fixture_publisher")

        assets = default_assets()
        self.declare_parameter("image_path", str(assets.image_path))
        self.declare_parameter("points_path", str(assets.points_path))
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("rotate_180", True)
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("pointcloud_topic", "/cloud_registered")
        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("camera_frame_id", "camera_color_optical_frame")
        self.declare_parameter("cloud_frame_id", "body")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("odom_child_frame_id", "body")
        self.declare_parameter("fx", DEFAULT_INTRINSICS.fx)
        self.declare_parameter("fy", DEFAULT_INTRINSICS.fy)
        self.declare_parameter("cx", DEFAULT_INTRINSICS.cx)
        self.declare_parameter("cy", DEFAULT_INTRINSICS.cy)

        image_path = Path(self.get_parameter("image_path").get_parameter_value().string_value)
        points_path = Path(self.get_parameter("points_path").get_parameter_value().string_value)
        self.publish_rate_hz = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self.rotate_180 = bool(self.get_parameter("rotate_180").value)
        self.camera_frame_id = self.get_parameter("camera_frame_id").get_parameter_value().string_value
        self.cloud_frame_id = self.get_parameter("cloud_frame_id").get_parameter_value().string_value
        self.odom_frame_id = self.get_parameter("odom_frame_id").get_parameter_value().string_value
        self.odom_child_frame_id = self.get_parameter("odom_child_frame_id").get_parameter_value().string_value

        self.fixture = load_fixture_assets(FixtureAssets(image_path=image_path, points_path=points_path))
        self.intrinsics = CameraIntrinsics(
            width=int(self.fixture.image_bgr.shape[1]),
            height=int(self.fixture.image_bgr.shape[0]),
            fx=float(self.get_parameter("fx").value),
            fy=float(self.get_parameter("fy").value),
            cx=float(self.get_parameter("cx").value),
            cy=float(self.get_parameter("cy").value),
        )
        uv = self.fixture.uv
        if self.rotate_180:
            uv = rotate_uv_180(uv, width=self.intrinsics.width, height=self.intrinsics.height)
        xyz_cam = uv_depth_to_xyz_cam(uv=uv, depth=self.fixture.depth, intrinsics=self.intrinsics)
        self.points_body = camera_points_to_body(
            xyz_cam=xyz_cam,
            t_cb=DEFAULT_CAMERA_EXTRINSIC_T,
            r_cb=DEFAULT_CAMERA_EXTRINSIC_R,
        )

        self.image_pub = self.create_publisher(
            Image,
            self.get_parameter("image_topic").get_parameter_value().string_value,
            10,
        )
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            self.get_parameter("camera_info_topic").get_parameter_value().string_value,
            10,
        )
        self.cloud_pub = self.create_publisher(
            PointCloud2,
            self.get_parameter("pointcloud_topic").get_parameter_value().string_value,
            10,
        )
        self.odom_pub = self.create_publisher(
            Odometry,
            self.get_parameter("odom_topic").get_parameter_value().string_value,
            10,
        )
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_once)
        self.get_logger().info(
            "publishing static scene fixture "
            f"points={self.points_body.shape[0]} image={self.intrinsics.width}x{self.intrinsics.height}"
        )

    def publish_once(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(_make_image_msg(self.fixture.image_bgr, stamp, self.camera_frame_id))
        self.camera_info_pub.publish(_make_camera_info_msg(self.intrinsics, stamp, self.camera_frame_id))
        self.cloud_pub.publish(_make_pointcloud_msg(self.points_body, stamp, self.cloud_frame_id))
        self.odom_pub.publish(_make_identity_odom_msg(stamp, self.odom_frame_id, self.odom_child_frame_id))


def main() -> None:
    rclpy.init()
    node = StaticSceneFixturePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
