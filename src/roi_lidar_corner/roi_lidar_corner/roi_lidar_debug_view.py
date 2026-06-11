#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import rclpy
from rclpy.node import Node


class RoiLidarDebugViewNode(Node):
    def __init__(self) -> None:
        super().__init__("roi_lidar_debug_view")

        self.declare_parameter("enabled", True)
        self.declare_parameter("image_topic", "/roi_lidar_corner/roi_debug")
        self.declare_parameter("window_name", "roi_lidar_debug")
        self.declare_parameter("window_wait_ms", 1)

        self.enabled = self.get_parameter("enabled").get_parameter_value().bool_value
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.window_name = self.get_parameter("window_name").get_parameter_value().string_value
        self.window_wait_ms = int(self.get_parameter("window_wait_ms").get_parameter_value().integer_value)

        self.bridge = CvBridge()
        self._running = self.enabled

        if not self.enabled:
            self.get_logger().info("debug image viewer disabled")
            return

        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.get_logger().info(f"opening debug image window: topic={self.image_topic}, window={self.window_name}")
        self.get_logger().info("press 'q' in image window to close")

    def image_callback(self, msg: Image) -> None:
        if not self._running:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"image decode failed: {exc}")
            return

        try:
            cv2.imshow(self.window_name, frame)
            key = cv2.waitKey(max(1, self.window_wait_ms)) & 0xFF
        except Exception as exc:
            self.get_logger().warn(f"open debug window failed: {exc}")
            self.get_logger().warn("viewer disabled after initialization failure")
            self._running = False
            return

        if key in (ord("q"), 27):
            self.get_logger().info("exit key received, closing debug window")
            self._running = False
            cv2.destroyWindow(self.window_name)


def main() -> None:
    rclpy.init()
    node: Optional[RoiLidarDebugViewNode] = None
    try:
        node = RoiLidarDebugViewNode()
        if node.enabled:
            rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if node is not None and node.enabled:
            cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
