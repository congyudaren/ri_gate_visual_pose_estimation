#!/usr/bin/env python3

from __future__ import annotations

from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
import rclpy
from visualization_msgs.msg import Marker, MarkerArray

from rotor_swarm_msgs.msg import FrontFaceCorners


class RoiLidarDebugMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__("roi_lidar_debug_markers")

        self.declare_parameter("corner_topic", "/roi_lidar_corner/front_face_corners")
        self.declare_parameter("marker_topic", "/roi_lidar_corner/front_face_markers")
        self.declare_parameter("show_invalid", True)
        self.declare_parameter("point_scale", 0.08)
        self.declare_parameter("text_scale", 0.08)
        self.declare_parameter("frame_id_fallback", "map")
        self.declare_parameter("use_text", True)

        self.corner_topic = self.get_parameter("corner_topic").get_parameter_value().string_value
        self.marker_topic = self.get_parameter("marker_topic").get_parameter_value().string_value
        self.show_invalid = self.get_parameter("show_invalid").get_parameter_value().bool_value
        self.point_scale = float(self.get_parameter("point_scale").get_parameter_value().double_value)
        self.text_scale = float(self.get_parameter("text_scale").get_parameter_value().double_value)
        self.frame_id_fallback = self.get_parameter("frame_id_fallback").get_parameter_value().string_value
        self.use_text = self.get_parameter("use_text").get_parameter_value().bool_value

        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.create_subscription(FrontFaceCorners, self.corner_topic, self.corner_callback, 10)

        self.get_logger().info(f"corner_topic={self.corner_topic}")
        self.get_logger().info(f"marker_topic={self.marker_topic}")

    def corner_callback(self, msg: FrontFaceCorners) -> None:
        markers = MarkerArray()
        clear_all = Marker()
        clear_all.header = msg.header
        clear_all.action = Marker.DELETEALL
        markers.markers.append(clear_all)

        frame_id = msg.header.frame_id if msg.header and msg.header.frame_id else self.frame_id_fallback
        marker_id = 0

        corners = (
            ("top_left", msg.top_left, msg.top_left_status),
            ("top_right", msg.top_right, msg.top_right_status),
            ("bottom_left", msg.bottom_left, msg.bottom_left_status),
            ("bottom_right", msg.bottom_right, msg.bottom_right_status),
        )
        for name, point, status in corners:
            valid = bool(msg.valid) and int(status) != FrontFaceCorners.CORNER_INVALID
            if not self.show_invalid and not valid:
                continue

            point_marker = Marker()
            point_marker.header.frame_id = frame_id
            point_marker.header.stamp = msg.header.stamp
            point_marker.ns = "roi_corner_points"
            point_marker.id = marker_id
            point_marker.type = Marker.SPHERE
            point_marker.action = Marker.ADD
            point_marker.pose.position = Point(x=float(point.x), y=float(point.y), z=float(point.z))
            point_marker.pose.orientation.w = 1.0
            point_marker.scale.x = self.point_scale
            point_marker.scale.y = self.point_scale
            point_marker.scale.z = self.point_scale

            if valid:
                point_marker.color = ColorRGBA(r=0.2, g=1.0, b=0.2, a=0.95)
            else:
                point_marker.color = ColorRGBA(r=1.0, g=0.25, b=0.25, a=0.5)
            markers.markers.append(point_marker)
            marker_id += 1

            if self.use_text:
                text_marker = Marker()
                text_marker.header.frame_id = frame_id
                text_marker.header.stamp = msg.header.stamp
                text_marker.ns = "roi_corner_labels"
                text_marker.id = marker_id
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position = Point(
                    x=float(point.x),
                    y=float(point.y),
                    z=float(point.z) + self.point_scale * 1.8,
                )
                text_marker.pose.orientation.w = 1.0
                text_marker.scale.z = self.text_scale
                text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
                text_marker.text = f"{name} valid={valid} state={int(msg.solution_state)} source={msg.top_source}"
                markers.markers.append(text_marker)
                marker_id += 1

        self.marker_pub.publish(markers)


def main() -> None:
    rclpy.init()
    node = RoiLidarDebugMarkerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
