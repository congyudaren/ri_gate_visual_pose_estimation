#!/usr/bin/env python3

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
from typing import Any, Deque, Dict, List, Optional, Sequence, Tuple

import struct
import numpy as np
import rclpy
from sensor_msgs.msg import PointField
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String, Header

from roi_lidar_corner.msg import Corner3D, Corner3DArray, FrontFaceDebug, FrontFaceROIArray
from rotor_swarm_msgs.msg import FrontFaceCorners
from roi_lidar_corner.front_face_restorer import restore_front_face
from roi_lidar_corner.lookback_solver import (
    DecodedCloudFrame,
    LookbackSolveParams,
    _project_cached_frame,
    prepare_cached_points,
    select_lookback_frames,
    solve_lookback_window,
    trim_cloud_cache,
)
from roi_lidar_corner.structure_point_filter import filter_structure_points
from roi_lidar_corner.structure_tracker import FaceTrack, ObservationStrength, PostObservation, update_post_state


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _quat_to_matrix(msg: Quaternion) -> np.ndarray:
    x = float(msg.x)
    y = float(msg.y)
    z = float(msg.z)
    w = float(msg.w)
    n = w * w + x * x + y * y + z * z
    if n <= 0.0:
        return np.eye(3, dtype=np.float64)
    s = 2.0 / n

    wx = s * w * x
    wy = s * w * y
    wz = s * w * z
    xx = s * x * x
    xy = s * x * y
    xz = s * x * z
    yy = s * y * y
    yz = s * y * z
    zz = s * z * z

    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=np.float64,
    )


def _load_camera_extrinsics(t_values: Sequence[float], r_values: Sequence[float]) -> Tuple[np.ndarray, np.ndarray]:
    if not isinstance(t_values, (list, tuple)) or len(t_values) != 3:
        raise ValueError("camera_extrinsic_t must contain 3 values")
    if not isinstance(r_values, (list, tuple)) or len(r_values) != 9:
        raise ValueError("camera_extrinsic_r must contain 9 values")

    return (
        np.array([float(value) for value in t_values], dtype=np.float64),
        np.array([float(value) for value in r_values], dtype=np.float64).reshape(3, 3),
    )


POINT_DTYPE_MAP = {
    PointField.INT8: "b",
    PointField.UINT8: "B",
    PointField.INT16: "h",
    PointField.UINT16: "H",
    PointField.INT32: "i",
    PointField.UINT32: "I",
    PointField.FLOAT32: "f",
    PointField.FLOAT64: "d",
}


def _lookup_field(fields, name: str):
    for fld in fields:
        if fld.name == name:
            return fld
    return None


def _read_xyz_point_offsets(msg: PointCloud2) -> Tuple[Optional[int], Optional[int], Optional[int], bool]:
    fx = _lookup_field(msg.fields, "x")
    fy = _lookup_field(msg.fields, "y")
    fz = _lookup_field(msg.fields, "z")
    if fx is None or fy is None or fz is None:
        return None, None, None, False

    if fx.datatype not in POINT_DTYPE_MAP or fy.datatype not in POINT_DTYPE_MAP or fz.datatype not in POINT_DTYPE_MAP:
        return None, None, None, False
    return int(fx.offset), int(fy.offset), int(fz.offset), msg.is_bigendian


def _to_np_points(msg: PointCloud2) -> np.ndarray:
    x_off, y_off, z_off, is_big = _read_xyz_point_offsets(msg)
    if x_off is None:
        raise ValueError("pointcloud2 has no xyz fields")

    if msg.point_step <= 0:
        raise ValueError("invalid point_step")

    n_points = len(msg.data) // int(msg.point_step)
    if n_points <= 0:
        return np.zeros((0, 3), dtype=np.float64)

    endian = ">" if is_big else "<"
    fmt = [
        endian + POINT_DTYPE_MAP[_lookup_field(msg.fields, "x").datatype],
        endian + POINT_DTYPE_MAP[_lookup_field(msg.fields, "y").datatype],
        endian + POINT_DTYPE_MAP[_lookup_field(msg.fields, "z").datatype],
    ]

    raw = memoryview(msg.data)
    out = np.zeros((n_points, 3), dtype=np.float64)
    for idx in range(n_points):
        base = idx * int(msg.point_step)
        out[idx, 0] = struct.unpack_from(fmt[0], raw, base + x_off)[0]
        out[idx, 1] = struct.unpack_from(fmt[1], raw, base + y_off)[0]
        out[idx, 2] = struct.unpack_from(fmt[2], raw, base + z_off)[0]

    valid = np.isfinite(out).all(axis=1)
    out = out[valid]
    if out.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    return out


@dataclass
class _BufferedPose:
    stamp: float
    t: np.ndarray
    R: np.ndarray


class CornerLidarSolverNode(Node):
    def __init__(self):
        super().__init__("corner_lidar_solver_node")

        self.declare_parameter("roi_topic", "/roi_lidar_corner/front_face_rois")
        self.declare_parameter("pointcloud_topic", "/cloud_registered")
        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("point_output_topic", "/roi_lidar_corner/front_face_corners")
        self.declare_parameter("debug_output_topic", "/roi_lidar_corner/front_face_debug")
        self.declare_parameter("diag_output_topic", "/roi_lidar_corner/solver_diag")
        self.declare_parameter("debug_uv_output_topic", "/roi_lidar_corner/solver_debug_uv")
        self.declare_parameter("debug_projected_cloud_stride", 8)
        self.declare_parameter("publish_debug_uv", True)
        self.declare_parameter("debug_overlay_frame_count", 1)
        self.declare_parameter("max_cloud_buffer_size", 20)
        self.declare_parameter("max_odom_buffer_size", 40)
        self.declare_parameter("max_time_diff_cloud", 1.0)
        self.declare_parameter("max_time_diff_odom", 0.5)
        self.declare_parameter("min_points", 2)
        self.declare_parameter("min_range", 0.2)
        self.declare_parameter("max_range", 30.0)
        self.declare_parameter("cloud_frame_mode", "auto")
        self.declare_parameter("history_window_sec", 1.0)
        self.declare_parameter("max_window_frames", 30)
        self.declare_parameter("cache_voxel_size", 0.1)
        self.declare_parameter("bbox_expand_ratio", 0.15)
        self.declare_parameter("corner_target_points", 6)
        self.declare_parameter("corner_target_frames", 2)
        self.declare_parameter("corner_cap_points", 96)
        self.declare_parameter("post_max_z_jump_m", 0.8)
        self.declare_parameter("frame_width_m", 1.0)
        self.declare_parameter("frame_height_m", 2.0)
        self.declare_parameter("front_bin_size", 0.05)
        self.declare_parameter("front_keep_tolerance", 0.08)
        self.declare_parameter("rear_gap_m", 1.0)
        self.declare_parameter("rear_gap_tolerance", 0.12)
        self.declare_parameter("output_frame_id", "camera")
        self.declare_parameter("output_extrinsic_t_x", 0.0)
        self.declare_parameter("output_extrinsic_t_y", 0.0)
        self.declare_parameter("output_extrinsic_t_z", 0.0)
        self.declare_parameter("output_extrinsic_r_00", 1.0)
        self.declare_parameter("output_extrinsic_r_01", 0.0)
        self.declare_parameter("output_extrinsic_r_02", 0.0)
        self.declare_parameter("output_extrinsic_r_10", 0.0)
        self.declare_parameter("output_extrinsic_r_11", 1.0)
        self.declare_parameter("output_extrinsic_r_12", 0.0)
        self.declare_parameter("output_extrinsic_r_20", 0.0)
        self.declare_parameter("output_extrinsic_r_21", 0.0)
        self.declare_parameter("output_extrinsic_r_22", 1.0)
        self.declare_parameter("camera_extrinsic_t", [0.0, 0.0, 0.0])
        self.declare_parameter(
            "camera_extrinsic_r",
            [
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
            ],
        )

        self.roi_topic = self.get_parameter("roi_topic").get_parameter_value().string_value
        self.pointcloud_topic = self.get_parameter("pointcloud_topic").get_parameter_value().string_value
        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.point_output_topic = self.get_parameter("point_output_topic").get_parameter_value().string_value
        self.debug_output_topic = self.get_parameter("debug_output_topic").get_parameter_value().string_value
        self.diag_output_topic = self.get_parameter("diag_output_topic").get_parameter_value().string_value
        self.debug_uv_output_topic = self.get_parameter("debug_uv_output_topic").get_parameter_value().string_value
        self.debug_projected_cloud_stride = max(
            1, int(self.get_parameter("debug_projected_cloud_stride").get_parameter_value().integer_value)
        )
        self.publish_debug_uv = self.get_parameter("publish_debug_uv").get_parameter_value().bool_value
        self.debug_overlay_frame_count = max(
            0,
            int(self.get_parameter("debug_overlay_frame_count").get_parameter_value().integer_value),
        )
        self.max_cloud_buffer_size = int(self.get_parameter("max_cloud_buffer_size").get_parameter_value().integer_value)
        self.max_odom_buffer_size = int(self.get_parameter("max_odom_buffer_size").get_parameter_value().integer_value)
        self.max_time_diff_cloud = float(self.get_parameter("max_time_diff_cloud").get_parameter_value().double_value)
        self.max_time_diff_odom = float(self.get_parameter("max_time_diff_odom").get_parameter_value().double_value)
        self.min_points = int(self.get_parameter("min_points").get_parameter_value().integer_value)
        self.min_range = float(self.get_parameter("min_range").get_parameter_value().double_value)
        self.max_range = float(self.get_parameter("max_range").get_parameter_value().double_value)
        self.debug_log_every_n_msgs = 20
        self.cloud_frame_mode = self.get_parameter("cloud_frame_mode").get_parameter_value().string_value.strip().lower()
        self.history_window_sec = float(self.get_parameter("history_window_sec").get_parameter_value().double_value)
        self.max_window_frames = max(0, int(self.get_parameter("max_window_frames").get_parameter_value().integer_value))
        self.cache_voxel_size = float(self.get_parameter("cache_voxel_size").get_parameter_value().double_value)
        self.lookback_params = LookbackSolveParams(
            bbox_expand_ratio=float(self.get_parameter("bbox_expand_ratio").get_parameter_value().double_value),
            corner_target_points=max(
                1, int(self.get_parameter("corner_target_points").get_parameter_value().integer_value)
            ),
            corner_target_frames=max(
                1, int(self.get_parameter("corner_target_frames").get_parameter_value().integer_value)
            ),
            corner_cap_points=max(0, int(self.get_parameter("corner_cap_points").get_parameter_value().integer_value)),
            min_range=self.min_range,
            max_range=self.max_range,
        )
        self.frame_width_m = float(self.get_parameter("frame_width_m").get_parameter_value().double_value)
        self.frame_height_m = float(self.get_parameter("frame_height_m").get_parameter_value().double_value)
        self.post_max_z_jump_m = float(self.get_parameter("post_max_z_jump_m").get_parameter_value().double_value)
        self.front_bin_size = float(self.get_parameter("front_bin_size").get_parameter_value().double_value)
        self.front_keep_tolerance = float(
            self.get_parameter("front_keep_tolerance").get_parameter_value().double_value
        )
        self.rear_gap_m = float(self.get_parameter("rear_gap_m").get_parameter_value().double_value)
        self.rear_gap_tolerance = float(self.get_parameter("rear_gap_tolerance").get_parameter_value().double_value)
        self.output_frame_id = self.get_parameter("output_frame_id").get_parameter_value().string_value.strip()
        self.output_extrinsic_t = np.array(
            [
                float(self.get_parameter("output_extrinsic_t_x").get_parameter_value().double_value),
                float(self.get_parameter("output_extrinsic_t_y").get_parameter_value().double_value),
                float(self.get_parameter("output_extrinsic_t_z").get_parameter_value().double_value),
            ],
            dtype=np.float64,
        )
        self.output_extrinsic_R = np.array(
            [
                [
                    float(self.get_parameter("output_extrinsic_r_00").get_parameter_value().double_value),
                    float(self.get_parameter("output_extrinsic_r_01").get_parameter_value().double_value),
                    float(self.get_parameter("output_extrinsic_r_02").get_parameter_value().double_value),
                ],
                [
                    float(self.get_parameter("output_extrinsic_r_10").get_parameter_value().double_value),
                    float(self.get_parameter("output_extrinsic_r_11").get_parameter_value().double_value),
                    float(self.get_parameter("output_extrinsic_r_12").get_parameter_value().double_value),
                ],
                [
                    float(self.get_parameter("output_extrinsic_r_20").get_parameter_value().double_value),
                    float(self.get_parameter("output_extrinsic_r_21").get_parameter_value().double_value),
                    float(self.get_parameter("output_extrinsic_r_22").get_parameter_value().double_value),
                ],
            ],
            dtype=np.float64,
        )
        # Use p_body = R_cb @ p_camera + t_cb when republishing into the body frame.
        self.roi_msg_counter = 0
        self.track = FaceTrack()

        try:
            self.t_cb, self.R_cb = _load_camera_extrinsics(
                self.get_parameter("camera_extrinsic_t").get_parameter_value().double_array_value,
                self.get_parameter("camera_extrinsic_r").get_parameter_value().double_array_value,
            )
            self.get_logger().info("Loaded solver camera extrinsics from ROI parameters")
        except Exception as exc:
            raise RuntimeError(f"Failed to load solver camera extrinsics from ROI parameters: {exc}") from exc

        self.fx = 0.0
        self.fy = 0.0
        self.cx = 0.0
        self.cy = 0.0

        self.camera_width = 0
        self.camera_height = 0
        self.camera_info_ready = False
        self.image_width = 0
        self.image_height = 0

        self.cloud_buffer: Deque[DecodedCloudFrame] = deque(
            maxlen=max(self.max_cloud_buffer_size, self.max_window_frames or 0, 1)
        )
        self.odom_buffer: Deque[_BufferedPose] = deque(maxlen=self.max_odom_buffer_size)

        self.point_pub = self.create_publisher(FrontFaceCorners, self.point_output_topic, 10)
        self.debug_pub = self.create_publisher(FrontFaceDebug, self.debug_output_topic, 10)
        self.diag_pub = self.create_publisher(String, self.diag_output_topic, 10)
        self.debug_uv_pub = self.create_publisher(String, self.debug_uv_output_topic, 10)

        self.create_subscription(FrontFaceROIArray, self.roi_topic, self.roi_callback, 10)
        self.create_subscription(PointCloud2, self.pointcloud_topic, self.cloud_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)
        self.create_subscription(Image, self.image_topic, self.image_callback, 10)

        self.get_logger().info(f"ROI topic: {self.roi_topic}")
        self.get_logger().info(f"Pointcloud topic: {self.pointcloud_topic}")
        self.get_logger().info(f"Odom topic: {self.odom_topic}")
        self.get_logger().info(f"Output topic: {self.point_output_topic}")

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_width = msg.width
        self.camera_height = msg.height
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.camera_info_ready = True

    def cloud_callback(self, msg: PointCloud2) -> None:
        stamp = _stamp_to_sec(msg.header.stamp)
        points = _to_np_points(msg)
        frame_mode = self._resolve_cloud_frame_mode(str(msg.header.frame_id))
        cached_points = prepare_cached_points(
            points_world_f32=points.astype(np.float32, copy=False),
            min_range=self.min_range,
            max_range=self.max_range,
            voxel_size=self.cache_voxel_size,
            apply_range_filter=frame_mode != "world",
        )
        self.cloud_buffer.append(
            DecodedCloudFrame(
                stamp=stamp,
                points_world_f32=np.asarray(cached_points, dtype=np.float32),
                frame_id=str(msg.header.frame_id),
            )
        )
        trim_cloud_cache(self.cloud_buffer, newest_stamp=stamp, history_window_sec=self.history_window_sec)

    def image_callback(self, msg: Image) -> None:
        self.image_width = int(msg.width)
        self.image_height = int(msg.height)

    def odom_callback(self, msg: Odometry) -> None:
        stamp = _stamp_to_sec(msg.header.stamp)
        t = np.array(
            [
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                float(msg.pose.pose.position.z),
            ],
            dtype=np.float64,
        )
        R = _quat_to_matrix(msg.pose.pose.orientation)
        self.odom_buffer.append(_BufferedPose(stamp=stamp, t=t, R=R))

    def roi_callback(self, msg: FrontFaceROIArray) -> None:
        self.roi_msg_counter += 1
        stamp = _stamp_to_sec(msg.header.stamp)
        pose = self._find_nearest_odom(stamp)
        window_frames = select_lookback_frames(
            self.cloud_buffer,
            stamp=stamp,
            history_window_sec=self.history_window_sec,
            max_window_frames=self.max_window_frames,
        )

        diag = {
            "stamp": stamp,
            "roi_objects": len(msg.objects),
            "cloud_ok": bool(window_frames),
            "pose_ok": pose is not None,
            "intrinsics": self.camera_info_ready,
        }
        if pose is None:
            diag["reason"] = "no_pose_near_stamp"
            self._publish_empty_outputs(msg.header)
            if self.roi_msg_counter % self.debug_log_every_n_msgs == 0:
                self.get_logger().info(f"solve_msg={self.roi_msg_counter} early_return reason={diag['reason']}")
            self.diag_pub.publish(String(data=str(diag)))
            return
        if self.fx <= 0.0 or self.fy <= 0.0:
            diag["reason"] = "invalid_intrinsics"
            self._publish_empty_outputs(msg.header)
            if self.roi_msg_counter % self.debug_log_every_n_msgs == 0:
                self.get_logger().info(f"solve_msg={self.roi_msg_counter} early_return reason={diag['reason']}")
            self.diag_pub.publish(String(data=str(diag)))
            return

        if not msg.objects:
            self._publish_empty_outputs(msg.header)
            empty_diag = {"stamp": stamp, "reason": "no_roi_objects"}
            if self.roi_msg_counter % self.debug_log_every_n_msgs == 0:
                self.get_logger().info(f"solve_msg={self.roi_msg_counter} early_return reason={empty_diag['reason']}")
            self.diag_pub.publish(String(data=str(empty_diag)))
            return

        if not window_frames:
            diag["reason"] = "no_cloud_in_window"
            self._publish_empty_outputs(
                msg.header,
                stats={"frames_seen": 0, "frames_used": 0, "stop_reason": "no_cloud_in_window"},
            )
            if self.roi_msg_counter % self.debug_log_every_n_msgs == 0:
                self.get_logger().info(f"solve_msg={self.roi_msg_counter} early_return reason={diag['reason']}")
            self.diag_pub.publish(String(data=str(diag)))
            return
        if abs(float(window_frames[0].stamp) - stamp) > self.max_time_diff_cloud:
            diag["reason"] = "no_cloud_near_stamp"
            self._publish_empty_outputs(
                msg.header,
                stats={
                    "frames_seen": len(window_frames),
                    "frames_used": 0,
                    "stop_reason": "no_cloud_near_stamp",
                },
            )
            if self.roi_msg_counter % self.debug_log_every_n_msgs == 0:
                self.get_logger().info(f"solve_msg={self.roi_msg_counter} early_return reason={diag['reason']}")
            self.diag_pub.publish(String(data=str(diag)))
            return

        try:
            stats = self._update_track_from_roi(msg, window_frames, pose)
            self._publish_solution(msg.header)
            if self.publish_debug_uv:
                self._publish_empty_debug_uv(msg.header, stats=stats)
            diag["valid"] = bool(self.point_pub.messages[-1].valid) if hasattr(self.point_pub, "messages") and self.point_pub.messages else None
            diag["frames_seen"] = int(stats.get("frames_seen", len(window_frames)))
            diag["frames_used"] = int(stats.get("frames_used", 0))
            diag["points_raw_sum"] = int(stats.get("points_raw_sum", 0))
            diag["points_kept_sum"] = int(stats.get("points_kept_sum", 0))
            diag["points_rear_rejected_sum"] = int(stats.get("points_rear_rejected_sum", 0))
            diag["reason"] = str(stats.get("stop_reason", "ok"))
            if self.roi_msg_counter % self.debug_log_every_n_msgs == 0:
                self.get_logger().info(
                    f"solve_msg={self.roi_msg_counter} roi_objects={len(msg.objects)} "
                    f"frames_seen={stats.get('frames_seen', 0)} frames_used={stats.get('frames_used', 0)} "
                    f"points_raw_sum={stats.get('points_raw_sum', 0)} "
                    f"points_kept_sum={stats.get('points_kept_sum', 0)}"
                )
            self.diag_pub.publish(String(data=str(diag)))
        except Exception as exc:
            diag["reason"] = f"solve_failed:{type(exc).__name__}:{exc}"
            self._publish_empty_outputs(msg.header)
            self.get_logger().info(f"solve_msg={self.roi_msg_counter} early_return reason={diag['reason']}")
            self.diag_pub.publish(String(data=str(diag)))

    def _make_empty_corner_array(self, header: Header) -> FrontFaceCorners:
        msg = FrontFaceCorners()
        msg.header = header
        msg.valid = False
        msg.solution_state = FrontFaceCorners.SOLUTION_INVALID
        msg.top_source = "none"
        msg.top_left_status = FrontFaceCorners.CORNER_INVALID
        msg.top_right_status = FrontFaceCorners.CORNER_INVALID
        msg.bottom_left_status = FrontFaceCorners.CORNER_INVALID
        msg.bottom_right_status = FrontFaceCorners.CORNER_INVALID
        return msg

    def _publish_empty_outputs(self, header: Header, stats: Optional[Dict] = None) -> None:
        self.point_pub.publish(self._make_empty_corner_array(header))
        debug = FrontFaceDebug()
        debug.header = header
        debug.valid = False
        debug.top_source = "none"
        self.debug_pub.publish(debug)
        if self.publish_debug_uv:
            self._publish_empty_debug_uv(header, stats=stats)

    def _publish_empty_debug_uv(self, header: Header, stats: Optional[Dict] = None) -> None:
        self.debug_uv_pub.publish(
            String(
                data=json.dumps(
                    {
                        "stamp": _stamp_to_sec(header.stamp),
                        "objects": [],
                        "cloud_uv": [],
                        "cloud_uv_depth": [],
                        "stats": stats or {},
                    },
                    ensure_ascii=False,
                )
            )
        )

    def _resolve_cloud_frame_mode(self, frame_id: str) -> str:
        if self.cloud_frame_mode in {"world", "map"}:
            return "world"
        if self.cloud_frame_mode in {"base", "body", "sensor"}:
            return "base"

        text = (frame_id or "").strip().lower()
        world_tokens = ("map", "odom", "world", "camera_init")
        if any(token in text for token in world_tokens):
            return "world"
        return "base"

    def _scaled_intrinsics(self) -> Tuple[float, float, float, float, int, int]:
        image_w = self.image_width if self.image_width > 0 else self.camera_width
        image_h = self.image_height if self.image_height > 0 else self.camera_height
        if image_w <= 0:
            image_w = self.camera_width
        if image_h <= 0:
            image_h = self.camera_height

        fx = self.fx
        fy = self.fy
        cx = self.cx
        cy = self.cy
        if self.camera_width > 0 and image_w > 0 and self.camera_width != image_w:
            scale_x = float(image_w) / float(self.camera_width)
            fx *= scale_x
            cx *= scale_x
        if self.camera_height > 0 and image_h > 0 and self.camera_height != image_h:
            scale_y = float(image_h) / float(self.camera_height)
            fy *= scale_y
            cy *= scale_y
        return fx, fy, cx, cy, image_w, image_h

    def _find_nearest_odom(self, stamp: float) -> Optional[_BufferedPose]:
        if not self.odom_buffer:
            return None
        best = min(self.odom_buffer, key=lambda item: abs(item.stamp - stamp))
        if abs(best.stamp - stamp) > self.max_time_diff_odom:
            return None
        return best

    def _normalize_window_frames(self, frames: Sequence[DecodedCloudFrame]) -> List[DecodedCloudFrame]:
        normalized_frames: List[DecodedCloudFrame] = []
        for frame in frames:
            frame_mode = self._resolve_cloud_frame_mode(frame.frame_id)
            normalized_frames.append(
                DecodedCloudFrame(
                    stamp=float(frame.stamp),
                    points_world_f32=np.asarray(frame.points_world_f32, dtype=np.float32),
                    frame_id=frame_mode,
                )
            )
        return normalized_frames

    def _publish_solution(self, header: Header) -> None:
        solution = restore_front_face(self.track, width_m=self.frame_width_m, height_m=self.frame_height_m)
        frame_header = Header()
        frame_header.stamp = header.stamp
        frame_header.frame_id = self.output_frame_id or header.frame_id

        def transform_point(point: Tuple[float, float, float]) -> Tuple[float, float, float]:
            xyz = np.asarray(point, dtype=np.float64).reshape(3)
            transformed = self.output_extrinsic_R @ xyz + self.output_extrinsic_t
            return float(transformed[0]), float(transformed[1]), float(transformed[2])

        top_left = transform_point(solution.top_left)
        top_right = transform_point(solution.top_right)
        bottom_left = transform_point(solution.bottom_left)
        bottom_right = transform_point(solution.bottom_right)

        msg = FrontFaceCorners()
        msg.header = frame_header
        msg.valid = bool(solution.valid)
        if solution.solution_state == "tracking":
            msg.solution_state = FrontFaceCorners.SOLUTION_TRACKING
        elif solution.solution_state == "lost":
            msg.solution_state = FrontFaceCorners.SOLUTION_LOST
        else:
            msg.solution_state = FrontFaceCorners.SOLUTION_INVALID
        msg.tracking_confidence = float(getattr(solution, "tracking_confidence", 0.0))
        msg.top_source = str(solution.top_source)
        msg.top_left.x, msg.top_left.y, msg.top_left.z = top_left
        msg.top_right.x, msg.top_right.y, msg.top_right.z = top_right
        msg.bottom_left.x, msg.bottom_left.y, msg.bottom_left.z = bottom_left
        msg.bottom_right.x, msg.bottom_right.y, msg.bottom_right.z = bottom_right
        status = FrontFaceCorners.CORNER_OBSERVED if solution.valid else FrontFaceCorners.CORNER_INVALID
        msg.top_left_status = status
        msg.top_right_status = status
        msg.bottom_left_status = status
        msg.bottom_right_status = status
        self.point_pub.publish(msg)

        debug = FrontFaceDebug()
        debug.header = header
        debug.valid = bool(solution.valid)
        debug.direct_top_left.x, debug.direct_top_left.y, debug.direct_top_left.z = solution.top_left
        debug.direct_top_right.x, debug.direct_top_right.y, debug.direct_top_right.z = solution.top_right
        debug.direct_top_left_valid = bool(solution.valid)
        debug.direct_top_right_valid = bool(solution.valid)
        debug.left_post_valid = bool(self.track.left_post.initialized)
        debug.left_post_x = float(self.track.left_post.x_state) if self.track.left_post.initialized else 0.0
        debug.left_post_z = float(self.track.left_post.z_state) if self.track.left_post.initialized else 0.0
        debug.left_post_confidence = float(self.track.left_post.confidence)
        debug.right_post_valid = bool(self.track.right_post.initialized)
        debug.right_post_x = float(self.track.right_post.x_state) if self.track.right_post.initialized else 0.0
        debug.right_post_z = float(self.track.right_post.z_state) if self.track.right_post.initialized else 0.0
        debug.right_post_confidence = float(self.track.right_post.confidence)
        debug.top_beam_valid = bool(self.track.top_beam.initialized)
        debug.top_beam_y = float(self.track.top_beam.y_top_state) if self.track.top_beam.initialized else 0.0
        debug.top_beam_confidence = float(self.track.top_beam.confidence)
        debug.tracking_confidence = float(getattr(solution, "tracking_confidence", 0.0))
        debug.top_source = str(solution.top_source)
        self.debug_pub.publish(debug)

    def _update_track_from_roi(
        self,
        roi_msg: FrontFaceROIArray,
        frames: Sequence[DecodedCloudFrame],
        pose: _BufferedPose,
    ) -> Dict:
        proj_fx, proj_fy, proj_cx, proj_cy, image_w, image_h = self._scaled_intrinsics()
        normalized_frames = self._normalize_window_frames(frames)
        all_xyz = []
        all_uv = []
        for frame in normalized_frames:
            frame_points, frame_uv, _, _ = _project_cached_frame(
                frame=frame,
                pose=pose,
                fx=proj_fx,
                fy=proj_fy,
                cx=proj_cx,
                cy=proj_cy,
                image_width=image_w,
                image_height=image_h,
                t_cb=self.t_cb.astype(np.float32),
                R_cb=self.R_cb.astype(np.float32),
                cloud_frame_mode=self.cloud_frame_mode,
                min_range=self.lookback_params.min_range,
                max_range=self.lookback_params.max_range,
            )
            if frame_uv.size:
                all_xyz.append(np.asarray(frame_points, dtype=np.float32).reshape(-1, 3))
                all_uv.append(np.asarray(frame_uv, dtype=np.float32).reshape(-1, 2))
        if not all_xyz:
            self.track.left_post = update_post_state(
                self.track.left_post,
                PostObservation.missing(_stamp_to_sec(roi_msg.header.stamp)),
                self.history_window_sec,
                max_z_jump_m=self.post_max_z_jump_m,
            )
            self.track.right_post = update_post_state(
                self.track.right_post,
                PostObservation.missing(_stamp_to_sec(roi_msg.header.stamp)),
                self.history_window_sec,
                max_z_jump_m=self.post_max_z_jump_m,
            )
            return {"frames_seen": len(frames), "frames_used": 0, "stop_reason": "no_projected_points"}

        xyz = np.vstack(all_xyz)
        uv = np.vstack(all_uv)
        stamp = _stamp_to_sec(roi_msg.header.stamp)
        stats = {
            "frames_seen": len(frames),
            "frames_used": len(normalized_frames),
            "points_raw_sum": 0,
            "points_kept_sum": 0,
            "points_rear_rejected_sum": 0,
            "stop_reason": "ok",
        }
        if not roi_msg.objects:
            return {**stats, "stop_reason": "no_roi_objects"}
        obj = roi_msg.objects[0]
        seen_posts = set()
        for structure in getattr(obj, "structures", []):
            selected_uv, selected_xyz = self._select_structure_points(structure, uv, xyz)
            filtered = filter_structure_points(
                uv=selected_uv,
                xyz_cam=selected_xyz,
                front_bin_size=self.front_bin_size,
                front_keep_tolerance=self.front_keep_tolerance,
                rear_gap_m=self.rear_gap_m,
                rear_gap_tolerance=self.rear_gap_tolerance,
            )
            stats["points_raw_sum"] += int(filtered.points_raw)
            stats["points_kept_sum"] += int(filtered.points_kept)
            stats["points_rear_rejected_sum"] += int(filtered.points_rear_rejected)
            if int(structure.structure_label) == int(structure.TOP_BEAM):
                if filtered.points_kept >= self.min_points:
                    self.track.top_beam.initialized = True
                    self.track.top_beam.y_top_state = float(np.median(filtered.xyz_cam[:, 1]))
                    self.track.top_beam.confidence = min(1.0, float(filtered.points_kept) / max(1, self.min_points))
                continue
            if int(structure.structure_label) not in {int(structure.LEFT_POST), int(structure.RIGHT_POST)}:
                continue
            seen_posts.add(int(structure.structure_label))
            if filtered.points_kept >= self.min_points:
                obs = PostObservation(
                    strength=ObservationStrength.OBSERVED,
                    stamp=stamp,
                    x_obs=float(np.median(filtered.xyz_cam[:, 0])),
                    z_obs=float(np.median(filtered.xyz_cam[:, 2])),
                    y_visible_min=float(np.min(filtered.xyz_cam[:, 1])),
                    y_visible_max=float(np.max(filtered.xyz_cam[:, 1])),
                    front_peak_confidence=min(1.0, float(filtered.points_kept) / max(1, self.min_points)),
                    top_side_sample_present=self.track.top_beam.initialized,
                    support_count=int(filtered.points_kept),
                )
            else:
                obs = PostObservation.missing(stamp=stamp)
            if int(structure.structure_label) == int(structure.LEFT_POST):
                self.track.left_post = update_post_state(
                    self.track.left_post,
                    obs,
                    self.history_window_sec,
                    max_z_jump_m=self.post_max_z_jump_m,
                )
            else:
                self.track.right_post = update_post_state(
                    self.track.right_post,
                    obs,
                    self.history_window_sec,
                    max_z_jump_m=self.post_max_z_jump_m,
                )
        if self.track.left_post.initialized and self.track.right_post.initialized and self.track.top_beam.initialized:
            self.track.model_initialized = True
        return stats

    def _select_structure_points(
        self,
        structure,
        uv: np.ndarray,
        xyz_cam: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        if uv.size == 0:
            return uv.reshape(0, 2), xyz_cam.reshape(0, 3)
        mask = np.asarray(structure.roi_mask, dtype=np.uint8).reshape(
            int(structure.mask_height),
            int(structure.mask_width),
        )
        u = np.round(uv[:, 0]).astype(np.int32) - int(structure.mask_origin_x)
        v = np.round(uv[:, 1]).astype(np.int32) - int(structure.mask_origin_y)
        inside = (u >= 0) & (v >= 0) & (u < int(structure.mask_width)) & (v < int(structure.mask_height))
        selected = np.zeros(uv.shape[0], dtype=bool)
        if np.any(inside):
            idx = np.where(inside)[0]
            selected[idx] = mask[v[idx], u[idx]] > 0
        return uv[selected], xyz_cam[selected]

    def _solve(
        self,
        roi_msg: FrontFaceROIArray,
        frames: Sequence[DecodedCloudFrame],
        pose: _BufferedPose,
        *,
        build_debug_payload: bool,
    ) -> Tuple[Corner3DArray, Dict, Optional[Dict]]:
        stamp = _stamp_to_sec(roi_msg.header.stamp)
        proj_fx, proj_fy, proj_cx, proj_cy, image_w, image_h = self._scaled_intrinsics()
        normalized_frames = self._normalize_window_frames(frames)
        helper_result = solve_lookback_window(
            roi_msg=roi_msg,
            frames=normalized_frames,
            pose=pose,
            fx=proj_fx,
            fy=proj_fy,
            cx=proj_cx,
            cy=proj_cy,
            image_width=image_w,
            image_height=image_h,
            t_cb=self.t_cb.astype(np.float32),
            R_cb=self.R_cb.astype(np.float32),
            cloud_frame_mode=self.cloud_frame_mode,
            params=self.lookback_params,
        )
        corners_by_key = helper_result.get("corners", {})
        corner_states = helper_result.get("corner_states", {})
        corner_support = helper_result.get("corner_support", {})
        stats = dict(helper_result.get("stats", {}))
        stats.update(
            {
                "stamp": stamp,
                "camera_w": int(self.camera_width),
                "camera_h": int(self.camera_height),
                "image_w": int(image_w),
                "image_h": int(image_h),
                "proj_fx": float(proj_fx),
                "proj_fy": float(proj_fy),
                "proj_cx": float(proj_cx),
                "proj_cy": float(proj_cy),
            }
        )

        out = Corner3DArray()
        out.header = roi_msg.header
        out.corners = []
        debug_objects = [] if build_debug_payload else None

        for obj in roi_msg.objects:
            if build_debug_payload:
                debug_object = {"object_id": int(obj.object_id), "corners": []}
            for corner_label in range(4):
                key = (int(obj.object_id), int(corner_label))
                center = corners_by_key.get(key)
                state = corner_states.get(key)
                support_state = corner_support.get(key, state)
                if state is None:
                    out.corners.append(
                        self._make_invalid_corner(roi_msg.header, obj.object_id, obj.class_id, corner_label)
                    )
                    if build_debug_payload:
                        debug_object["corners"].append(
                            self._make_debug_corner_entry(
                                corner_label=corner_label,
                                uv_points=[],
                                valid=False,
                                failure_reason="no_state",
                            )
                        )
                    continue

                support_points = np.asarray(support_state.points_cam_current, dtype=np.float32).reshape(-1, 3)
                uv_points = []
                if build_debug_payload:
                    uv_hits = np.asarray(support_state.uv_hits, dtype=np.float32).reshape(-1, 2)
                    uv_points = [[int(round(u)), int(round(v))] for u, v in uv_hits[:200].tolist()]
                if center is None:
                    out.corners.append(
                        self._make_invalid_corner(roi_msg.header, obj.object_id, obj.class_id, corner_label)
                    )
                    if build_debug_payload:
                        debug_object["corners"].append(
                            self._make_debug_corner_entry(
                                corner_label=corner_label,
                                uv_points=uv_points,
                                valid=False,
                                failure_reason=self._infer_missing_center_reason(
                                    support_points=support_points,
                                    support_state=support_state,
                                ),
                            ),
                        )
                    continue

                center = np.asarray(center, dtype=np.float32).reshape(-1)
                if center.size < 3:
                    out.corners.append(
                        self._make_invalid_corner(roi_msg.header, obj.object_id, obj.class_id, corner_label)
                    )
                    if build_debug_payload:
                        debug_object["corners"].append(
                            self._make_debug_corner_entry(
                                corner_label=corner_label,
                                uv_points=uv_points,
                                valid=False,
                                failure_reason="invalid_center",
                            )
                        )
                    continue

                if support_points.shape[0] < self.min_points:
                    out.corners.append(
                        self._make_invalid_corner(roi_msg.header, obj.object_id, obj.class_id, corner_label)
                    )
                    if build_debug_payload:
                        debug_object["corners"].append(
                            self._make_debug_corner_entry(
                                corner_label=corner_label,
                                uv_points=uv_points,
                                valid=False,
                                failure_reason="below_min_points",
                            )
                        )
                    continue

                corner = Corner3D()
                corner.header = roi_msg.header
                corner.object_id = int(obj.object_id)
                corner.class_id = int(obj.class_id)
                corner.corner_label = int(corner_label)
                corner.support_point_count = int(support_points.shape[0])
                corner.x = float(center[0])
                corner.y = float(center[1])
                corner.z = float(center[2])
                if support_points.size == 0:
                    corner.fit_error = 0.0
                else:
                    corner.fit_error = float(np.sqrt(np.mean((support_points - center[:3]) ** 2)))
                corner.valid = True
                out.corners.append(corner)
                if build_debug_payload:
                    debug_object["corners"].append(
                        self._make_debug_corner_entry(
                            corner_label=corner_label,
                            uv_points=uv_points,
                            valid=True,
                            failure_reason="",
                        )
                    )

            if build_debug_payload and debug_objects is not None:
                debug_objects.append(debug_object)

        if not build_debug_payload:
            return out, stats, None

        cloud_uv = []
        cloud_uv_depth = []
        overlay_frames = list(normalized_frames[: self.debug_overlay_frame_count])
        projected_cloud_uv = []
        projected_cloud_depth = []
        for frame in overlay_frames:
            frame_points, frame_uv, _, _ = _project_cached_frame(
                frame=frame,
                pose=pose,
                fx=proj_fx,
                fy=proj_fy,
                cx=proj_cx,
                cy=proj_cy,
                image_width=image_w,
                image_height=image_h,
                t_cb=self.t_cb.astype(np.float32),
                R_cb=self.R_cb.astype(np.float32),
                cloud_frame_mode=self.cloud_frame_mode,
                min_range=self.lookback_params.min_range,
                max_range=self.lookback_params.max_range,
            )
            if frame_uv.size != 0:
                projected_cloud_uv.append(frame_uv)
                projected_cloud_depth.append(np.asarray(frame_points[:, 2], dtype=np.float32).reshape(-1))
        if projected_cloud_uv:
            stacked_uv = np.vstack(projected_cloud_uv)
            stacked_depth = np.concatenate(projected_cloud_depth)
            for (u, v), depth in zip(
                stacked_uv[:: self.debug_projected_cloud_stride],
                stacked_depth[:: self.debug_projected_cloud_stride],
            ):
                u = float(np.floor(u))
                v = float(np.floor(v))
                cloud_uv.append([int(round(u)), int(round(v))])
                cloud_uv_depth.append(float(depth))

        return out, stats, {
            "stamp": stamp,
            "objects": debug_objects or [],
            "cloud_uv": cloud_uv,
            "cloud_uv_depth": cloud_uv_depth,
            "stats": stats,
        }

    def _infer_missing_center_reason(self, support_points: np.ndarray, support_state: Any) -> str:
        support_count = int(np.asarray(support_points, dtype=np.float32).reshape(-1, 3).shape[0])
        support_frames = int(getattr(support_state, "support_frames", 0))
        needs_more_points = support_count < int(self.lookback_params.corner_target_points)
        needs_more_frames = support_frames < int(self.lookback_params.corner_target_frames)
        if needs_more_points and needs_more_frames:
            return "insufficient_support_points_and_frames"
        if needs_more_points:
            return "insufficient_support_points"
        if needs_more_frames:
            return "insufficient_support_frames"
        return "center_unavailable"

    def _make_debug_corner_entry(
        self,
        *,
        corner_label: int,
        uv_points: Sequence[Sequence[int]],
        valid: bool,
        failure_reason: str,
    ) -> Dict:
        return {
            "corner_label": int(corner_label),
            "uv": [[int(point[0]), int(point[1])] for point in uv_points],
            "valid": bool(valid),
            "failure_reason": str(failure_reason),
        }

    def _make_invalid_corner(self, header: Header, object_id: int, class_id: int, corner_label: int) -> Corner3D:
        corner = Corner3D()
        corner.header = header
        corner.object_id = int(object_id)
        corner.class_id = int(class_id)
        corner.corner_label = int(corner_label)
        corner.support_point_count = 0
        corner.x = 0.0
        corner.y = 0.0
        corner.z = 0.0
        corner.fit_error = 0.0
        corner.valid = False
        return corner


def main() -> None:
    rclpy.init()
    node = CornerLidarSolverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
