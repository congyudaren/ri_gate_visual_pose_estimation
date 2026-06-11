#!/usr/bin/env python3

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import json
from pathlib import Path
import time
from typing import Any, Deque, Sequence

import cv2
import numpy as np

try:
    from roi_lidar_corner.corner_lidar_solver_node import (
        _load_fastlio_camera_offsets,
        _quat_to_matrix,
        _stamp_to_sec,
        _to_np_points,
    )
except ImportError:  # pragma: no cover - exercised when ROS message modules are unavailable
    _load_fastlio_camera_offsets = None  # type: ignore[assignment]
    _to_np_points = None  # type: ignore[assignment]

    def _stamp_to_sec(stamp: Any) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _quat_to_matrix(msg: Any) -> np.ndarray:
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


from roi_lidar_corner.lookback_solver import (  # noqa: E402
    DecodedCloudFrame,
    _project_cached_frame,
    prepare_cached_points,
)

try:
    import rclpy
    from rclpy.node import Node
    from cv_bridge import CvBridge
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
except ImportError:  # pragma: no cover - exercised only outside ROS runtime
    rclpy = None
    Node = object  # type: ignore[assignment]
    CvBridge = None  # type: ignore[assignment]
    Odometry = object  # type: ignore[assignment]
    CameraInfo = object  # type: ignore[assignment]
    Image = object  # type: ignore[assignment]
    PointCloud2 = object  # type: ignore[assignment]


@dataclass(frozen=True)
class _BufferedPose:
    stamp: float
    t: np.ndarray
    R: np.ndarray


class ProjectionExposureBuffer:
    def __init__(self, window_sec: float) -> None:
        self.window_sec = float(window_sec)
        self._items: list[tuple[float, np.ndarray, np.ndarray]] = []

    def add_points(self, *, stamp: float, uv: np.ndarray, depth: np.ndarray) -> None:
        uv = np.asarray(uv, dtype=np.float32).reshape(-1, 2)
        depth = np.asarray(depth, dtype=np.float32).reshape(-1)
        if uv.shape[0] != depth.shape[0]:
            raise ValueError("uv and depth must have matching row counts")
        self._items.append((float(stamp), uv, depth))
        self.trim(newest_stamp=float(stamp))

    def trim(self, *, newest_stamp: float) -> None:
        lower = float(newest_stamp) - self.window_sec
        self._items = [item for item in self._items if item[0] >= lower]

    @property
    def points_uv(self) -> np.ndarray:
        if not self._items:
            return np.zeros((0, 2), dtype=np.float32)
        return np.vstack([item[1] for item in self._items]).astype(np.float32, copy=False)

    @property
    def depth(self) -> np.ndarray:
        if not self._items:
            return np.zeros((0,), dtype=np.float32)
        return np.concatenate([item[2] for item in self._items]).astype(np.float32, copy=False)

    @property
    def stamps(self) -> np.ndarray:
        if not self._items:
            return np.zeros((0,), dtype=np.float64)
        return np.concatenate(
            [np.full(item[1].shape[0], item[0], dtype=np.float64) for item in self._items]
        ).astype(np.float64, copy=False)


def _prepare_apply_range_filter(cloud_frame_mode: str) -> bool:
    return str(cloud_frame_mode).strip().lower() not in {"world", "map", "odom"}


def project_points_to_image(
    *,
    points: np.ndarray,
    stamp: float,
    pose: Any,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    image_width: int,
    image_height: int,
    t_cb: np.ndarray,
    R_cb: np.ndarray,
    cloud_frame_mode: str,
    min_range: float,
    max_range: float,
    cache_voxel_size: float,
) -> tuple[np.ndarray, np.ndarray]:
    prepared_points = prepare_cached_points(
        points_world_f32=np.asarray(points, dtype=np.float32),
        min_range=float(min_range),
        max_range=float(max_range),
        voxel_size=float(cache_voxel_size),
        apply_range_filter=_prepare_apply_range_filter(cloud_frame_mode),
    )
    frame = DecodedCloudFrame(
        stamp=float(stamp),
        points_world_f32=np.asarray(prepared_points, dtype=np.float32),
        frame_id=str(cloud_frame_mode),
    )
    points_cam, uv, _, _ = _project_cached_frame(
        frame=frame,
        pose=pose,
        fx=float(fx),
        fy=float(fy),
        cx=float(cx),
        cy=float(cy),
        image_width=int(image_width),
        image_height=int(image_height),
        t_cb=np.asarray(t_cb, dtype=np.float32),
        R_cb=np.asarray(R_cb, dtype=np.float32),
        cloud_frame_mode=str(cloud_frame_mode),
        min_range=float(min_range),
        max_range=float(max_range),
    )
    return uv.astype(np.float32, copy=False), points_cam[:, 2].astype(np.float32, copy=False)


def _depth_to_bgr(depth: float, *, min_depth: float, max_depth: float) -> tuple[int, int, int]:
    if not np.isfinite(depth):
        return (180, 180, 180)
    if max_depth <= min_depth:
        value = 127
    else:
        normalized = np.clip((float(depth) - float(min_depth)) / (float(max_depth) - float(min_depth)), 0.0, 1.0)
        value = int(round((1.0 - normalized) * 255.0))
    color = cv2.applyColorMap(np.asarray([[value]], dtype=np.uint8), cv2.COLORMAP_TURBO)[0, 0]
    return int(color[0]), int(color[1]), int(color[2])


def _canvas_from_latest_image(
    latest_image: np.ndarray | None,
    *,
    camera_width: int,
    camera_height: int,
) -> np.ndarray:
    if latest_image is None:
        width = max(1, int(camera_width))
        height = max(1, int(camera_height))
        return np.zeros((height, width, 3), dtype=np.uint8)

    image = np.asarray(latest_image)
    if image.ndim == 2:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if image.ndim == 3 and image.shape[2] == 4:
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image.astype(np.uint8, copy=True)


def render_projection_exposure_capture(
    *,
    latest_image: np.ndarray | None,
    camera_width: int,
    camera_height: int,
    buffer: ProjectionExposureBuffer,
    duration_sec: float,
    window_sec: float,
    output_image: str,
    output_json: str,
    output_points_npz: str | None,
    image_topic: str,
    pointcloud_topic: str,
    odom_topic: str,
    camera_info_topic: str,
    max_time_diff_odom: float | None = None,
) -> dict[str, Any]:
    overlay = _canvas_from_latest_image(
        latest_image,
        camera_width=int(camera_width),
        camera_height=int(camera_height),
    )
    uv = buffer.points_uv
    depth = buffer.depth
    stamps = buffer.stamps
    finite_depth = depth[np.isfinite(depth)]
    min_depth = float(np.min(finite_depth)) if finite_depth.size else 0.0
    max_depth = float(np.max(finite_depth)) if finite_depth.size else 1.0

    for (u, v), z in zip(uv, depth):
        x = int(round(float(u)))
        y = int(round(float(v)))
        if 0 <= x < overlay.shape[1] and 0 <= y < overlay.shape[0]:
            color = _depth_to_bgr(float(z), min_depth=min_depth, max_depth=max_depth)
            cv2.circle(overlay, (x, y), 1, color, -1, lineType=cv2.LINE_AA)

    metadata = {
        "duration_sec": float(duration_sec),
        "window_sec": float(window_sec),
        "point_count": int(uv.shape[0]),
        "image_topic": str(image_topic),
        "pointcloud_topic": str(pointcloud_topic),
        "odom_topic": str(odom_topic),
        "camera_info_topic": str(camera_info_topic),
    }
    if max_time_diff_odom is not None:
        metadata["max_time_diff_odom"] = float(max_time_diff_odom)
    if output_points_npz:
        metadata["points_npz"] = str(output_points_npz)
        metadata["point_fields"] = ["uv", "depth", "stamp"]

    output_image_path = Path(output_image)
    output_json_path = Path(output_json)
    output_image_path.parent.mkdir(parents=True, exist_ok=True)
    output_json_path.parent.mkdir(parents=True, exist_ok=True)
    if output_points_npz:
        output_points_npz_path = Path(output_points_npz)
        output_points_npz_path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            output_points_npz_path,
            uv=uv.astype(np.float32, copy=False),
            depth=depth.astype(np.float32, copy=False),
            stamp=stamps.astype(np.float64, copy=False),
        )
    if not cv2.imwrite(str(output_image_path), overlay):
        raise RuntimeError(f"Failed to write exposure image: {output_image_path}")
    output_json_path.write_text(json.dumps(metadata, ensure_ascii=False), encoding="utf-8")
    return metadata


class LidarProjectionExposureCaptureNode(Node):  # pragma: no cover - exercised in ROS runtime
    def __init__(
        self,
        *,
        duration_sec: float,
        window_sec: float,
        output_image: str,
        output_json: str,
        output_points_npz: str,
        image_topic: str,
        pointcloud_topic: str,
        odom_topic: str,
        camera_info_topic: str,
        fastlio_config_path: str,
        fastlio_config_file: str,
        min_range: float,
        max_range: float,
        cache_voxel_size: float,
        cloud_frame_mode: str,
        point_stride: int,
        max_time_diff_odom: float,
    ) -> None:
        super().__init__("lidar_projection_exposure_capture")
        if CvBridge is None:
            raise RuntimeError("cv_bridge is required to run lidar_projection_exposure_capture")
        if _load_fastlio_camera_offsets is None or _to_np_points is None:
            raise RuntimeError("corner_lidar_solver_node helpers are required at ROS runtime")

        self.duration_sec = float(duration_sec)
        self.window_sec = float(window_sec)
        self.output_image = output_image
        self.output_json = output_json
        self.output_points_npz = output_points_npz
        self.image_topic = image_topic
        self.pointcloud_topic = pointcloud_topic
        self.odom_topic = odom_topic
        self.camera_info_topic = camera_info_topic
        self.min_range = float(min_range)
        self.max_range = float(max_range)
        self.cache_voxel_size = float(cache_voxel_size)
        self.cloud_frame_mode = str(cloud_frame_mode).strip().lower() or "auto"
        self.point_stride = max(1, int(point_stride))
        self.max_time_diff_odom = max(0.0, float(max_time_diff_odom))

        self.bridge = CvBridge()
        self.buffer = ProjectionExposureBuffer(window_sec=self.window_sec)
        self.odom_buffer: Deque[_BufferedPose] = deque(maxlen=240)
        self.latest_image: np.ndarray | None = None
        self.image_width = 0
        self.image_height = 0
        self.camera_width = 0
        self.camera_height = 0
        self.fx = 0.0
        self.fy = 0.0
        self.cx = 0.0
        self.cy = 0.0
        self.camera_info_ready = False

        self.t_cb, self.R_cb, cfg_source = _load_fastlio_camera_offsets(fastlio_config_path, fastlio_config_file)
        self.get_logger().info(f"Loaded projection camera extrinsics from Fast-LIO config: {cfg_source}")

        self.create_subscription(Image, self.image_topic, self._image_cb, 20)
        self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 50)
        self.create_subscription(PointCloud2, self.pointcloud_topic, self._cloud_cb, 10)

    def _image_cb(self, msg: Any) -> None:
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.image_width = int(self.latest_image.shape[1])
            self.image_height = int(self.latest_image.shape[0])
        except Exception as exc:
            self.get_logger().warning(f"failed to decode image: {exc}")

    def _camera_info_cb(self, msg: Any) -> None:
        self.camera_width = int(msg.width)
        self.camera_height = int(msg.height)
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.camera_info_ready = True

    def _odom_cb(self, msg: Any) -> None:
        stamp = _stamp_to_sec(msg.header.stamp)
        t = np.asarray(
            [
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                float(msg.pose.pose.position.z),
            ],
            dtype=np.float64,
        )
        R = _quat_to_matrix(msg.pose.pose.orientation)
        self.odom_buffer.append(_BufferedPose(stamp=stamp, t=t, R=R))

    def _resolve_cloud_frame_mode(self, frame_id: str) -> str:
        if self.cloud_frame_mode in {"world", "map", "odom"}:
            return "world"
        if self.cloud_frame_mode in {"base", "body", "sensor"}:
            return "base"
        text = (frame_id or "").strip().lower()
        if any(token in text for token in ("map", "odom", "world", "camera_init")):
            return "world"
        return "base"

    def _scaled_intrinsics(self) -> tuple[float, float, float, float, int, int]:
        image_w = self.image_width if self.image_width > 0 else self.camera_width
        image_h = self.image_height if self.image_height > 0 else self.camera_height
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
        return fx, fy, cx, cy, int(image_w), int(image_h)

    def _nearest_odom(self, stamp: float) -> _BufferedPose | None:
        if not self.odom_buffer:
            return None
        best = min(self.odom_buffer, key=lambda item: abs(item.stamp - stamp))
        if abs(float(best.stamp) - float(stamp)) > self.max_time_diff_odom:
            return None
        return best

    def _cloud_cb(self, msg: Any) -> None:
        if not self.camera_info_ready:
            return

        stamp = _stamp_to_sec(msg.header.stamp)
        frame_mode = self._resolve_cloud_frame_mode(str(msg.header.frame_id))
        pose = self._nearest_odom(stamp)
        if pose is None:
            return

        try:
            points = _to_np_points(msg)
        except Exception as exc:
            self.get_logger().warning(f"failed to decode pointcloud: {exc}")
            return
        points = np.asarray(points, dtype=np.float32)
        if self.point_stride > 1:
            points = points[:: self.point_stride]

        fx, fy, cx, cy, image_w, image_h = self._scaled_intrinsics()
        uv, depth = project_points_to_image(
            points=points,
            stamp=stamp,
            pose=pose,
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            image_width=image_w,
            image_height=image_h,
            t_cb=self.t_cb,
            R_cb=self.R_cb,
            cloud_frame_mode=frame_mode,
            min_range=self.min_range,
            max_range=self.max_range,
            cache_voxel_size=self.cache_voxel_size,
        )
        self.buffer.add_points(stamp=stamp, uv=uv, depth=depth)

    def run(self) -> None:
        start = time.time()
        while time.time() - start < self.duration_sec:
            rclpy.spin_once(self, timeout_sec=0.1)

        camera_width = self.image_width if self.image_width > 0 else self.camera_width
        camera_height = self.image_height if self.image_height > 0 else self.camera_height
        metadata = render_projection_exposure_capture(
            latest_image=self.latest_image,
            camera_width=camera_width if camera_width > 0 else 640,
            camera_height=camera_height if camera_height > 0 else 480,
            buffer=self.buffer,
            duration_sec=self.duration_sec,
            window_sec=self.window_sec,
            output_image=self.output_image,
            output_json=self.output_json,
            output_points_npz=self.output_points_npz,
            image_topic=self.image_topic,
            pointcloud_topic=self.pointcloud_topic,
            odom_topic=self.odom_topic,
            camera_info_topic=self.camera_info_topic,
            max_time_diff_odom=self.max_time_diff_odom,
        )
        self.get_logger().info(f"wrote {self.output_image}")
        self.get_logger().info(f"wrote {self.output_json} with {metadata['point_count']} projected points")
        if self.output_points_npz:
            self.get_logger().info(f"wrote {self.output_points_npz}")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Capture a long-exposure LiDAR projection overlay")
    parser.add_argument("--duration-sec", type=float, default=20.0)
    parser.add_argument("--window-sec", type=float, default=20.0)
    parser.add_argument("--output-image", default="/tmp/lidar_projection_exposure.png")
    parser.add_argument("--output-json", default="/tmp/lidar_projection_exposure.json")
    parser.add_argument("--output-points-npz", default="")
    parser.add_argument("--image-topic", default="/camera/color/image_raw")
    parser.add_argument("--pointcloud-topic", default="/cloud_registered")
    parser.add_argument("--odom-topic", default="/Odometry")
    parser.add_argument("--camera-info-topic", default="/camera/color/camera_info")
    parser.add_argument("--fastlio-config-path", default="")
    parser.add_argument("--fastlio-config-file", default="")
    parser.add_argument("--min-range", type=float, default=0.2)
    parser.add_argument("--max-range", type=float, default=30.0)
    parser.add_argument("--cache-voxel-size", type=float, default=0.1)
    parser.add_argument("--cloud-frame-mode", default="auto")
    parser.add_argument("--point-stride", type=int, default=1)
    parser.add_argument("--max-time-diff-odom", type=float, default=0.5)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:  # pragma: no cover - exercised in ROS runtime
    args = parse_args(argv)
    if rclpy is None:
        raise RuntimeError("rclpy is required to run lidar_projection_exposure_capture")
    rclpy.init(args=None)
    node = LidarProjectionExposureCaptureNode(
        duration_sec=args.duration_sec,
        window_sec=args.window_sec,
        output_image=args.output_image,
        output_json=args.output_json,
        output_points_npz=args.output_points_npz,
        image_topic=args.image_topic,
        pointcloud_topic=args.pointcloud_topic,
        odom_topic=args.odom_topic,
        camera_info_topic=args.camera_info_topic,
        fastlio_config_path=args.fastlio_config_path,
        fastlio_config_file=args.fastlio_config_file,
        min_range=args.min_range,
        max_range=args.max_range,
        cache_voxel_size=args.cache_voxel_size,
        cloud_frame_mode=args.cloud_frame_mode,
        point_stride=args.point_stride,
        max_time_diff_odom=args.max_time_diff_odom,
    )
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":  # pragma: no cover - exercised in ROS runtime
    raise SystemExit(main())
