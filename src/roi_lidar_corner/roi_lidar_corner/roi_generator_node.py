#!/usr/bin/env python3

from __future__ import annotations

import json
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String

from roi_lidar_corner.detection_runtime import (
    Detection,
    create_detector,
    resolve_resource_path,
)
from roi_lidar_corner.msg import (
    Corner3DArray,
    CornerROI,
    FrontFaceROI,
    FrontFaceROIArray,
    ObjectROI,
    StructureROI,
)
from rotor_swarm_msgs.msg import FrontFaceCorners
from roi_lidar_corner.roi_geometry_prior import (
    GeometryPriorConfig,
    StructureCandidate,
    TemporalPriorConfig,
    build_bbox_corners,
    validate_structure_candidate,
)
from roi_lidar_corner.neo_roi_refiner import refine_corners_inside_bbox
from roi_lidar_corner.structure_roi_builder import build_structure_lines, dilate_line_mask

CORNER_LABELS = {
    0: "TL",
    1: "TR",
    2: "BL",
    3: "BR",
}

CORNER_COLORS = {
    0: (0, 0, 255),
    1: (0, 255, 0),
    2: (255, 0, 0),
    3: (0, 255, 255),
}


def _default_share_file(*parts: str) -> str:
    try:
        return str(Path(get_package_share_directory("roi_lidar_corner"), *parts))
    except Exception:
        return ""


def _parse_detector_class_filter(raw_value: str) -> List[int]:
    text = str(raw_value).strip()
    if not text:
        return []

    try:
        parsed = json.loads(text)
    except json.JSONDecodeError:
        text = text.strip("[]").strip()
        if not text:
            return []
        parsed = [item.strip() for item in text.split(",") if item.strip()]

    if isinstance(parsed, (int, float)):
        values = [parsed]
    elif isinstance(parsed, (list, tuple)):
        values = list(parsed)
    else:
        return []

    class_filter: List[int] = []
    for value in values:
        class_id = int(value)
        if class_id >= 0:
            class_filter.append(class_id)
    return class_filter


def _build_corner_roi(
    header: Header,
    object_id: int,
    class_id: int,
    conf: float,
    corner_label: int,
    corner_uv: Tuple[float, float],
    img_shape: Tuple[int, int],
    radius: int,
) -> Optional[CornerROI]:
    h, w = img_shape[:2]
    if h <= 0 or w <= 0:
        return None

    cx = int(round(float(corner_uv[0])))
    cy = int(round(float(corner_uv[1])))
    cx = max(0, min(w - 1, cx))
    cy = max(0, min(h - 1, cy))
    radius = max(1, int(round(float(radius))))

    if int(corner_label) == 0:
        x0 = cx
        x1 = min(w, cx + radius + 1)
        y0 = cy
        y1 = min(h, cy + radius + 1)
    elif int(corner_label) == 1:
        x0 = max(0, cx - radius)
        x1 = min(w, cx + 1)
        y0 = cy
        y1 = min(h, cy + radius + 1)
    elif int(corner_label) == 2:
        x0 = cx
        x1 = min(w, cx + radius + 1)
        y0 = max(0, cy - radius)
        y1 = min(h, cy + 1)
    else:
        x0 = max(0, cx - radius)
        x1 = min(w, cx + 1)
        y0 = max(0, cy - radius)
        y1 = min(h, cy + 1)
    if x1 <= x0 or y1 <= y0:
        return None

    mask = np.ones((y1 - y0, x1 - x0), dtype=np.uint8) * 255

    msg = CornerROI()
    msg.header = header
    msg.object_id = int(object_id)
    msg.class_id = int(class_id)
    msg.conf = float(conf)
    msg.corner_label = int(corner_label)
    msg.mask_origin_x = int(x0)
    msg.mask_origin_y = int(y0)
    msg.mask_width = int(mask.shape[1])
    msg.mask_height = int(mask.shape[0])
    msg.corner_u = float(cx)
    msg.corner_v = float(cy)
    msg.roi_mask = [int(v) for v in mask.flatten().tolist()]
    msg.valid = True
    return msg


def _scaled_corner_radius(
    bbox: Sequence[float],
    base_radius: int,
    reference_short_side: float = 245.0,
) -> int:
    x1, y1, x2, y2 = [float(v) for v in bbox]
    short_side = max(1.0, min(abs(x2 - x1), abs(y2 - y1)))
    scale = short_side / max(1.0, float(reference_short_side))
    scaled = int(round(float(base_radius) * scale))
    return max(3, min(96, scaled))


class RoiGeneratorNode(Node):
    def __init__(self) -> None:
        super().__init__("roi_generator_node")

        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("roi_output_topic", "/roi_lidar_corner/front_face_rois")
        self.declare_parameter("corner_radius", 5)
        self.declare_parameter("post_mask_half_width", 5)
        self.declare_parameter("beam_mask_half_width", 5)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("debug_image_topic", "/roi_lidar_corner/roi_debug")
        self.declare_parameter("corner3d_topic", "/roi_lidar_corner/corners3d")
        self.declare_parameter("solver_debug_uv_topic", "/roi_lidar_corner/solver_debug_uv")
        self.declare_parameter("subscribe_corner3d_debug", True)
        self.declare_parameter("subscribe_solver_debug_uv", True)

        self.declare_parameter("detector_backend", "pt")
        self.declare_parameter("detector_model_path", _default_share_file("models", "best.pt"))
        self.declare_parameter("detector_names_path", _default_share_file("models", "detect.names"))
        self.declare_parameter("detector_conf_threshold", 0.25)
        self.declare_parameter("detector_iou_threshold", 0.45)
        self.declare_parameter("detector_input_size", 640)
        self.declare_parameter("detector_use_gpu", True)
        self.declare_parameter("detector_class_filter", "[]")

        self.declare_parameter("neo_canny_low", 50)
        self.declare_parameter("neo_canny_high", 200)
        self.declare_parameter("neo_hough_threshold", 50)
        self.declare_parameter("neo_min_line_length", 50)
        self.declare_parameter("neo_max_line_gap", 50)
        self.declare_parameter("neo_blur_kernel_size", 5)
        self.declare_parameter("neo_border_ratio", 0.15)
        self.declare_parameter("roi_enable_geometry_prior", True)
        self.declare_parameter("roi_enable_temporal_prior", True)
        self.declare_parameter("roi_temporal_hold_frames", 5)
        self.declare_parameter("roi_max_line_jump_px", 80.0)
        self.declare_parameter("roi_min_post_bbox_height_ratio", 0.45)
        self.declare_parameter("roi_expected_top_post_ratio", 0.5)
        self.declare_parameter("roi_top_post_ratio_tolerance", 0.25)
        self.declare_parameter("roi_border_relax_px", 8.0)

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.roi_output_topic = self.get_parameter("roi_output_topic").get_parameter_value().string_value
        self.corner_radius = int(self.get_parameter("corner_radius").get_parameter_value().integer_value)
        self.post_mask_half_width = int(
            self.get_parameter("post_mask_half_width").get_parameter_value().integer_value
        )
        self.beam_mask_half_width = int(
            self.get_parameter("beam_mask_half_width").get_parameter_value().integer_value
        )
        self.publish_debug_image = self.get_parameter("publish_debug_image").get_parameter_value().bool_value
        self.debug_image_topic = self.get_parameter("debug_image_topic").get_parameter_value().string_value
        self.corner3d_topic = self.get_parameter("corner3d_topic").get_parameter_value().string_value
        self.solver_debug_uv_topic = self.get_parameter("solver_debug_uv_topic").get_parameter_value().string_value
        self.subscribe_corner3d_debug = (
            self.get_parameter("subscribe_corner3d_debug").get_parameter_value().bool_value
        )
        self.subscribe_solver_debug_uv = (
            self.get_parameter("subscribe_solver_debug_uv").get_parameter_value().bool_value
        )

        detector_backend = self.get_parameter("detector_backend").get_parameter_value().string_value
        detector_model_path = self.get_parameter("detector_model_path").get_parameter_value().string_value
        detector_names_path = self.get_parameter("detector_names_path").get_parameter_value().string_value
        detector_conf_threshold = float(
            self.get_parameter("detector_conf_threshold").get_parameter_value().double_value
        )
        detector_iou_threshold = float(
            self.get_parameter("detector_iou_threshold").get_parameter_value().double_value
        )
        detector_input_size = int(self.get_parameter("detector_input_size").get_parameter_value().integer_value)
        detector_use_gpu = self.get_parameter("detector_use_gpu").get_parameter_value().bool_value
        detector_class_filter = _parse_detector_class_filter(
            self.get_parameter("detector_class_filter").get_parameter_value().string_value
        )

        self.neo_canny_low = int(self.get_parameter("neo_canny_low").get_parameter_value().integer_value)
        self.neo_canny_high = int(self.get_parameter("neo_canny_high").get_parameter_value().integer_value)
        self.neo_hough_threshold = int(self.get_parameter("neo_hough_threshold").get_parameter_value().integer_value)
        self.neo_min_line_length = int(
            self.get_parameter("neo_min_line_length").get_parameter_value().integer_value
        )
        self.neo_max_line_gap = int(self.get_parameter("neo_max_line_gap").get_parameter_value().integer_value)
        self.neo_blur_kernel_size = int(
            self.get_parameter("neo_blur_kernel_size").get_parameter_value().integer_value
        )
        self.neo_border_ratio = float(self.get_parameter("neo_border_ratio").get_parameter_value().double_value)
        self.roi_enable_geometry_prior = (
            self.get_parameter("roi_enable_geometry_prior").get_parameter_value().bool_value
        )
        self.roi_enable_temporal_prior = (
            self.get_parameter("roi_enable_temporal_prior").get_parameter_value().bool_value
        )
        self.roi_temporal_hold_frames = int(
            self.get_parameter("roi_temporal_hold_frames").get_parameter_value().integer_value
        )
        self.geometry_prior_config = GeometryPriorConfig(
            min_post_bbox_height_ratio=float(
                self.get_parameter("roi_min_post_bbox_height_ratio").get_parameter_value().double_value
            ),
            expected_top_post_ratio=float(
                self.get_parameter("roi_expected_top_post_ratio").get_parameter_value().double_value
            ),
            top_post_ratio_tolerance=float(
                self.get_parameter("roi_top_post_ratio_tolerance").get_parameter_value().double_value
            ),
            border_relax_px=float(self.get_parameter("roi_border_relax_px").get_parameter_value().double_value),
        )
        self.temporal_prior_config = TemporalPriorConfig(
            max_line_jump_px=float(self.get_parameter("roi_max_line_jump_px").get_parameter_value().double_value)
        )
        self.last_valid_candidate = None
        self.last_valid_corners = None
        self.last_valid_detection = None
        self.missed_detection_frames = 0
        self.debug_log_every_n_frames = 30
        self.frame_counter = 0

        script_path = Path(__file__)
        if detector_backend.strip().lower() == "onnx":
            detector_model_path = resolve_resource_path(
                detector_model_path,
                script_path,
                defaults=("models/best_detect.onnx",),
            )
            detector_names_path = resolve_resource_path(
                detector_names_path,
                script_path,
                defaults=("models/detect.names",),
            )
        else:
            detector_model_path = resolve_resource_path(
                detector_model_path,
                script_path,
                defaults=("models/best.pt", "scripts/best.pt", "model/best.pt"),
            )

        self.bridge = CvBridge()
        self.detector = None
        self.latest_corner3d = {}
        self.latest_solver_debug_uv = {}
        self.latest_projected_cloud_uv = []
        self.latest_corner3d_stamp: Optional[float] = None
        self.latest_solver_debug_uv_stamp: Optional[float] = None
        if detector_model_path:
            self.detector = create_detector(
                backend=detector_backend,
                detector_model_path=detector_model_path,
                detector_names_path=detector_names_path,
                detector_conf_threshold=detector_conf_threshold,
                detector_iou_threshold=detector_iou_threshold,
                detector_input_size=detector_input_size,
                detector_use_gpu=detector_use_gpu,
                detector_class_filter=detector_class_filter,
                ultralytics_device="" if detector_use_gpu else "cpu",
                logger=self.get_logger(),
            )
        else:
            self.get_logger().error(f"detector model not found for backend={detector_backend}")

        self.publisher = self.create_publisher(FrontFaceROIArray, self.roi_output_topic, 10)
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.corner3d_sub = None
        self.solver_debug_uv_sub = None
        if self.publish_debug_image and self.subscribe_corner3d_debug:
            self.corner3d_sub = self.create_subscription(
                Corner3DArray,
                self.corner3d_topic,
                self.corner3d_callback,
                10,
            )
        if self.publish_debug_image and self.subscribe_solver_debug_uv:
            self.solver_debug_uv_sub = self.create_subscription(
                String,
                self.solver_debug_uv_topic,
                self.solver_debug_uv_callback,
                10,
            )
        self.cv_publish_image = (
            self.create_publisher(Image, self.debug_image_topic, 10)
            if self.publish_debug_image
            else None
        )

        self.get_logger().info(f"ROI topic -> {self.roi_output_topic}, image topic -> {image_topic}")
        self.get_logger().info(
            f"detector backend = {detector_backend}, model = {detector_model_path or '<missing>'}"
        )
        if detector_names_path:
            self.get_logger().info(f"detector names = {detector_names_path}")
        if self.publish_debug_image:
            self.get_logger().info(f"publish debug image on {self.debug_image_topic}")

    def corner3d_callback(self, msg: Corner3DArray) -> None:
        latest = {}
        for corner in msg.corners:
            latest[(int(corner.object_id), int(corner.corner_label))] = corner
        self.latest_corner3d = latest
        self.latest_corner3d_stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def solver_debug_uv_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        latest = {}
        for obj in payload.get("objects", []):
            object_id = int(obj.get("object_id", -1))
            for corner in obj.get("corners", []):
                key = (object_id, int(corner.get("corner_label", -1)))
                latest[key] = {
                    "valid": bool(corner.get("valid", False)),
                    "uv": corner.get("uv", []),
                }
        self.latest_solver_debug_uv = latest
        self.latest_projected_cloud_uv = payload.get("cloud_uv", [])
        stamp_value = payload.get("stamp")
        self.latest_solver_debug_uv_stamp = float(stamp_value) if stamp_value is not None else None

    def _image_stamp(self, image_msg: Image) -> float:
        return float(image_msg.header.stamp.sec) + float(image_msg.header.stamp.nanosec) * 1e-9

    def _is_solver_debug_fresh(self, image_msg: Image) -> bool:
        if self.latest_solver_debug_uv_stamp is None:
            return False
        return abs(self.latest_solver_debug_uv_stamp - self._image_stamp(image_msg)) <= 0.2

    def _is_corner3d_fresh(self, image_msg: Image) -> bool:
        if self.latest_corner3d_stamp is None:
            return False
        return abs(self.latest_corner3d_stamp - self._image_stamp(image_msg)) <= 0.2

    def image_callback(self, msg: Image) -> None:
        self.frame_counter += 1
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"decode image failed: {exc}")
            return

        h, w = cv_image.shape[:2]
        detections = ()
        fallback_count = 0
        if self.detector is not None and getattr(self.detector, "available", True):
            try:
                detections = self.detector.detect(cv_image).detections
            except Exception as exc:
                self.get_logger().warn(f"detector inference failed: {exc}")

        objects: List[FrontFaceROI] = []
        for det_idx, detection in enumerate(detections):
            source = "corner_refined"
            try:
                corners = refine_corners_inside_bbox(
                    cv_image,
                    detection.bbox,
                    canny_low=self.neo_canny_low,
                    canny_high=self.neo_canny_high,
                    hough_threshold=self.neo_hough_threshold,
                    min_line_length=self.neo_min_line_length,
                    max_line_gap=self.neo_max_line_gap,
                    blur_kernel_size=self.neo_blur_kernel_size,
                    border_size=None,
                    border_ratio=self.neo_border_ratio,
                )
            except Exception as exc:
                self.get_logger().warn(f"neo refine failed, fallback to bbox corners: {exc}")
                corners = build_bbox_corners(detection.bbox)
                source = "bbox_fallback"
                fallback_count += 1

            bbox_corners = build_bbox_corners(detection.bbox)
            selected_corners = list(corners[:4])
            while len(selected_corners) < 4:
                selected_corners.append(bbox_corners[len(selected_corners)])

            if self.roi_enable_geometry_prior:
                candidate = self._candidate_from_corners(selected_corners)
                validation = validate_structure_candidate(
                    candidate,
                    bbox=detection.bbox,
                    image_shape=cv_image.shape[:2],
                    config=self.geometry_prior_config,
                )
                if not validation.valid:
                    selected_corners = bbox_corners
                    if source != "bbox_fallback":
                        source = "bbox_fallback"
                        fallback_count += 1

            object_roi = self._build_front_face_roi(
                header=msg.header,
                object_id=det_idx,
                detection=detection,
                corners=selected_corners,
                image_shape=cv_image.shape,
                source=source,
            )
            if object_roi is not None:
                objects.append(object_roi)

        out = FrontFaceROIArray()
        out.header = msg.header
        out.objects = objects
        self.publisher.publish(out)

        if self.frame_counter % self.debug_log_every_n_frames == 0:
            object_corner_counts = [len(getattr(obj, "structures", [])) for obj in objects]
            corner_texts = []
            for obj in objects:
                ordered = sorted(getattr(obj, "structures", []), key=lambda item: int(item.structure_label))
                entries = []
                for roi in ordered:
                    label = {
                        StructureROI.LEFT_POST: "left_post",
                        StructureROI.RIGHT_POST: "right_post",
                        StructureROI.TOP_BEAM: "top_beam",
                    }.get(int(roi.structure_label), str(int(roi.structure_label)))
                    entries.append(
                        f"{label}=({roi.line_u0:.1f},{roi.line_v0:.1f})->({roi.line_u1:.1f},{roi.line_v1:.1f})"
                    )
                corner_texts.append(f"obj{int(obj.object_id)}[" + ", ".join(entries) + "]")
            self.get_logger().info(
                f"frame={self.frame_counter} detections={len(detections)} published_objects={len(objects)} "
                f"corner_slots={object_corner_counts} neo_fallback_objects={fallback_count} "
                f"projected_cloud_points={len(self.latest_projected_cloud_uv)} "
                f"corners={' ; '.join(corner_texts) if corner_texts else '[]'}"
            )

        if self.cv_publish_image is not None:
            debug = cv_image.copy()
            draw_solver_debug = self._is_solver_debug_fresh(msg)
            draw_corner3d_debug = self._is_corner3d_fresh(msg)
            if draw_solver_debug:
                self._draw_projected_cloud(debug)
            self._draw_detections(debug, detections)
            self._draw_corner_rois(debug, objects, draw_solver_debug, draw_corner3d_debug)
            self.cv_publish_image.publish(self.bridge.cv2_to_imgmsg(debug, encoding="bgr8"))

    def _draw_projected_cloud(self, image: np.ndarray) -> None:
        for uv in self.latest_projected_cloud_uv:
            if len(uv) != 2:
                continue
            u = int(uv[0])
            v = int(uv[1])
            if u < 0 or v < 0 or u >= image.shape[1] or v >= image.shape[0]:
                continue
            cv2.circle(image, (u, v), 2, (160, 160, 160), -1)

    def _build_front_face_roi(
        self,
        header: Header,
        object_id: int,
        detection: Detection,
        corners: Sequence[Tuple[float, float]],
        image_shape: Tuple[int, int, int],
        source: str = "corner_refined",
    ) -> Optional[FrontFaceROI]:
        obj = FrontFaceROI()
        obj.header = header
        obj.object_id = int(object_id)
        obj.class_id = int(detection.class_id)
        obj.conf = float(detection.conf)
        obj.bbox_xyxy = [float(v) for v in detection.bbox]

        x1, y1, x2, y2 = detection.bbox
        fallback_corners = [
            (float(x1), float(y1)),
            (float(x2), float(y1)),
            (float(x1), float(y2)),
            (float(x2), float(y2)),
        ]

        ordered_corners = list(corners[:4])
        while len(ordered_corners) < 4:
            ordered_corners.append(fallback_corners[len(ordered_corners)])

        obj.structures = self._build_structure_rois(
            header=header,
            object_id=object_id,
            detection=detection,
            corners=ordered_corners,
            image_shape=image_shape,
            source=source,
        )
        return obj

    def _candidate_from_corners(self, corners: Sequence[Tuple[float, float]]) -> StructureCandidate:
        corners_by_label = {
            "TL": tuple(corners[0]),
            "TR": tuple(corners[1]),
            "BL": tuple(corners[2]),
            "BR": tuple(corners[3]),
        }
        return StructureCandidate(lines=build_structure_lines(corners_by_label))

    def _build_structure_rois(
        self,
        header: Header,
        object_id: int,
        detection: Detection,
        corners: Sequence[Tuple[float, float]],
        image_shape: Tuple[int, int, int],
        source: str = "corner_refined",
    ) -> List[StructureROI]:
        corners_by_label = {
            "TL": tuple(corners[0]),
            "TR": tuple(corners[1]),
            "BL": tuple(corners[2]),
            "BR": tuple(corners[3]),
        }
        lines = build_structure_lines(corners_by_label)
        configs = (
            ("left_post", StructureROI.LEFT_POST, self.post_mask_half_width),
            ("right_post", StructureROI.RIGHT_POST, self.post_mask_half_width),
            ("top_beam", StructureROI.TOP_BEAM, self.beam_mask_half_width),
        )
        structures: List[StructureROI] = []
        for name, label, half_width in configs:
            start, end = lines[name]
            mask = dilate_line_mask(image_shape[:2], start, end, half_width=half_width)
            ys, xs = np.where(mask > 0)
            if xs.size == 0 or ys.size == 0:
                continue
            x0 = int(xs.min())
            y0 = int(ys.min())
            x1 = int(xs.max()) + 1
            y1 = int(ys.max()) + 1
            msg = StructureROI()
            msg.header = header
            msg.object_id = int(object_id)
            msg.class_id = int(detection.class_id)
            msg.conf = float(detection.conf)
            msg.structure_label = int(label)
            msg.mask_origin_x = x0
            msg.mask_origin_y = y0
            msg.mask_width = x1 - x0
            msg.mask_height = y1 - y0
            msg.roi_mask = [int(v) for v in mask[y0:y1, x0:x1].reshape(-1).tolist()]
            msg.line_u0 = float(start[0])
            msg.line_v0 = float(start[1])
            msg.line_u1 = float(end[0])
            msg.line_v1 = float(end[1])
            msg.valid = True
            msg.structure_conf = float(detection.conf)
            msg.source = str(source)
            structures.append(msg)

        return structures

    def _draw_detections(self, image: np.ndarray, detections: Sequence[Detection]) -> None:
        for detection in detections:
            x1, y1, x2, y2 = [int(round(v)) for v in detection.bbox]
            cv2.rectangle(image, (x1, y1), (x2, y2), (255, 128, 0), 2)
            label = detection.class_name or str(detection.class_id)
            text = f"{label}:{detection.conf:.2f}"
            cv2.putText(
                image,
                text,
                (x1, max(18, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 128, 0),
                2,
            )

    def _draw_corner_rois(
        self,
        image: np.ndarray,
        objects: Sequence[object],
        draw_solver_debug: bool,
        draw_corner3d_debug: bool,
    ) -> None:
        for obj in objects:
            if hasattr(obj, "structures") and not hasattr(obj, "corner_rois"):
                self._draw_structure_rois(image, obj)
                continue
            ordered_rois = sorted(obj.corner_rois, key=lambda item: int(item.corner_label))
            corners_by_label = {}

            for roi in ordered_rois:
                x0 = roi.mask_origin_x
                y0 = roi.mask_origin_y
                mask = np.array(roi.roi_mask, dtype=np.uint8).reshape((roi.mask_height, roi.mask_width))
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for contour in contours:
                    contour[:, :, 0] += x0
                    contour[:, :, 1] += y0
                    cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)
                label_id = int(roi.corner_label)
                px = int(round(roi.corner_u))
                py = int(round(roi.corner_v))
                corners_by_label[label_id] = (px, py)
                color = CORNER_COLORS.get(label_id, (255, 255, 255))
                cross_half = max(6, int(round(self.corner_radius * 0.55)))
                cv2.line(image, (px - cross_half, py), (px + cross_half, py), color, 2)
                cv2.line(image, (px, py - cross_half), (px, py + cross_half), color, 2)
                label = CORNER_LABELS.get(label_id, str(label_id))
                text_offsets = {
                    0: (-56, -12),
                    1: (10, -12),
                    2: (-56, 24),
                    3: (10, 24),
                }
                dx, dy = text_offsets.get(label_id, (8, -8))
                corner3d = self.latest_corner3d.get((int(obj.object_id), label_id)) if draw_corner3d_debug else None
                if corner3d is None:
                    detail_text = f"{label} pts=? xyz=(?, ?, ?)"
                elif bool(corner3d.valid):
                    detail_text = (
                        f"{label} pts={int(corner3d.support_point_count)} "
                        f"xyz=({float(corner3d.x):.2f}, {float(corner3d.y):.2f}, {float(corner3d.z):.2f})"
                    )
                else:
                    detail_text = f"{label} pts={int(corner3d.support_point_count)} xyz=(invalid)"
                cv2.putText(
                    image,
                    detail_text,
                    (px + dx, py + dy),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    color,
                    2,
                )
                solver_debug = self.latest_solver_debug_uv.get((int(obj.object_id), label_id)) if draw_solver_debug else None
                if solver_debug is not None:
                    point_color = color if solver_debug.get("valid", False) else (80, 80, 80)
                    for uv in solver_debug.get("uv", []):
                        if len(uv) != 2:
                            continue
                        u = int(uv[0])
                        v = int(uv[1])
                        cv2.circle(image, (u, v), 1, point_color, -1)

            edge_pairs = ((0, 1), (1, 3), (3, 2), (2, 0))
            for a, b in edge_pairs:
                if a in corners_by_label and b in corners_by_label:
                    cv2.line(image, corners_by_label[a], corners_by_label[b], (200, 200, 200), 2)

    def _draw_structure_rois(self, image: np.ndarray, obj: FrontFaceROI) -> None:
        colors = {
            StructureROI.LEFT_POST: (0, 255, 0),
            StructureROI.RIGHT_POST: (255, 0, 0),
            StructureROI.TOP_BEAM: (0, 255, 255),
        }
        for roi in getattr(obj, "structures", []):
            if not getattr(roi, "valid", False):
                continue
            mask = np.array(roi.roi_mask, dtype=np.uint8).reshape((roi.mask_height, roi.mask_width))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            color = colors.get(int(roi.structure_label), (255, 255, 255))
            for contour in contours:
                contour[:, :, 0] += int(roi.mask_origin_x)
                contour[:, :, 1] += int(roi.mask_origin_y)
                cv2.drawContours(image, [contour], -1, color, 2)
            start = (int(round(roi.line_u0)), int(round(roi.line_v0)))
            end = (int(round(roi.line_u1)), int(round(roi.line_v1)))
            cv2.line(image, start, end, color, 2)


def main() -> None:
    rclpy.init()
    node = RoiGeneratorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
