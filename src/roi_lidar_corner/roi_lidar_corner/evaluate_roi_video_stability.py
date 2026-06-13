#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import importlib.util
import statistics
import sys
import time
import types
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


STRUCTURE_NAMES = {
    0: "left",
    1: "right",
    2: "top",
}

CSV_FIELDS = [
    "frame",
    "objects",
    "conf",
    "bbox_cx",
    "bbox_cy",
    "bbox_w",
    "bbox_h",
    "left_mx",
    "left_my",
    "left_len",
    "right_mx",
    "right_my",
    "right_len",
    "top_mx",
    "top_my",
    "top_len",
]


@dataclass(frozen=True)
class LineMetrics:
    mid_x: float
    mid_y: float
    length: float
    u0: Optional[float] = None
    v0: Optional[float] = None
    u1: Optional[float] = None
    v1: Optional[float] = None


@dataclass(frozen=True)
class ObjectMetrics:
    conf: float
    bbox_xyxy: Tuple[float, float, float, float]
    lines: Mapping[str, LineMetrics]


@dataclass(frozen=True)
class FrameResult:
    frame: int
    objects: Sequence[ObjectMetrics]


def _mean(values: Sequence[float]) -> Optional[float]:
    return float(statistics.mean(values)) if values else None


def _delta_summary(values: Sequence[float]) -> Optional[Dict[str, float]]:
    if len(values) < 2:
        return None
    deltas = [abs(float(values[index]) - float(values[index - 1])) for index in range(1, len(values))]
    ordered = sorted(deltas)
    p95_index = int(round((len(ordered) - 1) * 0.95))
    return {
        "mean": float(statistics.mean(deltas)),
        "p95": float(ordered[p95_index]),
        "max": float(max(deltas)),
    }


def _primary_objects(results: Iterable[FrameResult]) -> List[ObjectMetrics]:
    return [result.objects[0] for result in results if result.objects]


def summarize_results(results: Sequence[FrameResult], elapsed_sec: float) -> Dict[str, object]:
    primary = _primary_objects(results)
    missed = [result.frame for result in results if not result.objects]
    multi = [result.frame for result in results if len(result.objects) > 1]
    summary: Dict[str, object] = {
        "frames": len(results),
        "elapsed_sec": float(elapsed_sec),
        "throughput_fps": float(len(results) / elapsed_sec) if elapsed_sec > 0.0 else 0.0,
        "detected_frames": len(primary),
        "missed_frames": len(missed),
        "multi_object_frames": len(multi),
        "missed_frame_indices": missed,
        "multi_frame_indices": multi,
    }
    if not primary:
        return summary

    confs = [item.conf for item in primary]
    summary["conf_min"] = float(min(confs))
    summary["conf_mean"] = float(statistics.mean(confs))

    bbox_cx = []
    bbox_cy = []
    bbox_w = []
    bbox_h = []
    for item in primary:
        x1, y1, x2, y2 = item.bbox_xyxy
        bbox_cx.append((x1 + x2) / 2.0)
        bbox_cy.append((y1 + y2) / 2.0)
        bbox_w.append(x2 - x1)
        bbox_h.append(y2 - y1)
    for name, values in (
        ("bbox_cx_delta_px", bbox_cx),
        ("bbox_cy_delta_px", bbox_cy),
        ("bbox_w_delta_px", bbox_w),
        ("bbox_h_delta_px", bbox_h),
    ):
        value = _delta_summary(values)
        if value is not None:
            summary[name] = value

    for structure_name in ("left", "right", "top"):
        line_values = [item.lines.get(structure_name) for item in primary if structure_name in item.lines]
        for suffix, values in (
            ("mx", [line.mid_x for line in line_values if line is not None]),
            ("my", [line.mid_y for line in line_values if line is not None]),
            ("len", [line.length for line in line_values if line is not None]),
        ):
            value = _delta_summary(values)
            if value is not None:
                summary[f"{structure_name}_{suffix}_delta_px"] = value
    return summary


def _format_float(value: Optional[float]) -> str:
    return "" if value is None else f"{float(value):.6f}"


def _object_to_row(result: FrameResult) -> Dict[str, str]:
    row = {field: "" for field in CSV_FIELDS}
    row["frame"] = str(int(result.frame))
    row["objects"] = str(len(result.objects))
    if not result.objects:
        return row

    obj = result.objects[0]
    x1, y1, x2, y2 = obj.bbox_xyxy
    row["conf"] = _format_float(obj.conf)
    row["bbox_cx"] = _format_float((x1 + x2) / 2.0)
    row["bbox_cy"] = _format_float((y1 + y2) / 2.0)
    row["bbox_w"] = _format_float(x2 - x1)
    row["bbox_h"] = _format_float(y2 - y1)
    for structure_name, line in obj.lines.items():
        if structure_name not in {"left", "right", "top"}:
            continue
        row[f"{structure_name}_mx"] = _format_float(line.mid_x)
        row[f"{structure_name}_my"] = _format_float(line.mid_y)
        row[f"{structure_name}_len"] = _format_float(line.length)
    return row


def write_results_csv(results: Sequence[FrameResult], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for result in results:
            writer.writerow(_object_to_row(result))


def _point(x: float, y: float) -> Tuple[int, int]:
    return (int(round(float(x))), int(round(float(y))))


def _line_endpoints(name: str, line: LineMetrics) -> Tuple[Tuple[int, int], Tuple[int, int]]:
    if None not in (line.u0, line.v0, line.u1, line.v1):
        return _point(line.u0, line.v0), _point(line.u1, line.v1)
    half = float(line.length) * 0.5
    if name in {"left", "right"}:
        return _point(line.mid_x, line.mid_y - half), _point(line.mid_x, line.mid_y + half)
    return _point(line.mid_x - half, line.mid_y), _point(line.mid_x + half, line.mid_y)


def render_annotated_frame(image, result: FrameResult):
    import cv2

    annotated = image.copy()
    palette = {
        "left": (0, 255, 255),
        "right": (255, 255, 0),
        "top": (0, 255, 0),
    }
    cv2.putText(
        annotated,
        f"frame={result.frame} objects={len(result.objects)}",
        (12, 26),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    if not result.objects:
        cv2.putText(
            annotated,
            "MISS",
            (12, 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        return annotated

    for object_index, obj in enumerate(result.objects):
        x1, y1, x2, y2 = obj.bbox_xyxy
        cv2.rectangle(annotated, _point(x1, y1), _point(x2, y2), (255, 0, 0), 2)
        cv2.putText(
            annotated,
            f"id={object_index} conf={obj.conf:.2f}",
            _point(x1, max(16.0, y1 - 6.0)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 0, 0),
            2,
            cv2.LINE_AA,
        )
        for name, line in obj.lines.items():
            color = palette.get(name, (255, 255, 255))
            start, end = _line_endpoints(name, line)
            cv2.line(annotated, start, end, color, 3, cv2.LINE_AA)
            cv2.circle(annotated, _point(line.mid_x, line.mid_y), 4, color, -1)
            cv2.putText(
                annotated,
                f"{name}:{line.length:.0f}px",
                _point(line.mid_x + 6.0, line.mid_y - 6.0),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
                cv2.LINE_AA,
            )
    return annotated


def _open_video_writer(path: Path, fps: float, width: int, height: int):
    import cv2

    path.parent.mkdir(parents=True, exist_ok=True)
    for fourcc_name in ("mp4v", "avc1", "XVID"):
        writer = cv2.VideoWriter(
            str(path),
            cv2.VideoWriter_fourcc(*fourcc_name),
            float(fps),
            (int(width), int(height)),
        )
        if writer.isOpened():
            return writer
        writer.release()
    raise RuntimeError(f"failed to open output video writer: {path}")


def _install_module(name: str, module: types.ModuleType) -> None:
    sys.modules[name] = module


def _install_fake_ros_modules(package_root: Path, parameter_overrides: Mapping[str, object]) -> None:
    ament_index_python = types.ModuleType("ament_index_python")
    ament_packages = types.ModuleType("ament_index_python.packages")
    ament_packages.get_package_share_directory = lambda _package_name: str(package_root)
    _install_module("ament_index_python", ament_index_python)
    _install_module("ament_index_python.packages", ament_packages)

    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_qos = types.ModuleType("rclpy.qos")

    class FakeLogger:
        def info(self, *_args, **_kwargs) -> None:
            pass

        def warn(self, *args, **_kwargs) -> None:
            print("WARN", *args, file=sys.stderr)

        def error(self, *args, **_kwargs) -> None:
            print("ERROR", *args, file=sys.stderr)

    class FakeNode:
        def __init__(self, _name: str) -> None:
            self.parameters: Dict[str, object] = {}
            self.publishers = []
            self.subscriptions = []
            self.logger = FakeLogger()

        def declare_parameter(self, name: str, default_value: object) -> None:
            self.parameters[name] = parameter_overrides.get(name, default_value)

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
        def imgmsg_to_cv2(self, msg, desired_encoding: str = "bgr8"):
            return msg.cv_image

        def cv2_to_imgmsg(self, cv_image, encoding: str = "bgr8"):
            return types.SimpleNamespace(cv_image=cv_image, encoding=encoding)

    cv_bridge.CvBridge = FakeCvBridge
    _install_module("cv_bridge", cv_bridge)

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.Image = type("FakeImage", (), {})
    _install_module("sensor_msgs", sensor_msgs)
    _install_module("sensor_msgs.msg", sensor_msgs_msg)

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")

    class FakeHeader:
        def __init__(self) -> None:
            self.stamp = types.SimpleNamespace(sec=0, nanosec=0)

    class FakeString:
        def __init__(self) -> None:
            self.data = ""

    std_msgs_msg.Header = FakeHeader
    std_msgs_msg.String = FakeString
    _install_module("std_msgs", std_msgs)
    _install_module("std_msgs.msg", std_msgs_msg)

    roi_msg = types.ModuleType("roi_lidar_corner.msg")

    class FakeCorner3DArray:
        def __init__(self) -> None:
            self.header = FakeHeader()
            self.corners = []

    class FakeCornerROI:
        pass

    class FakeObjectROI:
        def __init__(self) -> None:
            self.corner_rois = []

    class FakeObjectROIArray:
        def __init__(self) -> None:
            self.objects = []

    class FakeStructureROI:
        LEFT_POST = 0
        RIGHT_POST = 1
        TOP_BEAM = 2

        def __init__(self) -> None:
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
        def __init__(self) -> None:
            self.header = None
            self.object_id = 0
            self.class_id = 0
            self.conf = 0.0
            self.bbox_xyxy = []
            self.structures = []

    class FakeFrontFaceROIArray:
        def __init__(self) -> None:
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
    rotor_swarm_msgs_msg.FrontFaceCorners = type("FakeFrontFaceCorners", (), {})
    _install_module("rotor_swarm_msgs", rotor_swarm_msgs)
    _install_module("rotor_swarm_msgs.msg", rotor_swarm_msgs_msg)


def _load_roi_generator(package_root: Path, parameter_overrides: Mapping[str, object]):
    sys.path.insert(0, str(package_root))
    _install_fake_ros_modules(package_root, parameter_overrides)
    module_path = package_root / "roi_lidar_corner" / "roi_generator_node.py"
    spec = importlib.util.spec_from_file_location("roi_video_generator_under_test", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules["roi_video_generator_under_test"] = module
    spec.loader.exec_module(module)
    return module


def _line_metrics(structure) -> LineMetrics:
    u0 = float(structure.line_u0)
    v0 = float(structure.line_v0)
    u1 = float(structure.line_u1)
    v1 = float(structure.line_v1)
    return LineMetrics(
        mid_x=(u0 + u1) / 2.0,
        mid_y=(v0 + v1) / 2.0,
        length=float(((u1 - u0) ** 2 + (v1 - v0) ** 2) ** 0.5),
        u0=u0,
        v0=v0,
        u1=u1,
        v1=v1,
    )


def _object_metrics(obj) -> ObjectMetrics:
    lines = {
        STRUCTURE_NAMES.get(int(structure.structure_label), str(int(structure.structure_label))): _line_metrics(structure)
        for structure in obj.structures
    }
    return ObjectMetrics(
        conf=float(obj.conf),
        bbox_xyxy=tuple(float(value) for value in obj.bbox_xyxy),
        lines=lines,
    )


def evaluate_video(
    video_path: Path,
    package_root: Path,
    model_path: Path,
    names_path: Path,
    backend: str,
    use_gpu: bool,
    conf_threshold: float,
    iou_threshold: float,
    input_size: int,
    class_filter: str,
    output_video_path: Optional[Path] = None,
) -> Tuple[List[FrameResult], float]:
    import cv2

    module = _load_roi_generator(
        package_root=package_root,
        parameter_overrides={
            "publish_debug_image": False,
            "detector_use_gpu": bool(use_gpu),
            "detector_model_path": str(model_path),
            "detector_names_path": str(names_path),
            "detector_backend": backend,
            "detector_conf_threshold": float(conf_threshold),
            "detector_iou_threshold": float(iou_threshold),
            "detector_input_size": int(input_size),
            "detector_class_filter": class_filter,
        },
    )
    node = module.RoiGeneratorNode()
    node.debug_log_every_n_frames = 10**9

    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise RuntimeError(f"failed to open video: {video_path}")

    fps = capture.get(cv2.CAP_PROP_FPS)
    fps = float(fps) if fps and fps > 0.0 else 15.0
    results: List[FrameResult] = []
    frame_index = 0
    writer = None
    start = time.monotonic()
    try:
        while True:
            ok, image = capture.read()
            if not ok:
                break
            msg = types.SimpleNamespace(
                header=types.SimpleNamespace(
                    stamp=types.SimpleNamespace(
                        sec=int(frame_index // fps),
                        nanosec=int((frame_index % fps) * (1e9 / fps)),
                    )
                ),
                cv_image=image,
            )
            before = len(node.publisher.published)
            node.image_callback(msg)
            if len(node.publisher.published) != before + 1:
                raise RuntimeError(f"frame {frame_index}: ROI callback did not publish exactly one message")
            output = node.publisher.published[-1]
            result = FrameResult(
                frame=frame_index,
                objects=[_object_metrics(obj) for obj in output.objects],
            )
            results.append(result)
            if output_video_path is not None:
                if writer is None:
                    height, width = image.shape[:2]
                    writer = _open_video_writer(output_video_path, fps, width, height)
                writer.write(render_annotated_frame(image, result))
            frame_index += 1
    finally:
        if writer is not None:
            writer.release()
        capture.release()
    return results, time.monotonic() - start


def _default_package_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _default_output_path(video_path: Path) -> Path:
    return video_path.with_name(f"{video_path.stem}_roi_stability.csv")


def _default_video_output_path(video_path: Path) -> Path:
    return video_path.with_name(f"{video_path.stem}_roi_stability_annotated.mp4")


def _print_delta(name: str, value: object) -> None:
    if not isinstance(value, dict):
        return
    print(
        f"{name} mean={value['mean']:.2f},p95={value['p95']:.2f},max={value['max']:.2f}"
    )


def print_summary(summary: Mapping[str, object], csv_path: Path) -> None:
    print(f"csv={csv_path}")
    print(
        f"frames={summary['frames']} elapsed_sec={summary['elapsed_sec']:.3f} "
        f"fps={summary['throughput_fps']:.2f}"
    )
    print(
        f"detected={summary['detected_frames']} missed={summary['missed_frames']} "
        f"multi={summary['multi_object_frames']}"
    )
    if "conf_min" in summary:
        print(f"conf_min={summary['conf_min']:.4f} conf_mean={summary['conf_mean']:.4f}")
    for key in (
        "bbox_cx_delta_px",
        "bbox_cy_delta_px",
        "bbox_w_delta_px",
        "bbox_h_delta_px",
        "left_mx_delta_px",
        "left_my_delta_px",
        "left_len_delta_px",
        "right_mx_delta_px",
        "right_my_delta_px",
        "right_len_delta_px",
        "top_mx_delta_px",
        "top_my_delta_px",
        "top_len_delta_px",
    ):
        _print_delta(key, summary.get(key))
    missed = summary.get("missed_frame_indices", [])
    if missed:
        print("missed_frame_indices=" + ",".join(str(value) for value in list(missed)[:100]))
    multi = summary.get("multi_frame_indices", [])
    if multi:
        print("multi_frame_indices=" + ",".join(str(value) for value in list(multi)[:100]))


def build_arg_parser() -> argparse.ArgumentParser:
    package_root = _default_package_root()
    parser = argparse.ArgumentParser(
        description="Evaluate first-stage ROI generator stability on an offline video."
    )
    parser.add_argument("video", type=Path, help="Input video path.")
    parser.add_argument("--output-csv", type=Path, default=None, help="CSV output path.")
    parser.add_argument(
        "--output-video",
        type=Path,
        default=None,
        help="Optional annotated MP4 path for visual ROI review.",
    )
    parser.add_argument(
        "--auto-output-video",
        action="store_true",
        help="Write an annotated review video beside the input video.",
    )
    parser.add_argument("--package-root", type=Path, default=package_root)
    parser.add_argument("--model", type=Path, default=package_root / "models" / "best.pt")
    parser.add_argument("--names", type=Path, default=package_root / "models" / "detect.names")
    parser.add_argument("--backend", default="pt", choices=("pt", "ultralytics", "onnx"))
    parser.add_argument("--use-gpu", action="store_true", help="Use GPU for detector inference if available.")
    parser.add_argument("--conf-threshold", type=float, default=0.25)
    parser.add_argument("--iou-threshold", type=float, default=0.45)
    parser.add_argument("--input-size", type=int, default=640)
    parser.add_argument("--class-filter", default="[]")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    video_path = args.video.expanduser().resolve()
    output_csv = args.output_csv.expanduser().resolve() if args.output_csv else _default_output_path(video_path)
    output_video = None
    if args.output_video is not None:
        output_video = args.output_video.expanduser().resolve()
    elif args.auto_output_video:
        output_video = _default_video_output_path(video_path).resolve()
    results, elapsed_sec = evaluate_video(
        video_path=video_path,
        package_root=args.package_root.expanduser().resolve(),
        model_path=args.model.expanduser().resolve(),
        names_path=args.names.expanduser().resolve(),
        backend=args.backend,
        use_gpu=args.use_gpu,
        conf_threshold=args.conf_threshold,
        iou_threshold=args.iou_threshold,
        input_size=args.input_size,
        class_filter=args.class_filter,
        output_video_path=output_video,
    )
    write_results_csv(results, output_csv)
    print_summary(summarize_results(results, elapsed_sec), output_csv)
    if output_video is not None:
        print(f"video={output_video}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
