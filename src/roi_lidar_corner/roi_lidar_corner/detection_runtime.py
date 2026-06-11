from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np

try:
    import onnxruntime as ort
except Exception:  # pragma: no cover - optional dependency
    ort = None

try:
    from ultralytics import YOLO
except Exception:  # pragma: no cover - optional dependency
    YOLO = None


@dataclass(frozen=True)
class Detection:
    bbox: Tuple[float, float, float, float]
    conf: float
    class_id: int
    class_name: str = ""


@dataclass(frozen=True)
class DetectionFrame:
    detections: Tuple[Detection, ...]


def _normalize_indices(indices) -> List[int]:
    if indices is None:
        return []
    if isinstance(indices, tuple):
        indices = list(indices)
    if hasattr(indices, "flatten"):
        return [int(v) for v in indices.flatten().tolist()]
    return [int(v[0] if isinstance(v, (list, tuple)) else v) for v in indices]


def load_class_names(path: str) -> List[str]:
    file_path = Path(path).expanduser()
    if not file_path.is_file():
        return []
    names: List[str] = []
    for line in file_path.read_text(encoding="utf-8").splitlines():
        text = line.strip()
        if text:
            names.append(text)
    return names


def resolve_resource_path(candidate: str, script_path: Path, defaults: Iterable[str]) -> str:
    if candidate:
        value = Path(candidate).expanduser()
        if value.is_file():
            return str(value)

    bases = [
        script_path.parent.parent,
        script_path.parents[2],
    ]
    for base in bases:
        for relative_path in defaults:
            fallback = base / relative_path
            if fallback.is_file():
                return str(fallback)
    return ""


class OnnxYoloDetector:
    def __init__(
        self,
        model_path: str,
        class_names_path: str,
        conf_threshold: float,
        iou_threshold: float,
        input_size: int,
        use_gpu: bool,
        class_filter: Sequence[int],
        logger,
    ) -> None:
        self.logger = logger
        self.conf_threshold = float(conf_threshold)
        self.iou_threshold = float(iou_threshold)
        self.input_size = max(32, int(input_size))
        self.class_filter = set(int(v) for v in class_filter)
        self.class_names = load_class_names(class_names_path)
        self.available = False

        if ort is None:
            self.logger.error("onnxruntime is not available, onnx detector disabled")
            return

        provider_names = ort.get_available_providers()
        providers = ["CPUExecutionProvider"]
        if use_gpu and "CUDAExecutionProvider" in provider_names:
            providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
        elif use_gpu:
            self.logger.warn("CUDAExecutionProvider unavailable, onnx detector falls back to CPU")

        try:
            session_options = ort.SessionOptions()
            self.session = ort.InferenceSession(model_path, sess_options=session_options, providers=providers)
        except Exception as exc:  # pragma: no cover - runtime issue
            self.logger.error(f"load onnx detector failed: {exc}")
            return

        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [item.name for item in self.session.get_outputs()]
        input_shape = self.session.get_inputs()[0].shape
        self.input_height = int(input_shape[2]) if isinstance(input_shape[2], int) and input_shape[2] > 0 else self.input_size
        self.input_width = int(input_shape[3]) if isinstance(input_shape[3], int) and input_shape[3] > 0 else self.input_size
        self.mask_dim = 32 if len(self.output_names) > 1 else 0
        self.available = True

    def detect(self, image: np.ndarray) -> DetectionFrame:
        if not self.available:
            return DetectionFrame(detections=())

        blob, gain, pad = self._preprocess(image)
        outputs = self.session.run(self.output_names, {self.input_name: blob})
        detections = self._postprocess(outputs[0], image.shape[:2], gain, pad)
        return DetectionFrame(detections=tuple(detections))

    def _preprocess(self, image: np.ndarray) -> Tuple[np.ndarray, float, Tuple[float, float]]:
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        resized, gain, pad = self._letterbox(rgb, (self.input_width, self.input_height))
        normalized = resized.astype(np.float32) / 255.0
        blob = np.transpose(normalized, (2, 0, 1))[None, ...]
        return np.ascontiguousarray(blob), gain, pad

    def _letterbox(self, image: np.ndarray, new_shape: Tuple[int, int]) -> Tuple[np.ndarray, float, Tuple[float, float]]:
        shape = image.shape[:2]
        gain = min(new_shape[1] / shape[0], new_shape[0] / shape[1])
        new_unpad = (int(round(shape[1] * gain)), int(round(shape[0] * gain)))
        dw = (new_shape[0] - new_unpad[0]) / 2.0
        dh = (new_shape[1] - new_unpad[1]) / 2.0

        if shape[::-1] != new_unpad:
            image = cv2.resize(image, new_unpad, interpolation=cv2.INTER_LINEAR)

        top = int(round(dh - 0.1))
        bottom = int(round(dh + 0.1))
        left = int(round(dw - 0.1))
        right = int(round(dw + 0.1))
        bordered = cv2.copyMakeBorder(
            image,
            top,
            bottom,
            left,
            right,
            cv2.BORDER_CONSTANT,
            value=(114, 114, 114),
        )
        return bordered, gain, (dw, dh)

    def _postprocess(
        self,
        output0: np.ndarray,
        image_shape: Tuple[int, int],
        gain: float,
        pad: Tuple[float, float],
    ) -> List[Detection]:
        predictions = output0[0]
        if predictions.ndim != 2:
            return []
        if predictions.shape[0] <= 256:
            predictions = predictions.T

        class_count = predictions.shape[1] - 4 - self.mask_dim
        if class_count <= 0:
            return []

        boxes_xywh: List[List[float]] = []
        scores: List[float] = []
        class_ids: List[int] = []

        for row in predictions:
            class_scores = row[4 : 4 + class_count]
            class_id = int(np.argmax(class_scores))
            if self.class_filter and class_id not in self.class_filter:
                continue
            score = float(class_scores[class_id])
            if score < self.conf_threshold:
                continue

            cx, cy, width, height = [float(v) for v in row[:4]]
            left = cx - width / 2.0
            top = cy - height / 2.0
            boxes_xywh.append([left, top, width, height])
            scores.append(score)
            class_ids.append(class_id)

        kept = _normalize_indices(
            cv2.dnn.NMSBoxes(boxes_xywh, scores, self.conf_threshold, self.iou_threshold, eta=1.0, top_k=0)
        )

        detections: List[Detection] = []
        pad_w, pad_h = pad
        image_h, image_w = image_shape
        for idx in kept:
            left, top, width, height = boxes_xywh[idx]
            x1 = max(0.0, min(image_w - 1.0, (left - pad_w) / gain))
            y1 = max(0.0, min(image_h - 1.0, (top - pad_h) / gain))
            x2 = max(0.0, min(image_w * 1.0, (left + width - pad_w) / gain))
            y2 = max(0.0, min(image_h * 1.0, (top + height - pad_h) / gain))
            if x2 <= x1 or y2 <= y1:
                continue

            class_id = class_ids[idx]
            class_name = self.class_names[class_id] if class_id < len(self.class_names) else ""
            detections.append(
                Detection(
                    bbox=(x1, y1, x2, y2),
                    conf=scores[idx],
                    class_id=class_id,
                    class_name=class_name,
                )
            )
        return detections


class UltralyticsDetector:
    def __init__(
        self,
        model_path: str,
        conf_threshold: float,
        input_size: int,
        class_filter: Sequence[int],
        device: str,
        logger,
    ) -> None:
        self.logger = logger
        self.conf_threshold = float(conf_threshold)
        self.input_size = max(32, int(input_size))
        self.class_filter = set(int(v) for v in class_filter)
        self.device = device or None
        self.available = False

        if YOLO is None:
            self.logger.error("ultralytics is not available, pt detector disabled")
            return

        try:
            self.model = YOLO(model_path)
        except Exception as exc:  # pragma: no cover - runtime issue
            self.logger.error(f"load ultralytics detector failed: {exc}")
            return

        self.available = True

    def detect(self, image: np.ndarray) -> DetectionFrame:
        if not self.available:
            return DetectionFrame(detections=())

        original_copy_make_border = cv2.copyMakeBorder

        def _safe_copy_make_border(src, top, bottom, left, right, border_type, *args, **kwargs):
            return original_copy_make_border(
                src,
                int(round(float(top))),
                int(round(float(bottom))),
                int(round(float(left))),
                int(round(float(right))),
                border_type,
                *args,
                **kwargs,
            )

        cv2.copyMakeBorder = _safe_copy_make_border
        try:
            results = self.model(
                image,
                conf=self.conf_threshold,
                imgsz=self.input_size,
                verbose=False,
                device=self.device,
            )
        finally:
            cv2.copyMakeBorder = original_copy_make_border
        detections: List[Detection] = []
        if not results:
            return DetectionFrame(detections=())

        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return DetectionFrame(detections=())

        for det in boxes:
            class_id = int(det.cls.item())
            if self.class_filter and class_id not in self.class_filter:
                continue
            x1, y1, x2, y2 = det.xyxy[0].cpu().numpy().tolist()
            detections.append(
                Detection(
                    bbox=(float(x1), float(y1), float(x2), float(y2)),
                    conf=float(det.conf.item()),
                    class_id=class_id,
                    class_name="",
                )
            )
        return DetectionFrame(detections=tuple(detections))


def create_detector(
    backend: str,
    detector_model_path: str,
    detector_names_path: str,
    detector_conf_threshold: float,
    detector_iou_threshold: float,
    detector_input_size: int,
    detector_use_gpu: bool,
    detector_class_filter: Sequence[int],
    ultralytics_device: str,
    logger,
) -> Optional[object]:
    normalized = (backend or "").strip().lower()
    if normalized == "onnx":
        return OnnxYoloDetector(
            model_path=detector_model_path,
            class_names_path=detector_names_path,
            conf_threshold=detector_conf_threshold,
            iou_threshold=detector_iou_threshold,
            input_size=detector_input_size,
            use_gpu=detector_use_gpu,
            class_filter=detector_class_filter,
            logger=logger,
        )
    if normalized in {"ultralytics", "pt"}:
        return UltralyticsDetector(
            model_path=detector_model_path,
            conf_threshold=detector_conf_threshold,
            input_size=detector_input_size,
            class_filter=detector_class_filter,
            device=ultralytics_device,
            logger=logger,
        )
    logger.error(f"unsupported detector backend: {backend}")
    return None
