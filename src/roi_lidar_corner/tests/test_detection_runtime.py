from __future__ import annotations

from pathlib import Path
import sys
import types

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from roi_lidar_corner import detection_runtime


def test_ultralytics_detector_disables_prediction_saving(monkeypatch) -> None:
    calls = []

    class FakeYolo:
        def __init__(self, model_path: str) -> None:
            self.model_path = model_path

        def __call__(self, *args, **kwargs):
            calls.append(kwargs)
            return [types.SimpleNamespace(boxes=None)]

    monkeypatch.setattr(detection_runtime, "YOLO", FakeYolo)
    detector = detection_runtime.UltralyticsDetector(
        model_path="model.pt",
        conf_threshold=0.25,
        input_size=640,
        class_filter=[],
        device="cpu",
        logger=types.SimpleNamespace(error=lambda *_args: None, warn=lambda *_args: None),
    )

    detector.detect(np.zeros((32, 32, 3), dtype=np.uint8))

    assert calls
    assert calls[0]["save"] is False
    assert str(calls[0]["project"]).startswith("/tmp/")
    assert calls[0]["name"] == "roi_lidar_corner_predict"
    assert calls[0]["exist_ok"] is True
