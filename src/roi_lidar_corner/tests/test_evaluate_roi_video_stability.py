from __future__ import annotations

import csv
from pathlib import Path
import sys

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from roi_lidar_corner.evaluate_roi_video_stability import (
    FrameResult,
    LineMetrics,
    ObjectMetrics,
    render_annotated_frame,
    summarize_results,
    write_results_csv,
)


def test_summarize_results_reports_detection_and_delta_metrics() -> None:
    results = [
        FrameResult(
            frame=0,
            objects=[
                ObjectMetrics(
                    conf=0.80,
                    bbox_xyxy=(10.0, 20.0, 110.0, 220.0),
                    lines={
                        "left": LineMetrics(mid_x=10.0, mid_y=120.0, length=200.0),
                        "top": LineMetrics(mid_x=60.0, mid_y=20.0, length=100.0),
                    },
                )
            ],
        ),
        FrameResult(
            frame=1,
            objects=[
                ObjectMetrics(
                    conf=0.90,
                    bbox_xyxy=(12.0, 24.0, 112.0, 226.0),
                    lines={
                        "left": LineMetrics(mid_x=12.0, mid_y=125.0, length=202.0),
                        "top": LineMetrics(mid_x=62.0, mid_y=24.0, length=100.0),
                    },
                )
            ],
        ),
        FrameResult(frame=2, objects=[]),
    ]

    summary = summarize_results(results, elapsed_sec=0.5)

    assert summary["frames"] == 3
    assert summary["detected_frames"] == 2
    assert summary["missed_frames"] == 1
    assert summary["multi_object_frames"] == 0
    assert summary["conf_min"] == pytest.approx(0.80)
    assert summary["conf_mean"] == pytest.approx(0.85)
    assert summary["bbox_cx_delta_px"]["mean"] == pytest.approx(2.0)
    assert summary["bbox_cy_delta_px"]["max"] == pytest.approx(5.0)
    assert summary["left_len_delta_px"]["mean"] == pytest.approx(2.0)
    assert summary["top_my_delta_px"]["max"] == pytest.approx(4.0)


def test_write_results_csv_includes_bbox_and_structure_columns(tmp_path: Path) -> None:
    output_path = tmp_path / "roi_stability.csv"
    results = [
        FrameResult(
            frame=4,
            objects=[
                ObjectMetrics(
                    conf=0.95,
                    bbox_xyxy=(100.0, 40.0, 300.0, 440.0),
                    sources={"left": "corner_refined", "right": "corner_refined", "top": "corner_refined"},
                    lines={
                        "left": LineMetrics(mid_x=105.0, mid_y=240.0, length=400.0),
                        "right": LineMetrics(mid_x=295.0, mid_y=240.0, length=400.0),
                        "top": LineMetrics(mid_x=200.0, mid_y=45.0, length=190.0),
                    },
                )
            ],
        )
    ]

    write_results_csv(results, output_path)

    rows = list(csv.DictReader(output_path.open(encoding="utf-8")))
    assert rows[0]["frame"] == "4"
    assert rows[0]["objects"] == "1"
    assert rows[0]["bbox_cx"] == "200.000000"
    assert rows[0]["left_source"] == "corner_refined"
    assert rows[0]["left_len"] == "400.000000"
    assert rows[0]["top_my"] == "45.000000"


def test_render_annotated_frame_draws_bbox_and_structure_lines() -> None:
    import numpy as np

    image = np.zeros((120, 160, 3), dtype=np.uint8)
    result = FrameResult(
        frame=7,
        objects=[
            ObjectMetrics(
                conf=0.95,
                bbox_xyxy=(20.0, 10.0, 120.0, 100.0),
                sources={"left": "corner_refined", "top": "corner_refined"},
                lines={
                    "left": LineMetrics(
                        mid_x=25.0,
                        mid_y=55.0,
                        length=90.0,
                        u0=25.0,
                        v0=10.0,
                        u1=25.0,
                        v1=100.0,
                    ),
                    "top": LineMetrics(
                        mid_x=70.0,
                        mid_y=12.0,
                        length=90.0,
                        u0=25.0,
                        v0=12.0,
                        u1=115.0,
                        v1=12.0,
                    ),
                },
            )
        ],
    )

    annotated = render_annotated_frame(image, result)

    assert annotated.shape == image.shape
    assert np.any(annotated != image)
    assert annotated[10, 20].any()
    assert annotated[55, 25].any()


def test_summarize_results_reports_source_counts_and_bbox_like_frames() -> None:
    results = [
        FrameResult(
            frame=0,
            objects=[
                ObjectMetrics(
                    conf=0.95,
                    bbox_xyxy=(20.0, 10.0, 120.0, 100.0),
                    sources={"left": "corner_refined", "right": "corner_refined", "top": "corner_refined"},
                    lines={
                        "left": LineMetrics(
                            mid_x=20.0,
                            mid_y=55.0,
                            length=90.0,
                            u0=20.0,
                            v0=10.0,
                            u1=20.0,
                            v1=100.0,
                        ),
                        "right": LineMetrics(
                            mid_x=120.0,
                            mid_y=55.0,
                            length=90.0,
                            u0=120.0,
                            v0=10.0,
                            u1=120.0,
                            v1=100.0,
                        ),
                        "top": LineMetrics(
                            mid_x=70.0,
                            mid_y=10.0,
                            length=100.0,
                            u0=20.0,
                            v0=10.0,
                            u1=120.0,
                            v1=10.0,
                        ),
                    },
                )
            ],
        ),
        FrameResult(
            frame=1,
            objects=[
                ObjectMetrics(
                    conf=0.95,
                    bbox_xyxy=(20.0, 10.0, 120.0, 100.0),
                    sources={
                        "left": "bbox_fallback:no_hough_lines",
                        "right": "bbox_fallback:no_hough_lines",
                        "top": "bbox_fallback:no_hough_lines",
                    },
                    lines={
                        "left": LineMetrics(
                            mid_x=35.0,
                            mid_y=55.0,
                            length=80.0,
                            u0=35.0,
                            v0=18.0,
                            u1=35.0,
                            v1=92.0,
                        ),
                        "right": LineMetrics(
                            mid_x=105.0,
                            mid_y=55.0,
                            length=80.0,
                            u0=105.0,
                            v0=18.0,
                            u1=105.0,
                            v1=92.0,
                        ),
                        "top": LineMetrics(
                            mid_x=70.0,
                            mid_y=22.0,
                            length=70.0,
                            u0=35.0,
                            v0=22.0,
                            u1=105.0,
                            v1=22.0,
                        ),
                    },
                )
            ],
        ),
    ]

    summary = summarize_results(results, elapsed_sec=0.5)

    assert summary["source_counts"] == {"corner_refined": 3, "bbox_fallback:no_hough_lines": 3}
    assert summary["bbox_like_frames"] == 1
    assert summary["bbox_like_frame_indices"] == [0]
