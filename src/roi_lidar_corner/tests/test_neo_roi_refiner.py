from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from roi_lidar_corner import neo_roi_refiner


def _image() -> np.ndarray:
    return np.zeros((120, 160, 3), dtype=np.uint8)


def test_hough_failure_returns_bbox_fallback_not_corner_refined(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(neo_roi_refiner.cv2, "HoughLinesP", lambda *_args, **_kwargs: None)

    result = neo_roi_refiner.refine_corners_inside_bbox_with_source(
        _image(),
        (20.0, 10.0, 120.0, 100.0),
    )

    assert result.source == "bbox_fallback"
    assert result.reason == "no_hough_lines"
    assert result.corners == [
        (20.0, 10.0),
        (120.0, 10.0),
        (20.0, 100.0),
        (120.0, 100.0),
    ]


def test_missing_horizontal_lines_returns_specific_reason(monkeypatch: pytest.MonkeyPatch) -> None:
    lines = np.array(
        [
            [[14, 12, 14, 76]],
            [[86, 12, 86, 76]],
        ],
        dtype=np.int32,
    )
    monkeypatch.setattr(neo_roi_refiner.cv2, "HoughLinesP", lambda *_args, **_kwargs: lines)

    result = neo_roi_refiner.refine_corners_inside_bbox_with_source(
        _image(),
        (20.0, 10.0, 120.0, 100.0),
    )

    assert result.source == "bbox_fallback"
    assert result.reason == "missing_horizontal"


def test_missing_vertical_lines_returns_specific_reason(monkeypatch: pytest.MonkeyPatch) -> None:
    lines = np.array(
        [
            [[14, 12, 86, 12]],
            [[14, 76, 86, 76]],
        ],
        dtype=np.int32,
    )
    monkeypatch.setattr(neo_roi_refiner.cv2, "HoughLinesP", lambda *_args, **_kwargs: lines)

    result = neo_roi_refiner.refine_corners_inside_bbox_with_source(
        _image(),
        (20.0, 10.0, 120.0, 100.0),
    )

    assert result.source == "bbox_fallback"
    assert result.reason == "missing_vertical"


def test_bbox_border_like_refined_lines_are_demoted(monkeypatch: pytest.MonkeyPatch) -> None:
    lines = np.array(
        [
            [[0, 0, 99, 0]],
            [[0, 89, 99, 89]],
            [[0, 0, 0, 89]],
            [[99, 0, 99, 89]],
        ],
        dtype=np.int32,
    )
    monkeypatch.setattr(neo_roi_refiner.cv2, "HoughLinesP", lambda *_args, **_kwargs: lines)

    result = neo_roi_refiner.refine_corners_inside_bbox_with_source(
        _image(),
        (20.0, 10.0, 120.0, 100.0),
        bbox_border_tolerance_px=2.0,
    )

    assert result.source == "bbox_fallback"
    assert result.reason == "bbox_border_like"
    assert result.corners == [
        (20.0, 10.0),
        (120.0, 10.0),
        (20.0, 100.0),
        (120.0, 100.0),
    ]


def test_valid_interior_supported_structure_lines_remain_corner_refined(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    lines = np.array(
        [
            [[14, 12, 86, 12]],
            [[14, 76, 86, 76]],
            [[14, 12, 14, 76]],
            [[86, 12, 86, 76]],
        ],
        dtype=np.int32,
    )
    monkeypatch.setattr(neo_roi_refiner.cv2, "HoughLinesP", lambda *_args, **_kwargs: lines)

    result = neo_roi_refiner.refine_corners_inside_bbox_with_source(
        _image(),
        (20.0, 10.0, 120.0, 100.0),
        bbox_border_tolerance_px=2.0,
    )

    assert result.source == "corner_refined"
    assert result.reason == "ok"
    assert result.corners == [
        (34.0, 22.0),
        (106.0, 22.0),
        (34.0, 86.0),
        (106.0, 86.0),
    ]


def test_adjacent_top_horizontals_are_not_reported_as_refined_posts(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    lines = np.array(
        [
            [[14, 12, 86, 12]],
            [[14, 18, 86, 18]],
            [[14, 12, 14, 76]],
            [[86, 12, 86, 76]],
        ],
        dtype=np.int32,
    )
    monkeypatch.setattr(neo_roi_refiner.cv2, "HoughLinesP", lambda *_args, **_kwargs: lines)

    result = neo_roi_refiner.refine_corners_inside_bbox_with_source(
        _image(),
        (20.0, 10.0, 120.0, 100.0),
        bbox_border_tolerance_px=2.0,
    )

    assert result.source == "bbox_fallback"
    assert result.reason == "collapsed_horizontal"
    assert result.corners == [
        (20.0, 10.0),
        (120.0, 10.0),
        (20.0, 100.0),
        (120.0, 100.0),
    ]


def test_vertical_selection_prefers_long_overlapping_posts_over_extreme_short_segment(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    lines = np.array(
        [
            [[14, 12, 86, 12]],
            [[14, 76, 86, 76]],
            [[5, 12, 5, 42]],
            [[14, 12, 14, 76]],
            [[86, 12, 86, 76]],
        ],
        dtype=np.int32,
    )
    monkeypatch.setattr(neo_roi_refiner.cv2, "HoughLinesP", lambda *_args, **_kwargs: lines)

    result = neo_roi_refiner.refine_corners_inside_bbox_with_source(
        _image(),
        (20.0, 10.0, 120.0, 100.0),
        bbox_border_tolerance_px=2.0,
    )

    assert result.source == "corner_refined"
    assert result.reason == "ok"
    assert result.corners == [
        (34.0, 22.0),
        (106.0, 22.0),
        (34.0, 86.0),
        (106.0, 86.0),
    ]


def test_vertical_selection_prefers_outer_side_post_when_it_has_enough_coverage(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    lines = np.array(
        [
            [[5, 12, 86, 12]],
            [[5, 76, 86, 76]],
            [[5, 12, 5, 57]],
            [[14, 12, 14, 76]],
            [[86, 12, 86, 76]],
        ],
        dtype=np.int32,
    )
    monkeypatch.setattr(neo_roi_refiner.cv2, "HoughLinesP", lambda *_args, **_kwargs: lines)

    result = neo_roi_refiner.refine_corners_inside_bbox_with_source(
        _image(),
        (20.0, 10.0, 120.0, 100.0),
        bbox_border_tolerance_px=2.0,
    )

    assert result.source == "corner_refined"
    assert result.reason == "ok"
    assert result.corners == [
        (25.0, 22.0),
        (106.0, 22.0),
        (25.0, 86.0),
        (106.0, 86.0),
    ]
