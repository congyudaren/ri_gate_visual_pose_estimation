from __future__ import annotations

from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from roi_lidar_corner.roi_geometry_prior import (
    GeometryPriorConfig,
    StructureCandidate,
    TemporalPriorConfig,
    build_bbox_corners,
    line_length,
    line_midpoint,
    temporal_jump_reason,
    validate_structure_candidate,
)


def candidate(left, right, top):
    return StructureCandidate(lines={"left_post": left, "right_post": right, "top_beam": top})


def test_expected_one_to_two_gate_ratio_is_accepted() -> None:
    config = GeometryPriorConfig()
    result = validate_structure_candidate(
        candidate(
            left=((100.0, 80.0), (100.0, 280.0)),
            right=((200.0, 80.0), (200.0, 280.0)),
            top=((100.0, 80.0), (200.0, 80.0)),
        ),
        bbox=(90.0, 70.0, 210.0, 290.0),
        image_shape=(480, 640),
        config=config,
    )

    assert result.valid is True
    assert result.reason == "accepted"


def test_collapsed_post_is_rejected() -> None:
    result = validate_structure_candidate(
        candidate(
            left=((100.0, 80.0), (100.0, 84.0)),
            right=((200.0, 80.0), (200.0, 280.0)),
            top=((100.0, 80.0), (200.0, 80.0)),
        ),
        bbox=(90.0, 70.0, 210.0, 290.0),
        image_shape=(480, 640),
        config=GeometryPriorConfig(),
    )

    assert result.valid is False
    assert result.reason == "post_too_short"


def test_top_bottom_flip_is_rejected() -> None:
    result = validate_structure_candidate(
        candidate(
            left=((100.0, 80.0), (100.0, 280.0)),
            right=((200.0, 80.0), (200.0, 280.0)),
            top=((100.0, 270.0), (200.0, 270.0)),
        ),
        bbox=(90.0, 70.0, 210.0, 290.0),
        image_shape=(480, 640),
        config=GeometryPriorConfig(),
    )

    assert result.valid is False
    assert result.reason == "top_not_near_bbox_top"


def test_inverted_camera_top_beam_near_bbox_bottom_is_accepted() -> None:
    result = validate_structure_candidate(
        candidate(
            left=((200.0, 80.0), (200.0, 280.0)),
            right=((100.0, 80.0), (100.0, 280.0)),
            top=((100.0, 280.0), (200.0, 280.0)),
        ),
        bbox=(90.0, 70.0, 210.0, 290.0),
        image_shape=(480, 640),
        config=GeometryPriorConfig(top_edge_reference="bbox_bottom"),
    )

    assert result.valid is True
    assert result.reason == "accepted"


def test_border_clipped_bbox_relaxes_ratio_check() -> None:
    result = validate_structure_candidate(
        candidate(
            left=((0.0, 40.0), (0.0, 240.0)),
            right=((170.0, 40.0), (170.0, 240.0)),
            top=((0.0, 40.0), (170.0, 40.0)),
        ),
        bbox=(0.0, 30.0, 180.0, 250.0),
        image_shape=(480, 640),
        config=GeometryPriorConfig(top_post_ratio_tolerance=0.20, border_relax_px=8.0),
    )

    assert result.valid is True


def test_extreme_border_clipped_ratio_is_still_rejected() -> None:
    result = validate_structure_candidate(
        candidate(
            left=((0.0, 40.0), (0.0, 240.0)),
            right=((500.0, 40.0), (500.0, 240.0)),
            top=((0.0, 40.0), (500.0, 40.0)),
        ),
        bbox=(0.0, 30.0, 510.0, 250.0),
        image_shape=(480, 640),
        config=GeometryPriorConfig(
            expected_top_post_ratio=0.5,
            top_post_ratio_tolerance=0.20,
            border_relax_px=8.0,
        ),
    )

    assert result.valid is False
    assert result.reason == "top_post_ratio"


def test_side_border_top_bottom_flip_is_rejected() -> None:
    result = validate_structure_candidate(
        candidate(
            left=((0.0, 80.0), (0.0, 280.0)),
            right=((100.0, 80.0), (100.0, 280.0)),
            top=((0.0, 270.0), (100.0, 270.0)),
        ),
        bbox=(0.0, 70.0, 110.0, 290.0),
        image_shape=(480, 640),
        config=GeometryPriorConfig(border_relax_px=8.0),
    )

    assert result.valid is False
    assert result.reason == "top_not_near_bbox_top"


def test_top_border_top_bottom_flip_is_rejected() -> None:
    result = validate_structure_candidate(
        candidate(
            left=((100.0, 80.0), (100.0, 280.0)),
            right=((200.0, 80.0), (200.0, 280.0)),
            top=((100.0, 270.0), (200.0, 270.0)),
        ),
        bbox=(90.0, 0.0, 210.0, 290.0),
        image_shape=(480, 640),
        config=GeometryPriorConfig(border_relax_px=8.0),
    )

    assert result.valid is False
    assert result.reason == "top_not_near_bbox_top"


def test_side_border_short_post_is_rejected_without_vertical_relaxation() -> None:
    result = validate_structure_candidate(
        candidate(
            left=((0.0, 40.0), (0.0, 95.0)),
            right=((30.0, 40.0), (30.0, 95.0)),
            top=((0.0, 40.0), (30.0, 40.0)),
        ),
        bbox=(0.0, 30.0, 80.0, 250.0),
        image_shape=(480, 640),
        config=GeometryPriorConfig(border_relax_px=8.0),
    )

    assert result.valid is False
    assert result.reason == "post_too_short"


def test_border_clipped_tiny_post_is_still_rejected() -> None:
    result = validate_structure_candidate(
        candidate(
            left=((0.0, 40.0), (0.0, 60.0)),
            right=((10.0, 40.0), (10.0, 60.0)),
            top=((0.0, 40.0), (10.0, 40.0)),
        ),
        bbox=(0.0, 30.0, 20.0, 250.0),
        image_shape=(480, 640),
        config=GeometryPriorConfig(border_relax_px=8.0),
    )

    assert result.valid is False
    assert result.reason == "post_too_short"


def test_bbox_corners_preserve_bbox_order() -> None:
    assert build_bbox_corners((10.0, 20.0, 110.0, 220.0)) == [
        (10.0, 20.0),
        (110.0, 20.0),
        (10.0, 220.0),
        (110.0, 220.0),
    ]


def test_line_length_and_midpoint_return_expected_values() -> None:
    line = ((1.0, 2.0), (4.0, 6.0))

    assert line_length(line) == 5.0
    assert line_midpoint(line) == (2.5, 4.0)


def test_temporal_jump_reason_accepts_small_movement() -> None:
    previous = candidate(
        left=((0.0, 0.0), (0.0, 100.0)),
        right=((100.0, 0.0), (100.0, 100.0)),
        top=((0.0, 0.0), (100.0, 0.0)),
    )
    current = candidate(
        left=((10.0, 0.0), (10.0, 100.0)),
        right=((110.0, 0.0), (110.0, 100.0)),
        top=((10.0, 0.0), (110.0, 0.0)),
    )

    assert temporal_jump_reason(
        previous,
        current,
        TemporalPriorConfig(max_line_jump_px=80.0),
    ) is None


def test_temporal_jump_reason_allows_missing_previous_line() -> None:
    previous = StructureCandidate(
        lines={
            "right_post": ((100.0, 0.0), (100.0, 100.0)),
        }
    )
    current = candidate(
        left=((100.0, 0.0), (100.0, 100.0)),
        right=((100.0, 0.0), (100.0, 100.0)),
        top=((0.0, 0.0), (100.0, 0.0)),
    )

    assert temporal_jump_reason(
        previous,
        current,
        TemporalPriorConfig(max_line_jump_px=80.0),
    ) is None


def test_temporal_jump_reason_ignores_missing_previous_candidate() -> None:
    current = candidate(
        left=((100.0, 0.0), (100.0, 100.0)),
        right=((100.0, 0.0), (100.0, 100.0)),
        top=((0.0, 0.0), (100.0, 0.0)),
    )

    assert temporal_jump_reason(
        None,
        current,
        TemporalPriorConfig(max_line_jump_px=80.0),
    ) is None


def test_temporal_jump_reason_reports_large_line_jump() -> None:
    previous = candidate(
        left=((0.0, 0.0), (0.0, 100.0)),
        right=((100.0, 0.0), (100.0, 100.0)),
        top=((0.0, 0.0), (100.0, 0.0)),
    )
    current = candidate(
        left=((100.0, 0.0), (100.0, 100.0)),
        right=((100.0, 0.0), (100.0, 100.0)),
        top=((0.0, 0.0), (100.0, 0.0)),
    )

    assert temporal_jump_reason(
        previous,
        current,
        TemporalPriorConfig(max_line_jump_px=80.0),
    ) == "left_post_temporal_jump"


def test_temporal_jump_reason_reports_length_only_jump() -> None:
    previous = candidate(
        left=((0.0, 0.0), (0.0, 100.0)),
        right=((100.0, 0.0), (100.0, 100.0)),
        top=((0.0, 0.0), (100.0, 0.0)),
    )
    current = candidate(
        left=((0.0, -60.0), (0.0, 160.0)),
        right=((100.0, 0.0), (100.0, 100.0)),
        top=((0.0, 0.0), (100.0, 0.0)),
    )

    assert temporal_jump_reason(
        previous,
        current,
        TemporalPriorConfig(max_line_jump_px=80.0),
    ) == "left_post_temporal_jump"
