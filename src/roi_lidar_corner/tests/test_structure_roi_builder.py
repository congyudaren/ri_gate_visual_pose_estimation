from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from roi_lidar_corner.structure_roi_builder import (
    build_structure_lines,
    dilate_line_mask,
)


def test_build_structure_lines_uses_image_top_edge_for_normal_semantics() -> None:
    corners = {
        "TL": (10.0, 20.0),
        "TR": (30.0, 20.0),
        "BL": (10.0, 60.0),
        "BR": (30.0, 60.0),
    }

    lines = build_structure_lines(corners, structure_semantics="normal")

    assert lines["top_beam"] == ((10.0, 20.0), (30.0, 20.0))
    assert lines["left_post"] == ((10.0, 20.0), (10.0, 60.0))
    assert lines["right_post"] == ((30.0, 20.0), (30.0, 60.0))


def test_build_structure_lines_uses_image_bottom_edge_for_inverted_camera_top_beam() -> None:
    corners = {
        "TL": (10.0, 20.0),
        "TR": (30.0, 20.0),
        "BL": (10.0, 60.0),
        "BR": (30.0, 60.0),
    }

    lines = build_structure_lines(corners, structure_semantics="inverted_camera")

    assert lines["top_beam"] == ((10.0, 60.0), (30.0, 60.0))
    assert lines["left_post"] == ((30.0, 20.0), (30.0, 60.0))
    assert lines["right_post"] == ((10.0, 20.0), (10.0, 60.0))


def test_build_structure_lines_rejects_unknown_semantics() -> None:
    corners = {
        "TL": (10.0, 20.0),
        "TR": (30.0, 20.0),
        "BL": (10.0, 60.0),
        "BR": (30.0, 60.0),
    }

    try:
        build_structure_lines(corners, structure_semantics="rotated_sideways")
    except ValueError as exc:
        assert "structure_semantics" in str(exc)
    else:
        raise AssertionError("unknown structure semantics should be rejected")


def test_dilate_line_mask_marks_pixels_around_vertical_line() -> None:
    mask = dilate_line_mask(
        image_shape=(80, 80),
        start=(10.0, 10.0),
        end=(10.0, 50.0),
        half_width=2,
    )

    assert mask.shape == (80, 80)
    assert int(mask[30, 10]) == 255
    assert int(mask[30, 12]) == 255
    assert int(mask[30, 16]) == 0
