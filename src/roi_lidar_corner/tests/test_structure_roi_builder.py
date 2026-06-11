from roi_lidar_corner.structure_roi_builder import (
    build_structure_lines,
    dilate_line_mask,
)


def test_build_structure_lines_uses_physical_semantics_for_inverted_camera() -> None:
    corners = {
        "TL": (10.0, 20.0),
        "TR": (30.0, 20.0),
        "BL": (10.0, 60.0),
        "BR": (30.0, 60.0),
    }

    lines = build_structure_lines(corners)

    assert lines["top_beam"] == ((10.0, 60.0), (30.0, 60.0))
    assert lines["left_post"] == ((30.0, 20.0), (30.0, 60.0))
    assert lines["right_post"] == ((10.0, 20.0), (10.0, 60.0))


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
