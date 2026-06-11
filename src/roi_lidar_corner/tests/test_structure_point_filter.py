import numpy as np

from roi_lidar_corner.structure_point_filter import filter_structure_points


def test_filter_structure_points_keeps_front_peak_and_rejects_rear_bar() -> None:
    uv = np.array([[10.0, 10.0], [11.0, 10.0], [12.0, 10.0], [10.5, 10.5]], dtype=np.float32)
    xyz = np.array(
        [
            [0.1, 0.2, 3.00],
            [0.1, 0.2, 3.05],
            [0.1, 0.2, 4.02],
            [0.1, 0.2, 4.05],
        ],
        dtype=np.float32,
    )

    result = filter_structure_points(
        uv=uv,
        xyz_cam=xyz,
        front_bin_size=0.05,
        front_keep_tolerance=0.08,
        rear_gap_m=1.0,
        rear_gap_tolerance=0.12,
    )

    np.testing.assert_allclose(result.xyz_cam[:, 2], np.array([3.00, 3.05], dtype=np.float32))
    assert result.points_raw == 4
    assert result.points_kept == 2
    assert result.points_rear_rejected == 2
