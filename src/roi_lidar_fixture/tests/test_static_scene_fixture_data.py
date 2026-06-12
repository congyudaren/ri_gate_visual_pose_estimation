from pathlib import Path
import sys
from array import array

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from roi_lidar_fixture.static_scene_fixture_data import (
    CameraIntrinsics,
    FixtureAssets,
    camera_points_to_body,
    default_assets,
    load_fixture_assets,
    points_to_xyz32_bytes,
    uv_depth_to_xyz_cam,
)


def test_default_assets_point_to_existing_long_exposure_files() -> None:
    assets = default_assets()

    assert assets.image_path.exists()
    assert assets.points_path.exists()
    assert assets.image_path.name == "lidar_projection_exposure_20260423_220029_0p5s.png"
    assert assets.points_path.name == "lidar_projection_exposure_20260423_221950_20s_points.npz"


def test_load_fixture_assets_reads_image_and_projected_points() -> None:
    data = load_fixture_assets(default_assets())

    assert data.image_bgr.shape == (480, 640, 3)
    assert data.uv.shape == (65048, 2)
    assert data.depth.shape == (65048,)
    assert data.stamp.shape == (65048,)


def test_uv_depth_to_xyz_cam_uses_pinhole_projection() -> None:
    intrinsics = CameraIntrinsics(
        width=640,
        height=480,
        fx=600.0,
        fy=500.0,
        cx=320.0,
        cy=240.0,
    )
    uv = np.array([[320.0, 240.0], [620.0, 490.0]], dtype=np.float32)
    depth = np.array([2.0, 4.0], dtype=np.float32)

    xyz = uv_depth_to_xyz_cam(uv=uv, depth=depth, intrinsics=intrinsics)

    np.testing.assert_allclose(
        xyz,
        np.array([[0.0, 0.0, 2.0], [2.0, 2.0, 4.0]], dtype=np.float32),
    )


def test_camera_points_to_body_inverts_configured_body_to_camera_transform() -> None:
    xyz_cam = np.array([[0.0, 0.0, 2.0], [1.0, 2.0, 3.0]], dtype=np.float32)
    t_cb = np.array([0.049, 0.29671, 0.01812], dtype=np.float32)
    r_cb = np.array(
        [
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ],
        dtype=np.float32,
    )

    xyz_body = camera_points_to_body(xyz_cam=xyz_cam, t_cb=t_cb, r_cb=r_cb)
    round_trip = (xyz_body - t_cb.reshape(1, 3)) @ r_cb

    np.testing.assert_allclose(round_trip, xyz_cam, atol=1e-6)


def test_points_to_xyz32_bytes_drops_nonfinite_points_and_uses_xyz_layout() -> None:
    points = np.array(
        [
            [1.0, 2.0, 3.0],
            [np.nan, 2.0, 3.0],
            [4.0, 5.0, 6.0],
        ],
        dtype=np.float32,
    )

    payload, kept = points_to_xyz32_bytes(points)

    assert kept == 2
    assert len(payload) == kept * 12
    decoded = np.frombuffer(payload, dtype="<f4").reshape(-1, 3)
    np.testing.assert_allclose(decoded, np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32))


def test_points_to_xyz32_bytes_returns_ros_compatible_uint8_array() -> None:
    payload, kept = points_to_xyz32_bytes(np.array([[1.0, 2.0, 3.0]], dtype=np.float32))

    assert kept == 1
    assert isinstance(payload, array)
    assert payload.typecode == "B"
    assert len(payload) == 12


def test_load_fixture_assets_allows_explicit_paths(tmp_path: Path) -> None:
    source = default_assets()
    assets = FixtureAssets(image_path=source.image_path, points_path=source.points_path)

    data = load_fixture_assets(assets)

    assert data.image_bgr.shape[1] == 640
