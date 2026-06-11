from pathlib import Path

import numpy as np

from roi_lidar_corner.offline_front_face_validation import (
    CameraIntrinsics,
    load_static_scene_case,
    rotate_uv_180,
    uv_depth_to_xyz_cam,
)


def test_load_static_scene_case_reads_rgb_and_npz_assets() -> None:
    workspace = Path(__file__).resolve().parents[4]
    image_path = workspace / "analysis_artifacts/exposure_from_nx/lidar_projection_exposure_20260423_220029_0p5s.png"
    npz_path = workspace / "analysis_artifacts/exposure_from_nx/lidar_projection_exposure_20260423_221950_20s_points.npz"

    case = load_static_scene_case(image_path=image_path, npz_path=npz_path)

    assert case.image.shape[0] > 0
    assert case.uv.shape[1] == 2
    assert case.depth.shape[0] == case.uv.shape[0] == case.stamp.shape[0]


def test_rotate_uv_180_flips_projected_pixels_around_image_bounds() -> None:
    uv = np.array([[0.0, 0.0], [639.0, 479.0], [100.5, 200.25]], dtype=np.float32)

    rotated = rotate_uv_180(uv, width=640, height=480)

    np.testing.assert_allclose(
        rotated,
        np.array([[639.0, 479.0], [0.0, 0.0], [538.5, 278.75]], dtype=np.float32),
    )


def test_uv_depth_to_xyz_cam_uses_pinhole_intrinsics() -> None:
    intrinsics = CameraIntrinsics(width=640, height=480, fx=600.0, fy=500.0, cx=320.0, cy=240.0)
    uv = np.array([[320.0, 240.0], [620.0, 490.0]], dtype=np.float32)
    depth = np.array([2.0, 4.0], dtype=np.float32)

    xyz = uv_depth_to_xyz_cam(uv=uv, depth=depth, intrinsics=intrinsics)

    np.testing.assert_allclose(
        xyz,
        np.array([[0.0, 0.0, 2.0], [2.0, 2.0, 4.0]], dtype=np.float32),
    )


def test_load_static_scene_case_can_apply_rotation_and_intrinsics() -> None:
    workspace = Path(__file__).resolve().parents[4]
    image_path = workspace / "analysis_artifacts/exposure_from_nx/lidar_projection_exposure_20260423_220029_0p5s.png"
    npz_path = workspace / "analysis_artifacts/exposure_from_nx/lidar_projection_exposure_20260423_221950_20s_points.npz"
    intrinsics = CameraIntrinsics(
        width=640,
        height=480,
        fx=603.7161865234375,
        fy=603.680908203125,
        cx=317.5208740234375,
        cy=248.23284912109375,
    )

    case = load_static_scene_case(
        image_path=image_path,
        npz_path=npz_path,
        intrinsics=intrinsics,
        rotate_180=True,
    )

    assert case.uv_aligned.shape == case.uv.shape
    assert case.xyz_cam is not None
    assert case.xyz_cam.shape == (case.uv.shape[0], 3)
    np.testing.assert_allclose(case.uv_aligned[0], rotate_uv_180(case.uv[:1], width=640, height=480)[0])
