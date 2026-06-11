from __future__ import annotations

from collections import deque
from pathlib import Path
import sys

import cv2
import numpy as np
import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))


import roi_lidar_corner.lidar_projection_exposure_capture as exposure_capture  # noqa: E402
from roi_lidar_corner.lidar_projection_exposure_capture import (  # noqa: E402
    LidarProjectionExposureCaptureNode,
    ProjectionExposureBuffer,
    _BufferedPose,
    project_points_to_image,
    render_projection_exposure_capture,
)


def test_projection_exposure_buffer_trims_to_window() -> None:
    buffer = ProjectionExposureBuffer(window_sec=2.0)

    buffer.add_points(
        stamp=10.0,
        uv=np.asarray([[1, 2], [3, 4]], dtype=np.float32),
        depth=np.asarray([5.0, 6.0], dtype=np.float32),
    )
    buffer.add_points(
        stamp=12.5,
        uv=np.asarray([[7, 8]], dtype=np.float32),
        depth=np.asarray([9.0], dtype=np.float32),
    )
    buffer.trim(newest_stamp=12.5)

    assert buffer.points_uv.tolist() == [[7.0, 8.0]]
    assert buffer.depth.tolist() == [9.0]
    assert buffer.stamps.tolist() == [12.5]


def test_projection_exposure_buffer_rejects_mismatched_rows() -> None:
    buffer = ProjectionExposureBuffer(window_sec=1.0)

    with pytest.raises(ValueError, match="matching row counts"):
        buffer.add_points(
            stamp=1.0,
            uv=np.asarray([[1.0, 2.0], [3.0, 4.0]], dtype=np.float32),
            depth=np.asarray([5.0], dtype=np.float32),
        )


def test_project_points_to_image_drops_points_behind_camera_and_outside_image() -> None:
    uv, depth = project_points_to_image(
        points=np.asarray(
            [
                [0.0, 0.0, 5.0],
                [10.0, 0.0, 5.0],
                [0.0, 0.0, -1.0],
                [1.0, 1.0, 2.0],
            ],
            dtype=np.float32,
        ),
        stamp=10.0,
        pose={"t": np.zeros(3, dtype=np.float32), "R": np.eye(3, dtype=np.float32)},
        fx=10.0,
        fy=10.0,
        cx=5.0,
        cy=5.0,
        image_width=20,
        image_height=20,
        t_cb=np.zeros(3, dtype=np.float32),
        R_cb=np.eye(3, dtype=np.float32),
        cloud_frame_mode="base",
        min_range=0.0,
        max_range=100.0,
        cache_voxel_size=0.0,
    )

    np.testing.assert_allclose(uv, np.asarray([[5.0, 5.0], [10.0, 10.0]], dtype=np.float32))
    np.testing.assert_allclose(depth, np.asarray([5.0, 2.0], dtype=np.float32))


def test_render_projection_exposure_capture_writes_empty_overlay_and_metadata(tmp_path: Path) -> None:
    buffer = ProjectionExposureBuffer(window_sec=20.0)
    output_image = tmp_path / "exposure.png"
    output_json = tmp_path / "exposure.json"

    metadata = render_projection_exposure_capture(
        latest_image=None,
        camera_width=32,
        camera_height=24,
        buffer=buffer,
        duration_sec=20.0,
        window_sec=20.0,
        output_image=str(output_image),
        output_json=str(output_json),
        output_points_npz=None,
        image_topic="/camera/color/image_raw",
        pointcloud_topic="/cloud_registered",
        odom_topic="/Odometry",
        camera_info_topic="/camera/color/camera_info",
    )

    written = cv2.imread(str(output_image), cv2.IMREAD_COLOR)

    assert metadata["point_count"] == 0
    assert metadata["duration_sec"] == 20.0
    assert metadata["window_sec"] == 20.0
    assert metadata["image_topic"] == "/camera/color/image_raw"
    assert metadata["pointcloud_topic"] == "/cloud_registered"
    assert metadata["odom_topic"] == "/Odometry"
    assert metadata["camera_info_topic"] == "/camera/color/camera_info"
    assert output_json.read_text(encoding="utf-8")
    assert written.shape == (24, 32, 3)
    assert np.count_nonzero(written) == 0


def test_render_projection_exposure_capture_handles_nan_depth(tmp_path: Path) -> None:
    buffer = ProjectionExposureBuffer(window_sec=20.0)
    buffer.add_points(
        stamp=10.0,
        uv=np.asarray([[3.0, 4.0]], dtype=np.float32),
        depth=np.asarray([np.nan], dtype=np.float32),
    )
    output_image = tmp_path / "nan_depth.png"
    output_json = tmp_path / "nan_depth.json"

    metadata = render_projection_exposure_capture(
        latest_image=None,
        camera_width=16,
        camera_height=12,
        buffer=buffer,
        duration_sec=20.0,
        window_sec=20.0,
        output_image=str(output_image),
        output_json=str(output_json),
        output_points_npz=None,
        image_topic="/camera/color/image_raw",
        pointcloud_topic="/cloud_registered",
        odom_topic="/Odometry",
        camera_info_topic="/camera/color/camera_info",
    )

    assert metadata["point_count"] == 1
    assert output_image.exists()
    assert output_json.exists()


def test_render_projection_exposure_capture_writes_points_npz(tmp_path: Path) -> None:
    buffer = ProjectionExposureBuffer(window_sec=20.0)
    buffer.add_points(
        stamp=10.5,
        uv=np.asarray([[3.0, 4.0], [5.0, 6.0]], dtype=np.float32),
        depth=np.asarray([7.0, 8.0], dtype=np.float32),
    )
    output_image = tmp_path / "exposure.png"
    output_json = tmp_path / "exposure.json"
    output_points_npz = tmp_path / "points.npz"

    metadata = render_projection_exposure_capture(
        latest_image=None,
        camera_width=16,
        camera_height=12,
        buffer=buffer,
        duration_sec=20.0,
        window_sec=20.0,
        output_image=str(output_image),
        output_json=str(output_json),
        output_points_npz=str(output_points_npz),
        image_topic="/camera/color/image_raw",
        pointcloud_topic="/cloud_registered",
        odom_topic="/Odometry",
        camera_info_topic="/camera/color/camera_info",
    )
    points = np.load(output_points_npz)

    assert metadata["points_npz"] == str(output_points_npz)
    assert metadata["point_fields"] == ["uv", "depth", "stamp"]
    np.testing.assert_allclose(points["uv"], np.asarray([[3.0, 4.0], [5.0, 6.0]], dtype=np.float32))
    np.testing.assert_allclose(points["depth"], np.asarray([7.0, 8.0], dtype=np.float32))
    np.testing.assert_allclose(points["stamp"], np.asarray([10.5, 10.5], dtype=np.float64))


def test_nearest_odom_rejects_stale_pose_and_accepts_close_pose() -> None:
    node = object.__new__(LidarProjectionExposureCaptureNode)
    node.max_time_diff_odom = 0.5
    close_pose = _BufferedPose(stamp=9.7, t=np.zeros(3, dtype=np.float32), R=np.eye(3, dtype=np.float32))
    stale_pose = _BufferedPose(stamp=20.0, t=np.ones(3, dtype=np.float32), R=np.eye(3, dtype=np.float32))
    node.odom_buffer = deque([close_pose, stale_pose])

    assert node._nearest_odom(10.0) is close_pose
    assert node._nearest_odom(30.0) is None


def test_render_projection_exposure_capture_raises_when_image_write_fails(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    buffer = ProjectionExposureBuffer(window_sec=20.0)

    monkeypatch.setattr(exposure_capture.cv2, "imwrite", lambda *_args, **_kwargs: False)

    with pytest.raises(RuntimeError, match="Failed to write exposure image"):
        render_projection_exposure_capture(
            latest_image=None,
            camera_width=16,
            camera_height=12,
            buffer=buffer,
            duration_sec=20.0,
            window_sec=20.0,
            output_image=str(tmp_path / "failed.png"),
            output_json=str(tmp_path / "failed.json"),
            output_points_npz=None,
            image_topic="/camera/color/image_raw",
            pointcloud_topic="/cloud_registered",
            odom_topic="/Odometry",
            camera_info_topic="/camera/color/camera_info",
        )
