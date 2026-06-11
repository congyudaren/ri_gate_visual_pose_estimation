from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import sys
from pathlib import Path

import numpy as np
import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

import roi_lidar_corner.lookback_solver as lookback_solver  # noqa: E402

DecodedCloudFrame = lookback_solver.DecodedCloudFrame
prepare_cached_points = lookback_solver.prepare_cached_points
select_lookback_frames = lookback_solver.select_lookback_frames
trim_cloud_cache = lookback_solver.trim_cloud_cache


@dataclass(frozen=True)
class CornerRoi:
    corner_label: int
    mask_origin_x: int
    mask_origin_y: int
    mask_width: int
    mask_height: int
    roi_mask: object
    valid: bool = True


@dataclass(frozen=True)
class RoiObject:
    object_id: int
    bbox_xyxy: tuple[float, float, float, float]
    corner_rois: list[CornerRoi]
    track_id: int | None = None


@dataclass(frozen=True)
class RoiMessage:
    objects: list[RoiObject]
    stamp: float | None = None
    header: object | None = None


@dataclass(frozen=True)
class StampMsg:
    sec: int
    nanosec: int = 0


@dataclass(frozen=True)
class HeaderMsg:
    stamp: StampMsg


@dataclass(frozen=True)
class Vector3Msg:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class QuaternionMsg:
    x: float
    y: float
    z: float
    w: float


@dataclass(frozen=True)
class PoseMsg:
    position: Vector3Msg
    orientation: QuaternionMsg


@dataclass(frozen=True)
class BufferedPose:
    t: np.ndarray
    R: np.ndarray


@dataclass(frozen=True)
class RoiFrame:
    stamp: float
    points_cam_current: np.ndarray
    projected_uv: np.ndarray


def frame(stamp: float) -> DecodedCloudFrame:
    return DecodedCloudFrame(
        stamp=stamp,
        points_world_f32=np.array([[stamp, 0.0, 0.0]], dtype=np.float32),
    )


def decoded_frame(
    stamp: float,
    points_world_f32: list[list[float]],
    *,
    frame_id: str = "world",
) -> DecodedCloudFrame:
    return DecodedCloudFrame(
        stamp=stamp,
        points_world_f32=np.asarray(points_world_f32, dtype=np.float32),
        frame_id=frame_id,
    )


def identity_pose_msg() -> PoseMsg:
    return PoseMsg(
        position=Vector3Msg(x=0.0, y=0.0, z=0.0),
        orientation=QuaternionMsg(x=0.0, y=0.0, z=0.0, w=1.0),
    )


def rotation_z_90() -> np.ndarray:
    return np.asarray(
        [
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )


def test_select_lookback_frames_is_newest_first_and_capped() -> None:
    frames = [frame(7.0), frame(8.0), frame(9.0), frame(10.0), frame(10.5)]

    selected = select_lookback_frames(frames, stamp=10.0, history_window_sec=3.0, max_window_frames=2)

    assert [item.stamp for item in selected] == [10.0, 9.0]
    assert 10.5 not in [item.stamp for item in selected]


def test_select_lookback_frames_returns_empty_when_max_window_non_positive() -> None:
    frames = [frame(9.0), frame(10.0)]

    assert select_lookback_frames(frames, stamp=10.0, history_window_sec=3.0, max_window_frames=0) == []
    assert select_lookback_frames(frames, stamp=10.0, history_window_sec=3.0, max_window_frames=-1) == []


def test_trim_cloud_cache_mutates_deque_in_place() -> None:
    cache = deque([frame(7.0), frame(8.0), frame(9.0), frame(10.0)])

    trim_cloud_cache(cache, newest_stamp=10.0, history_window_sec=1.1)

    assert [item.stamp for item in cache] == [9.0, 10.0]


def test_prepare_cached_points_filters_raw_numpy_input_and_accepts_voxel_size() -> None:
    points = np.array(
        [
            [0.1, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.1, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [4.0, 0.0, 0.0],
        ],
        dtype=np.float32,
    )

    prepared = prepare_cached_points(points, min_range=0.5, max_range=3.0, voxel_size=0.5)

    assert isinstance(prepared, np.ndarray)
    assert prepared.dtype == np.float32
    assert prepared.shape[1] == 3
    assert np.all((np.linalg.norm(prepared[:, :3], axis=1) >= 0.5) & (np.linalg.norm(prepared[:, :3], axis=1) <= 3.0))
    assert prepared.shape[0] == 2


def test_prepare_cached_points_preserves_empty_column_count_for_extra_columns() -> None:
    empty_points = np.zeros((0, 5), dtype=np.float32)
    filtered_empty_points = np.array(
        [
            [10.0, 0.0, 0.0, 1.0, 2.0],
            [11.0, 0.0, 0.0, 3.0, 4.0],
        ],
        dtype=np.float32,
    )

    prepared_empty = prepare_cached_points(empty_points, min_range=0.5, max_range=3.0, voxel_size=0.5)
    prepared_filtered_empty = prepare_cached_points(filtered_empty_points, min_range=0.5, max_range=3.0, voxel_size=0.5)

    assert prepared_empty.shape == (0, 5)
    assert prepared_filtered_empty.shape == (0, 5)


def roi_object(
    object_id: int,
    bbox_xyxy: tuple[float, float, float, float],
    corner_rois: list[CornerRoi],
) -> RoiObject:
    return RoiObject(
        object_id=object_id,
        track_id=object_id,
        bbox_xyxy=bbox_xyxy,
        corner_rois=corner_rois,
    )


def corner_roi(
    corner_label: int,
    roi_mask: object,
    *,
    mask_origin_x: int = 0,
    mask_origin_y: int = 0,
    mask_width: int | None = None,
    mask_height: int | None = None,
    valid: bool = True,
) -> CornerRoi:
    roi_mask_array = np.asarray(roi_mask)
    if mask_width is None or mask_height is None:
        if roi_mask_array.ndim != 2:
            raise ValueError("flat roi_mask requires explicit mask_width and mask_height")
        mask_height = int(roi_mask_array.shape[0])
        mask_width = int(roi_mask_array.shape[1])
    return CornerRoi(
        corner_label=corner_label,
        mask_origin_x=mask_origin_x,
        mask_origin_y=mask_origin_y,
        mask_width=int(mask_width),
        mask_height=int(mask_height),
        roi_mask=roi_mask,
        valid=valid,
    )


def object_corner(
    object_id: int,
    bbox_xyxy: tuple[float, float, float, float],
    roi_mask: object,
    *,
    corner_label: int = 0,
    mask_origin_x: int = 0,
    mask_origin_y: int = 0,
    mask_width: int | None = None,
    mask_height: int | None = None,
    valid: bool = True,
) -> dict[str, object]:
    roi_mask_array = np.asarray(roi_mask)
    if mask_width is None or mask_height is None:
        if roi_mask_array.ndim != 2:
            raise ValueError("flat roi_mask requires explicit mask_width and mask_height")
        mask_height = int(roi_mask_array.shape[0])
        mask_width = int(roi_mask_array.shape[1])
    return {
        "object_id": object_id,
        "track_id": object_id,
        "bbox_xyxy": bbox_xyxy,
        "corner_label": corner_label,
        "mask_origin_x": mask_origin_x,
        "mask_origin_y": mask_origin_y,
        "mask_width": int(mask_width),
        "mask_height": int(mask_height),
        "roi_mask": roi_mask,
        "valid": valid,
    }


def label_only_corner(
    label: int,
    bbox_xyxy: tuple[float, float, float, float],
    roi_mask: object,
) -> dict[str, object]:
    roi_mask_array = np.asarray(roi_mask)
    if roi_mask_array.ndim != 2:
        raise ValueError("flat roi_mask requires explicit mask_width and mask_height")
    return {
        "label": label,
        "bbox_xyxy": bbox_xyxy,
        "corner_label": 0,
        "mask_origin_x": 0,
        "mask_origin_y": 0,
        "mask_width": int(roi_mask_array.shape[1]),
        "mask_height": int(roi_mask_array.shape[0]),
        "roi_mask": roi_mask,
        "valid": True,
    }


def object_without_bbox(
    object_id: int,
    corner_rois: list[CornerRoi],
) -> dict[str, object]:
    return {
        "object_id": object_id,
        "track_id": object_id,
        "corner_rois": corner_rois,
    }


def roi_frame(
    stamp: float,
    points_cam_current: list[list[float]],
    projected_uv: list[list[float]],
) -> RoiFrame:
    return RoiFrame(
        stamp=stamp,
        points_cam_current=np.asarray(points_cam_current, dtype=np.float32),
        projected_uv=np.asarray(projected_uv, dtype=np.float32),
    )


def test_accumulate_window_hits_applies_bbox_padding_then_roi_mask() -> None:
    params = lookback_solver.LookbackSolveParams(
        bbox_expand_ratio=0.1,
        corner_target_points=3,
        corner_target_frames=1,
        corner_cap_points=8,
    )
    mask = np.zeros((40, 40), dtype=bool)
    mask[15, 9] = True
    mask[18, 18] = True
    obj = object_corner(object_id=7, bbox_xyxy=(10.0, 10.0, 20.0, 20.0), roi_mask=mask)
    expanded_bbox = lookback_solver.expand_bbox_xyxy(
        bbox_xyxy=obj["bbox_xyxy"],
        image_width=40,
        image_height=40,
        ratio=params.bbox_expand_ratio,
    )

    states = lookback_solver.accumulate_window_hits(
        obj=obj,
        expanded_bbox=expanded_bbox,
        points_cam_current=np.asarray(
            [
                [1.0, 0.0, 0.0],
                [2.0, 0.0, 0.0],
                [3.0, 0.0, 0.0],
                [4.0, 0.0, 0.0],
            ],
            dtype=np.float32,
        ),
        projected_uv=np.asarray(
            [
                [9.0, 15.0],
                [15.0, 15.0],
                [18.0, 18.0],
                [30.0, 30.0],
            ],
            dtype=np.float32,
        ),
        frame_stamp=1.0,
        corner_states={},
        params=params,
    )

    state = states[(7, 0)]
    np.testing.assert_allclose(state.points_cam_current, np.asarray([[1.0, 0.0, 0.0], [3.0, 0.0, 0.0]], dtype=np.float32))
    np.testing.assert_allclose(state.uv_hits, np.asarray([[9.0, 15.0], [18.0, 18.0]], dtype=np.float32))
    assert state.support_frames == 1
    assert state.frozen is False
    assert state.latest_frame_stamp == 1.0


def test_accumulate_window_hits_supports_flat_mask_with_origin_offset() -> None:
    obj = object_corner(
        object_id=7,
        bbox_xyxy=(10.0, 10.0, 20.0, 20.0),
        roi_mask=[0, 0, 0, 1],
        mask_origin_x=14,
        mask_origin_y=14,
        mask_width=2,
        mask_height=2,
    )

    states = lookback_solver.accumulate_window_hits(
        obj=obj,
        expanded_bbox=(10.0, 10.0, 20.0, 20.0),
        points_cam_current=np.asarray([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]], dtype=np.float32),
        projected_uv=np.asarray([[14.0, 14.0], [15.0, 15.0]], dtype=np.float32),
        frame_stamp=1.0,
        corner_states={},
        params=lookback_solver.LookbackSolveParams(),
    )

    state = states[(7, 0)]
    np.testing.assert_allclose(state.points_cam_current, np.asarray([[2.0, 0.0, 0.0]], dtype=np.float32))
    np.testing.assert_allclose(state.uv_hits, np.asarray([[15.0, 15.0]], dtype=np.float32))


def test_accumulate_window_hits_rejects_mismatched_point_and_uv_counts() -> None:
    obj = object_corner(
        object_id=7,
        bbox_xyxy=(10.0, 10.0, 20.0, 20.0),
        roi_mask=np.ones((4, 4), dtype=bool),
    )

    with pytest.raises(ValueError, match="points_cam_current and projected_uv"):
        lookback_solver.accumulate_window_hits(
            obj=obj,
            expanded_bbox=(10.0, 10.0, 20.0, 20.0),
            points_cam_current=np.asarray([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]], dtype=np.float32),
            projected_uv=np.asarray([[11.0, 11.0]], dtype=np.float32),
            frame_stamp=1.0,
            corner_states={},
            params=lookback_solver.LookbackSolveParams(),
        )


def test_corner_ready_requires_points_and_support_frames() -> None:
    params = lookback_solver.LookbackSolveParams(
        bbox_expand_ratio=0.0,
        corner_target_points=3,
        corner_target_frames=2,
        corner_cap_points=8,
    )

    enough_points_not_frames = lookback_solver.CornerAccumState(
        points_cam_current=np.asarray([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]], dtype=np.float32),
        uv_hits=np.asarray([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]], dtype=np.float32),
        support_frames=1,
        frozen=False,
        latest_frame_stamp=1.0,
    )
    enough_frames_not_points = lookback_solver.CornerAccumState(
        points_cam_current=np.asarray([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float32),
        uv_hits=np.asarray([[0.0, 0.0], [1.0, 0.0]], dtype=np.float32),
        support_frames=2,
        frozen=False,
        latest_frame_stamp=1.0,
    )
    ready_state = lookback_solver.CornerAccumState(
        points_cam_current=np.asarray([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]], dtype=np.float32),
        uv_hits=np.asarray([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]], dtype=np.float32),
        support_frames=2,
        frozen=False,
        latest_frame_stamp=1.0,
    )

    assert lookback_solver.corner_ready(enough_points_not_frames, params) is False
    assert lookback_solver.corner_ready(enough_frames_not_points, params) is False
    assert lookback_solver.corner_ready(ready_state, params) is True


def test_solve_lookback_window_uses_two_frames_for_static_target() -> None:
    params = lookback_solver.LookbackSolveParams(
        bbox_expand_ratio=0.1,
        corner_target_points=2,
        corner_target_frames=2,
        corner_cap_points=8,
    )
    mask = np.zeros((40, 40), dtype=bool)
    mask[15, 15] = True
    roi_msg = RoiMessage(
        stamp=10.0,
        objects=[roi_object(object_id=7, bbox_xyxy=(10.0, 10.0, 20.0, 20.0), corner_rois=[corner_roi(0, mask)])],
    )
    frames = [
        roi_frame(
            stamp=9.0,
            points_cam_current=[[1.0, 2.0, 3.0]],
            projected_uv=[[15.0, 15.0]],
        ),
        roi_frame(
            stamp=10.0,
            points_cam_current=[[1.0, 2.0, 3.0]],
            projected_uv=[[15.0, 15.0]],
        ),
    ]

    result = lookback_solver.solve_lookback_window(
        roi_msg=roi_msg,
        frames=frames,
        pose=None,
        fx=1.0,
        fy=1.0,
        cx=0.0,
        cy=0.0,
        image_width=40,
        image_height=40,
        t_cb=np.zeros(3, dtype=np.float32),
        R_cb=np.eye(3, dtype=np.float32),
        cloud_frame_mode="camera",
        params=params,
    )

    assert result["frames_used"] == 2
    state = result["corner_states"][(7, 0)]
    assert state.support_frames == 2
    assert state.frozen is True
    assert state.latest_frame_stamp == 10.0
    np.testing.assert_allclose(result["corners"][(7, 0)], np.asarray([1.0, 2.0, 3.0], dtype=np.float32))


def test_solve_lookback_window_is_order_stable_for_same_selected_frames() -> None:
    params = lookback_solver.LookbackSolveParams(
        bbox_expand_ratio=0.0,
        corner_target_points=1,
        corner_target_frames=2,
        corner_cap_points=1,
    )
    mask = np.zeros((40, 40), dtype=bool)
    mask[15, 15] = True
    roi_msg = RoiMessage(
        stamp=10.0,
        objects=[roi_object(object_id=7, bbox_xyxy=(10.0, 10.0, 20.0, 20.0), corner_rois=[corner_roi(0, mask)])],
    )
    ordered_frames = [
        roi_frame(
            stamp=9.0,
            points_cam_current=[[1.0, 2.0, 3.0]],
            projected_uv=[[15.0, 15.0]],
        ),
        roi_frame(
            stamp=10.0,
            points_cam_current=[[4.0, 5.0, 6.0]],
            projected_uv=[[15.0, 15.0]],
        ),
    ]
    reversed_frames = list(reversed(ordered_frames))

    ordered_result = lookback_solver.solve_lookback_window(
        roi_msg=roi_msg,
        frames=ordered_frames,
        pose=None,
        fx=1.0,
        fy=1.0,
        cx=0.0,
        cy=0.0,
        image_width=40,
        image_height=40,
        t_cb=np.zeros(3, dtype=np.float32),
        R_cb=np.eye(3, dtype=np.float32),
        cloud_frame_mode="camera",
        params=params,
    )
    reversed_result = lookback_solver.solve_lookback_window(
        roi_msg=roi_msg,
        frames=reversed_frames,
        pose=None,
        fx=1.0,
        fy=1.0,
        cx=0.0,
        cy=0.0,
        image_width=40,
        image_height=40,
        t_cb=np.zeros(3, dtype=np.float32),
        R_cb=np.eye(3, dtype=np.float32),
        cloud_frame_mode="camera",
        params=params,
    )

    assert ordered_result["frames_used"] == 2
    assert reversed_result["frames_used"] == 2
    assert ordered_result["corners"][(7, 0)] is None
    assert reversed_result["corners"][(7, 0)] is None
    assert ordered_result["corner_states"][(7, 0)].support_frames == 1
    assert reversed_result["corner_states"][(7, 0)].support_frames == 1
    np.testing.assert_allclose(
        ordered_result["corner_states"][(7, 0)].points_cam_current,
        reversed_result["corner_states"][(7, 0)].points_cam_current,
    )
    np.testing.assert_allclose(
        ordered_result["corner_states"][(7, 0)].points_cam_current,
        np.asarray([[4.0, 5.0, 6.0]], dtype=np.float32),
    )


def test_solve_lookback_window_corner_cap_recomputes_support_frames_from_retained_points() -> None:
    params = lookback_solver.LookbackSolveParams(
        bbox_expand_ratio=0.0,
        corner_target_points=2,
        corner_target_frames=2,
        corner_cap_points=2,
    )
    mask = np.zeros((40, 40), dtype=bool)
    mask[15, 15] = True
    roi_msg = RoiMessage(
        stamp=10.0,
        objects=[roi_object(object_id=7, bbox_xyxy=(10.0, 10.0, 20.0, 20.0), corner_rois=[corner_roi(0, mask)])],
    )
    frames = [
        roi_frame(
            stamp=9.0,
            points_cam_current=[[1.0, 2.0, 3.0]],
            projected_uv=[[15.0, 15.0]],
        ),
        roi_frame(
            stamp=10.0,
            points_cam_current=[[4.0, 5.0, 6.0], [7.0, 8.0, 9.0]],
            projected_uv=[[15.0, 15.0], [15.0, 15.0]],
        ),
    ]

    result = lookback_solver.solve_lookback_window(
        roi_msg=roi_msg,
        frames=frames,
        pose=None,
        fx=1.0,
        fy=1.0,
        cx=0.0,
        cy=0.0,
        image_width=40,
        image_height=40,
        t_cb=np.zeros(3, dtype=np.float32),
        R_cb=np.eye(3, dtype=np.float32),
        cloud_frame_mode="camera",
        params=params,
    )

    state = result["corner_states"][(7, 0)]
    assert result["frames_used"] == 2
    assert state.support_frames == 1
    assert state.frozen is False
    assert result["corners"][(7, 0)] is None
    np.testing.assert_allclose(
        state.points_cam_current,
        np.asarray([[4.0, 5.0, 6.0], [7.0, 8.0, 9.0]], dtype=np.float32),
    )


def test_solve_lookback_window_accumulates_and_freezes_per_corner_independently() -> None:
    params = lookback_solver.LookbackSolveParams(
        bbox_expand_ratio=0.0,
        corner_target_points=1,
        corner_target_frames=2,
        corner_cap_points=8,
    )
    corner0_mask = np.zeros((40, 40), dtype=bool)
    corner0_mask[15, 15] = True
    corner1_mask = np.zeros((40, 40), dtype=bool)
    corner1_mask[16, 16] = True
    roi_msg = RoiMessage(
        stamp=10.0,
        objects=[
            roi_object(
                object_id=7,
                bbox_xyxy=(10.0, 10.0, 20.0, 20.0),
                corner_rois=[corner_roi(0, corner0_mask), corner_roi(1, corner1_mask)],
            )
        ],
    )
    frames = [
        roi_frame(
            stamp=9.0,
            points_cam_current=[[1.0, 2.0, 3.0], [9.0, 9.0, 9.0]],
            projected_uv=[[15.0, 15.0], [30.0, 30.0]],
        ),
        roi_frame(
            stamp=10.0,
            points_cam_current=[[4.0, 5.0, 6.0], [7.0, 8.0, 9.0]],
            projected_uv=[[15.0, 15.0], [16.0, 16.0]],
        ),
    ]

    result = lookback_solver.solve_lookback_window(
        roi_msg=roi_msg,
        frames=frames,
        pose=None,
        fx=1.0,
        fy=1.0,
        cx=0.0,
        cy=0.0,
        image_width=40,
        image_height=40,
        t_cb=np.zeros(3, dtype=np.float32),
        R_cb=np.eye(3, dtype=np.float32),
        cloud_frame_mode="camera",
        params=params,
    )

    corner0_state = result["corner_states"][(7, 0)]
    corner1_state = result["corner_states"][(7, 1)]
    assert corner0_state.support_frames == 2
    assert corner0_state.frozen is True
    assert corner1_state.support_frames == 1
    assert corner1_state.frozen is False
    np.testing.assert_allclose(result["corners"][(7, 0)], np.asarray([2.5, 3.5, 4.5], dtype=np.float32))
    assert result["corners"][(7, 1)] is None


def test_solve_lookback_window_uses_robust_center_when_support_contains_outlier() -> None:
    params = lookback_solver.LookbackSolveParams(
        bbox_expand_ratio=0.0,
        corner_target_points=1,
        corner_target_frames=1,
        corner_cap_points=8,
    )
    mask = np.zeros((40, 40), dtype=bool)
    mask[15, 15] = True
    roi_msg = RoiMessage(
        stamp=10.0,
        objects=[roi_object(object_id=7, bbox_xyxy=(10.0, 10.0, 20.0, 20.0), corner_rois=[corner_roi(0, mask)])],
    )
    frames = [
        roi_frame(
            stamp=10.0,
            points_cam_current=[
                [1.0, 2.0, 3.0],
                [1.2, 2.2, 3.2],
                [30.0, 40.0, 50.0],
            ],
            projected_uv=[[15.0, 15.0], [15.0, 15.0], [15.0, 15.0]],
        ),
    ]

    result = lookback_solver.solve_lookback_window(
        roi_msg=roi_msg,
        frames=frames,
        pose=None,
        fx=1.0,
        fy=1.0,
        cx=0.0,
        cy=0.0,
        image_width=40,
        image_height=40,
        t_cb=np.zeros(3, dtype=np.float32),
        R_cb=np.eye(3, dtype=np.float32),
        cloud_frame_mode="camera",
        params=params,
    )

    np.testing.assert_allclose(
        result["corners"][(7, 0)],
        np.asarray([1.1, 2.1, 3.1], dtype=np.float32),
        atol=1e-5,
    )
    robust_support = result["corner_support"][(7, 0)]
    np.testing.assert_allclose(
        robust_support.points_cam_current,
        np.asarray([[1.0, 2.0, 3.0], [1.2, 2.2, 3.2]], dtype=np.float32),
        atol=1e-5,
    )
    np.testing.assert_allclose(
        robust_support.uv_hits,
        np.asarray([[15.0, 15.0], [15.0, 15.0]], dtype=np.float32),
        atol=1e-5,
    )


def test_solve_lookback_window_rejects_corner_when_robust_support_drops_below_point_target() -> None:
    params = lookback_solver.LookbackSolveParams(
        bbox_expand_ratio=0.0,
        corner_target_points=3,
        corner_target_frames=1,
        corner_cap_points=8,
    )
    mask = np.zeros((40, 40), dtype=bool)
    mask[15, 15] = True
    roi_msg = RoiMessage(
        stamp=10.0,
        objects=[roi_object(object_id=7, bbox_xyxy=(10.0, 10.0, 20.0, 20.0), corner_rois=[corner_roi(0, mask)])],
    )
    frames = [
        roi_frame(
            stamp=10.0,
            points_cam_current=[
                [1.0, 2.0, 3.0],
                [1.2, 2.2, 3.2],
                [30.0, 40.0, 50.0],
            ],
            projected_uv=[[15.0, 15.0], [15.0, 15.0], [15.0, 15.0]],
        ),
    ]

    result = lookback_solver.solve_lookback_window(
        roi_msg=roi_msg,
        frames=frames,
        pose=None,
        fx=1.0,
        fy=1.0,
        cx=0.0,
        cy=0.0,
        image_width=40,
        image_height=40,
        t_cb=np.zeros(3, dtype=np.float32),
        R_cb=np.eye(3, dtype=np.float32),
        cloud_frame_mode="camera",
        params=params,
    )

    robust_support = result["corner_support"][(7, 0)]
    assert robust_support.support_frames == 1
    assert robust_support.frozen is False
    assert result["corners"][(7, 0)] is None
    np.testing.assert_allclose(
        robust_support.points_cam_current,
        np.asarray([[1.0, 2.0, 3.0], [1.2, 2.2, 3.2]], dtype=np.float32),
        atol=1e-5,
    )


def test_solve_lookback_window_rejects_corner_when_robust_support_drops_below_frame_target() -> None:
    params = lookback_solver.LookbackSolveParams(
        bbox_expand_ratio=0.0,
        corner_target_points=2,
        corner_target_frames=2,
        corner_cap_points=8,
    )
    mask = np.zeros((40, 40), dtype=bool)
    mask[15, 15] = True
    roi_msg = RoiMessage(
        stamp=10.0,
        objects=[roi_object(object_id=7, bbox_xyxy=(10.0, 10.0, 20.0, 20.0), corner_rois=[corner_roi(0, mask)])],
    )
    frames = [
        roi_frame(
            stamp=9.0,
            points_cam_current=[[30.0, 40.0, 50.0]],
            projected_uv=[[15.0, 15.0]],
        ),
        roi_frame(
            stamp=10.0,
            points_cam_current=[[1.0, 2.0, 3.0], [1.2, 2.2, 3.2]],
            projected_uv=[[15.0, 15.0], [15.0, 15.0]],
        ),
    ]

    result = lookback_solver.solve_lookback_window(
        roi_msg=roi_msg,
        frames=frames,
        pose=None,
        fx=1.0,
        fy=1.0,
        cx=0.0,
        cy=0.0,
        image_width=40,
        image_height=40,
        t_cb=np.zeros(3, dtype=np.float32),
        R_cb=np.eye(3, dtype=np.float32),
        cloud_frame_mode="camera",
        params=params,
    )

    robust_support = result["corner_support"][(7, 0)]
    assert robust_support.support_frames == 1
    assert robust_support.frozen is False
    assert result["corners"][(7, 0)] is None
    np.testing.assert_allclose(
        robust_support.points_cam_current,
        np.asarray([[1.0, 2.0, 3.0], [1.2, 2.2, 3.2]], dtype=np.float32),
        atol=1e-5,
    )


def test_solve_lookback_window_rejects_label_only_object_keys() -> None:
    mask = np.zeros((40, 40), dtype=bool)
    mask[15, 15] = True
    roi_msg = RoiMessage(
        stamp=10.0,
        objects=[label_only_corner(label=7, bbox_xyxy=(10.0, 10.0, 20.0, 20.0), roi_mask=mask)],
    )

    with pytest.raises(AttributeError, match="identifier"):
        lookback_solver.solve_lookback_window(
            roi_msg=roi_msg,
            frames=[roi_frame(stamp=10.0, points_cam_current=[[1.0, 2.0, 3.0]], projected_uv=[[15.0, 15.0]])],
            pose=None,
            fx=1.0,
            fy=1.0,
            cx=0.0,
            cy=0.0,
            image_width=40,
            image_height=40,
            t_cb=np.zeros(3, dtype=np.float32),
            R_cb=np.eye(3, dtype=np.float32),
            cloud_frame_mode="camera",
            params=lookback_solver.LookbackSolveParams(),
        )


def test_solve_lookback_window_derives_bbox_from_corner_rois_when_bbox_missing() -> None:
    params = lookback_solver.LookbackSolveParams(
        bbox_expand_ratio=0.0,
        corner_target_points=1,
        corner_target_frames=1,
        corner_cap_points=8,
    )
    tight_mask = np.ones((2, 2), dtype=bool)
    roi_msg = RoiMessage(
        stamp=10.0,
        objects=[
            object_without_bbox(
                object_id=7,
                corner_rois=[
                    corner_roi(0, tight_mask, mask_origin_x=14, mask_origin_y=14),
                    corner_roi(1, tight_mask, mask_origin_x=30, mask_origin_y=30, valid=False),
                ],
            )
        ],
    )

    result = lookback_solver.solve_lookback_window(
        roi_msg=roi_msg,
        frames=[roi_frame(stamp=10.0, points_cam_current=[[1.0, 2.0, 3.0]], projected_uv=[[15.0, 15.0]])],
        pose=None,
        fx=1.0,
        fy=1.0,
        cx=0.0,
        cy=0.0,
        image_width=40,
        image_height=40,
        t_cb=np.zeros(3, dtype=np.float32),
        R_cb=np.eye(3, dtype=np.float32),
        cloud_frame_mode="camera",
        params=params,
    )

    state = result["corner_states"][(7, 0)]
    assert state.support_frames == 1
    assert state.frozen is True
    np.testing.assert_allclose(result["corners"][(7, 0)], np.asarray([1.0, 2.0, 3.0], dtype=np.float32))
    assert (7, 1) not in result["corner_states"]


def test_solve_lookback_window_supports_flat_corner_roi_masks() -> None:
    params = lookback_solver.LookbackSolveParams(
        bbox_expand_ratio=0.0,
        corner_target_points=2,
        corner_target_frames=2,
        corner_cap_points=8,
    )
    roi_msg = RoiMessage(
        stamp=10.0,
        objects=[
            object_without_bbox(
                object_id=7,
                corner_rois=[
                    corner_roi(
                        0,
                        [0, 1, 0, 0],
                        mask_origin_x=14,
                        mask_origin_y=14,
                        mask_width=2,
                        mask_height=2,
                    )
                ],
            )
        ],
    )
    frames = [
        roi_frame(
            stamp=9.0,
            points_cam_current=[[1.0, 2.0, 3.0], [9.0, 9.0, 9.0]],
            projected_uv=[[15.0, 14.0], [14.0, 15.0]],
        ),
        roi_frame(
            stamp=10.0,
            points_cam_current=[[4.0, 5.0, 6.0], [7.0, 8.0, 9.0]],
            projected_uv=[[15.0, 14.0], [15.0, 15.0]],
        ),
    ]

    result = lookback_solver.solve_lookback_window(
        roi_msg=roi_msg,
        frames=frames,
        pose=None,
        fx=1.0,
        fy=1.0,
        cx=0.0,
        cy=0.0,
        image_width=40,
        image_height=40,
        t_cb=np.zeros(3, dtype=np.float32),
        R_cb=np.eye(3, dtype=np.float32),
        cloud_frame_mode="camera",
        params=params,
    )

    state = result["corner_states"][(7, 0)]
    assert state.support_frames == 2
    assert state.frozen is True
    np.testing.assert_allclose(
        state.points_cam_current,
        np.asarray([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32),
    )
    np.testing.assert_allclose(result["corners"][(7, 0)], np.asarray([2.5, 3.5, 4.5], dtype=np.float32))


def test_solve_lookback_window_stops_after_all_corners_frozen_and_counts_traversed_frames() -> None:
    params = lookback_solver.LookbackSolveParams(
        bbox_expand_ratio=0.0,
        corner_target_points=1,
        corner_target_frames=2,
        corner_cap_points=8,
    )
    corner0_mask = np.zeros((40, 40), dtype=bool)
    corner0_mask[15, 15] = True
    corner1_mask = np.zeros((40, 40), dtype=bool)
    corner1_mask[16, 16] = True
    roi_msg = RoiMessage(
        stamp=10.0,
        objects=[
            roi_object(
                object_id=7,
                bbox_xyxy=(10.0, 10.0, 20.0, 20.0),
                corner_rois=[corner_roi(0, corner0_mask), corner_roi(1, corner1_mask)],
            )
        ],
    )
    frames = [
        roi_frame(
            stamp=8.0,
            points_cam_current=[[100.0, 100.0, 100.0], [200.0, 200.0, 200.0]],
            projected_uv=[[15.0, 15.0], [16.0, 16.0]],
        ),
        roi_frame(
            stamp=9.0,
            points_cam_current=[[1.0, 1.0, 1.0], [2.0, 2.0, 2.0]],
            projected_uv=[[15.0, 15.0], [16.0, 16.0]],
        ),
        roi_frame(
            stamp=10.0,
            points_cam_current=[[3.0, 3.0, 3.0], [4.0, 4.0, 4.0]],
            projected_uv=[[15.0, 15.0], [16.0, 16.0]],
        ),
    ]

    result = lookback_solver.solve_lookback_window(
        roi_msg=roi_msg,
        frames=frames,
        pose=None,
        fx=1.0,
        fy=1.0,
        cx=0.0,
        cy=0.0,
        image_width=40,
        image_height=40,
        t_cb=np.zeros(3, dtype=np.float32),
        R_cb=np.eye(3, dtype=np.float32),
        cloud_frame_mode="camera",
        params=params,
    )

    assert result["frames_used"] == 2
    np.testing.assert_allclose(
        result["corner_states"][(7, 0)].points_cam_current,
        np.asarray([[1.0, 1.0, 1.0], [3.0, 3.0, 3.0]], dtype=np.float32),
    )
    np.testing.assert_allclose(
        result["corner_states"][(7, 1)].points_cam_current,
        np.asarray([[2.0, 2.0, 2.0], [4.0, 4.0, 4.0]], dtype=np.float32),
    )
    np.testing.assert_allclose(result["corners"][(7, 0)], np.asarray([2.0, 2.0, 2.0], dtype=np.float32))
    np.testing.assert_allclose(result["corners"][(7, 1)], np.asarray([3.0, 3.0, 3.0], dtype=np.float32))


def test_solve_lookback_window_projects_decoded_world_frames_and_returns_stats() -> None:
    params = lookback_solver.LookbackSolveParams(
        bbox_expand_ratio=0.0,
        corner_target_points=2,
        corner_target_frames=2,
        corner_cap_points=8,
    )
    mask = np.ones((3, 3), dtype=bool)
    roi_msg = RoiMessage(
        objects=[
            object_without_bbox(
                object_id=7,
                corner_rois=[corner_roi(0, mask, mask_origin_x=19, mask_origin_y=19)],
            )
        ],
        header=HeaderMsg(stamp=StampMsg(sec=10, nanosec=0)),
    )
    frames = [
        decoded_frame(9.0, [[0.0, 0.0, 5.0]], frame_id="world"),
        decoded_frame(10.0, [[0.0, 0.0, 5.0]], frame_id="world"),
    ]

    result = lookback_solver.solve_lookback_window(
        roi_msg=roi_msg,
        frames=frames,
        pose=identity_pose_msg(),
        fx=10.0,
        fy=10.0,
        cx=20.0,
        cy=20.0,
        image_width=40,
        image_height=40,
        t_cb=np.zeros(3, dtype=np.float32),
        R_cb=np.eye(3, dtype=np.float32),
        cloud_frame_mode="world",
        params=params,
    )

    state = result["corner_states"][(7, 0)]
    stats = result["stats"]
    assert state.support_frames == 2
    assert state.frozen is True
    np.testing.assert_allclose(state.uv_hits, np.asarray([[20.0, 20.0], [20.0, 20.0]], dtype=np.float32))
    np.testing.assert_allclose(result["corners"][(7, 0)], np.asarray([0.0, 0.0, 5.0], dtype=np.float32))
    assert set(stats) >= {
        "frames_seen",
        "frames_used",
        "stop_reason",
        "points_raw_sum",
        "points_after_cache_filter_sum",
        "points_after_bbox_sum",
        "points_after_mask_sum",
        "solve_ms",
    }
    assert stats["frames_seen"] == 2
    assert stats["frames_used"] == 2
    assert stats["stop_reason"] == "all_corners_frozen"
    assert stats["points_raw_sum"] == 2
    assert stats["points_after_cache_filter_sum"] == 2
    assert stats["points_after_bbox_sum"] == 2
    assert stats["points_after_mask_sum"] == 2
    assert stats["solve_ms"] >= 0.0


def test_solve_lookback_window_reports_no_cloud_in_window_stats() -> None:
    roi_msg = RoiMessage(
        objects=[
            object_without_bbox(
                object_id=7,
                corner_rois=[corner_roi(0, np.ones((3, 3), dtype=bool), mask_origin_x=19, mask_origin_y=19)],
            )
        ],
        header=HeaderMsg(stamp=StampMsg(sec=10, nanosec=0)),
    )

    result = lookback_solver.solve_lookback_window(
        roi_msg=roi_msg,
        frames=[decoded_frame(11.0, [[0.0, 0.0, 5.0]], frame_id="world")],
        pose=identity_pose_msg(),
        fx=10.0,
        fy=10.0,
        cx=20.0,
        cy=20.0,
        image_width=40,
        image_height=40,
        t_cb=np.zeros(3, dtype=np.float32),
        R_cb=np.eye(3, dtype=np.float32),
        cloud_frame_mode="world",
        params=lookback_solver.LookbackSolveParams(),
    )

    assert result["stats"]["frames_seen"] == 0
    assert result["stats"]["frames_used"] == 0
    assert result["stats"]["stop_reason"] == "no_cloud_in_window"
    assert result["stats"]["points_raw_sum"] == 0
    assert result["stats"]["points_after_bbox_sum"] == 0
    assert result["stats"]["points_after_mask_sum"] == 0
    assert result["corners"][(7, 0)] is None


def test_solve_lookback_window_reports_no_points_after_bbox_stop_reason() -> None:
    roi_msg = RoiMessage(
        objects=[
            roi_object(
                object_id=7,
                bbox_xyxy=(5.0, 5.0, 10.0, 10.0),
                corner_rois=[corner_roi(0, np.ones((2, 2), dtype=bool), mask_origin_x=5, mask_origin_y=5)],
            )
        ],
        header=HeaderMsg(stamp=StampMsg(sec=10, nanosec=0)),
    )

    result = lookback_solver.solve_lookback_window(
        roi_msg=roi_msg,
        frames=[decoded_frame(10.0, [[0.0, 0.0, 5.0]], frame_id="world")],
        pose=identity_pose_msg(),
        fx=10.0,
        fy=10.0,
        cx=20.0,
        cy=20.0,
        image_width=40,
        image_height=40,
        t_cb=np.zeros(3, dtype=np.float32),
        R_cb=np.eye(3, dtype=np.float32),
        cloud_frame_mode="world",
        params=lookback_solver.LookbackSolveParams(),
    )

    assert result["stats"]["frames_seen"] == 1
    assert result["stats"]["frames_used"] == 1
    assert result["stats"]["stop_reason"] == "no_points_after_bbox"
    assert result["stats"]["points_raw_sum"] == 1
    assert result["stats"]["points_after_cache_filter_sum"] == 1
    assert result["stats"]["points_after_bbox_sum"] == 0
    assert result["stats"]["points_after_mask_sum"] == 0
    assert result["corners"][(7, 0)] is None


def test_solve_lookback_window_supports_buffered_pose_t_r_with_world_transform() -> None:
    params = lookback_solver.LookbackSolveParams(
        bbox_expand_ratio=0.0,
        corner_target_points=1,
        corner_target_frames=1,
        corner_cap_points=8,
    )
    pose = BufferedPose(
        t=np.asarray([1.0, 2.0, 3.0], dtype=np.float32),
        R=rotation_z_90(),
    )
    desired_base_point = np.asarray([2.0, 0.0, 5.0], dtype=np.float32)
    world_point = desired_base_point @ pose.R + pose.t
    roi_msg = RoiMessage(
        objects=[
            object_without_bbox(
                object_id=7,
                corner_rois=[corner_roi(0, np.ones((3, 3), dtype=bool), mask_origin_x=23, mask_origin_y=19)],
            )
        ],
        header=HeaderMsg(stamp=StampMsg(sec=10, nanosec=0)),
    )

    result = lookback_solver.solve_lookback_window(
        roi_msg=roi_msg,
        frames=[decoded_frame(10.0, [world_point.tolist()], frame_id="world")],
        pose=pose,
        fx=10.0,
        fy=10.0,
        cx=20.0,
        cy=20.0,
        image_width=40,
        image_height=40,
        t_cb=np.zeros(3, dtype=np.float32),
        R_cb=np.eye(3, dtype=np.float32),
        cloud_frame_mode="world",
        params=params,
    )

    state = result["corner_states"][(7, 0)]
    np.testing.assert_allclose(state.points_cam_current, np.asarray([[2.0, 0.0, 5.0]], dtype=np.float32))
    np.testing.assert_allclose(state.uv_hits, np.asarray([[24.0, 20.0]], dtype=np.float32))
    np.testing.assert_allclose(result["corners"][(7, 0)], np.asarray([2.0, 0.0, 5.0], dtype=np.float32))


def test_solve_lookback_window_uses_node_style_base_to_camera_translation() -> None:
    params = lookback_solver.LookbackSolveParams(
        bbox_expand_ratio=0.0,
        corner_target_points=1,
        corner_target_frames=1,
        corner_cap_points=8,
    )
    roi_msg = RoiMessage(
        objects=[
            object_without_bbox(
                object_id=7,
                corner_rois=[corner_roi(0, np.ones((3, 3), dtype=bool), mask_origin_x=19, mask_origin_y=19)],
            )
        ],
        header=HeaderMsg(stamp=StampMsg(sec=10, nanosec=0)),
    )

    result = lookback_solver.solve_lookback_window(
        roi_msg=roi_msg,
        frames=[decoded_frame(10.0, [[1.0, 0.0, 5.0]], frame_id="base")],
        pose=identity_pose_msg(),
        fx=10.0,
        fy=10.0,
        cx=20.0,
        cy=20.0,
        image_width=40,
        image_height=40,
        t_cb=np.asarray([1.0, 0.0, 0.0], dtype=np.float32),
        R_cb=np.eye(3, dtype=np.float32),
        cloud_frame_mode="base",
        params=params,
    )

    state = result["corner_states"][(7, 0)]
    np.testing.assert_allclose(state.points_cam_current, np.asarray([[0.0, 0.0, 5.0]], dtype=np.float32))
    np.testing.assert_allclose(state.uv_hits, np.asarray([[20.0, 20.0]], dtype=np.float32))
    np.testing.assert_allclose(result["corners"][(7, 0)], np.asarray([0.0, 0.0, 5.0], dtype=np.float32))
