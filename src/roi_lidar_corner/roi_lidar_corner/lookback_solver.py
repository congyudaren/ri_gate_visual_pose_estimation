from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import time
from typing import Any, Deque, Iterable, List, Sequence

import numpy as np


@dataclass(frozen=True)
class DecodedCloudFrame:
    stamp: float
    points_world_f32: np.ndarray
    frame_id: str = "world"


@dataclass(frozen=True)
class LookbackSolveParams:
    bbox_expand_ratio: float = 0.0
    corner_target_points: int = 1
    corner_target_frames: int = 1
    corner_cap_points: int = 0
    min_range: float = 0.0
    max_range: float = float("inf")


@dataclass(frozen=True)
class CornerAccumState:
    points_cam_current: np.ndarray
    uv_hits: np.ndarray
    support_frames: int = 0
    frozen: bool = False
    latest_frame_stamp: float | None = None
    _point_frame_stamps: tuple[float, ...] = field(default=(), init=False, repr=False, compare=False)


def select_lookback_frames(
    frames: Sequence[DecodedCloudFrame],
    stamp: float,
    history_window_sec: float,
    max_window_frames: int,
) -> List[DecodedCloudFrame]:
    if int(max_window_frames) <= 0:
        return []

    lower_bound = float(stamp) - float(history_window_sec)
    selected: List[DecodedCloudFrame] = []
    for frame in reversed(frames):
        if float(frame.stamp) < lower_bound:
            break
        if float(frame.stamp) <= float(stamp):
            selected.append(frame)
            if len(selected) >= int(max_window_frames):
                break
    return selected


def trim_cloud_cache(
    cache: Deque[DecodedCloudFrame],
    newest_stamp: float,
    history_window_sec: float,
) -> None:
    lower_bound = float(newest_stamp) - float(history_window_sec)
    while cache and float(cache[0].stamp) < lower_bound:
        cache.popleft()


def prepare_cached_points(
    points_world_f32: np.ndarray,
    min_range: float,
    max_range: float,
    voxel_size: float,
    *,
    apply_range_filter: bool = True,
) -> np.ndarray:
    points = np.asarray(points_world_f32, dtype=np.float32)
    column_count = points.shape[1] if points.ndim == 2 else 3
    if points.size == 0:
        return points.reshape(0, column_count)

    filtered = points
    if apply_range_filter:
        ranges = np.linalg.norm(points[:, :3], axis=1)
        mask = (ranges >= float(min_range)) & (ranges <= float(max_range))
        filtered = points[mask]
        if filtered.size == 0:
            return filtered.reshape(0, column_count)

    voxel_size = float(voxel_size)
    if voxel_size <= 0.0:
        return filtered

    voxels = np.floor(filtered[:, :3] / voxel_size).astype(np.int64)
    _, keep_indices = np.unique(voxels, axis=0, return_index=True)
    keep_indices = np.sort(keep_indices)
    return filtered[keep_indices]


def corner_ready(state: CornerAccumState, params: LookbackSolveParams) -> bool:
    return (
        int(np.asarray(state.points_cam_current).shape[0]) >= int(params.corner_target_points)
        and int(state.support_frames) >= int(params.corner_target_frames)
    )


def expand_bbox_xyxy(
    bbox_xyxy: Sequence[float],
    image_width: int,
    image_height: int,
    ratio: float,
) -> tuple[float, float, float, float]:
    x1, y1, x2, y2 = (float(value) for value in bbox_xyxy)
    expand_x = max(0.0, (x2 - x1) * float(ratio))
    expand_y = max(0.0, (y2 - y1) * float(ratio))
    max_x = max(0.0, float(image_width) - 1.0)
    max_y = max(0.0, float(image_height) - 1.0)
    return (
        max(0.0, x1 - expand_x),
        max(0.0, y1 - expand_y),
        min(max_x, x2 + expand_x),
        min(max_y, y2 + expand_y),
    )


def _value(obj: Any, name: str) -> Any:
    if isinstance(obj, dict):
        return obj[name]
    return getattr(obj, name)


def _optional_value(obj: Any, name: str, default: Any = None) -> Any:
    if isinstance(obj, dict):
        return obj.get(name, default)
    return getattr(obj, name, default)


def _stamp_seconds(value: Any) -> float:
    sec = _optional_value(value, "sec", None)
    if sec is not None:
        return float(sec) + float(_optional_value(value, "nanosec", 0)) * 1e-9
    return float(value)


def _message_stamp_seconds(msg: Any) -> float:
    stamp = _optional_value(msg, "stamp", None)
    if stamp is not None:
        return _stamp_seconds(stamp)

    header = _optional_value(msg, "header", None)
    if header is not None:
        header_stamp = _optional_value(header, "stamp", None)
        if header_stamp is not None:
            return _stamp_seconds(header_stamp)

    raise AttributeError("Message is missing stamp/header.stamp")


def _frame_stamp_seconds(frame: Any) -> float:
    return _stamp_seconds(_value(frame, "stamp"))


def _object_key(obj: Any) -> Any:
    for name in ("object_id", "track_id", "id"):
        if isinstance(obj, dict) and name in obj:
            return obj[name]
        if hasattr(obj, name):
            return getattr(obj, name)
    raise AttributeError("Object is missing an identifier field")


def _make_state(
    *,
    points_cam_current: np.ndarray,
    uv_hits: np.ndarray,
    support_frames: int = 0,
    frozen: bool = False,
    latest_frame_stamp: float | None = None,
    point_frame_stamps: Iterable[float] = (),
) -> CornerAccumState:
    state = CornerAccumState(
        points_cam_current=points_cam_current,
        uv_hits=uv_hits,
        support_frames=int(support_frames),
        frozen=bool(frozen),
        latest_frame_stamp=latest_frame_stamp,
    )
    object.__setattr__(state, "_point_frame_stamps", tuple(float(stamp) for stamp in point_frame_stamps))
    return state


def _empty_state() -> CornerAccumState:
    return _make_state(
        points_cam_current=np.zeros((0, 3), dtype=np.float32),
        uv_hits=np.zeros((0, 2), dtype=np.float32),
    )


def _corner_key(obj: Any) -> tuple[Any, Any]:
    return (_object_key(obj), _value(obj, "corner_label"))


def _cap_rows(values: np.ndarray, cap_points: int) -> np.ndarray:
    if int(cap_points) > 0 and values.shape[0] > int(cap_points):
        return values[-int(cap_points) :]
    return values


def _normalized_roi_mask(obj: Any) -> np.ndarray:
    roi_mask = np.asarray(_value(obj, "roi_mask"))
    if roi_mask.ndim == 2:
        return roi_mask.astype(bool, copy=False)

    mask_width = int(_optional_value(obj, "mask_width", 0))
    mask_height = int(_optional_value(obj, "mask_height", 0))
    if roi_mask.ndim == 1 and mask_width > 0 and mask_height > 0 and roi_mask.size == mask_width * mask_height:
        return roi_mask.reshape(mask_height, mask_width).astype(bool, copy=False)

    return np.zeros((0, 0), dtype=bool)


def _coarse_and_fine_masks(
    obj: Any,
    expanded_bbox: Sequence[float],
    projected_uv: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    if projected_uv.size == 0:
        empty = np.zeros(0, dtype=bool)
        return empty, empty

    x1, y1, x2, y2 = (float(value) for value in expanded_bbox)
    coarse_mask = (
        (projected_uv[:, 0] >= x1)
        & (projected_uv[:, 0] <= x2)
        & (projected_uv[:, 1] >= y1)
        & (projected_uv[:, 1] <= y2)
    )
    fine_mask = coarse_mask & _sample_roi_mask(obj, projected_uv)
    return coarse_mask, fine_mask


def _sample_roi_mask(obj: Any, projected_uv: np.ndarray) -> np.ndarray:
    roi_mask = _normalized_roi_mask(obj)
    if roi_mask.ndim != 2:
        return np.zeros(projected_uv.shape[0], dtype=bool)

    xs = np.rint(projected_uv[:, 0]).astype(np.int64) - int(_optional_value(obj, "mask_origin_x", 0))
    ys = np.rint(projected_uv[:, 1]).astype(np.int64) - int(_optional_value(obj, "mask_origin_y", 0))
    valid = (
        (xs >= 0)
        & (xs < roi_mask.shape[1])
        & (ys >= 0)
        & (ys < roi_mask.shape[0])
    )
    sampled = np.zeros(projected_uv.shape[0], dtype=bool)
    sampled[valid] = roi_mask[ys[valid], xs[valid]]
    return sampled


def _corner_entry(obj: Any, corner_roi: Any) -> dict[str, Any]:
    roi_mask = _value(corner_roi, "roi_mask")
    roi_mask_array = np.asarray(roi_mask)
    if _optional_value(corner_roi, "mask_width", None) is not None:
        mask_width = int(_value(corner_roi, "mask_width"))
    elif roi_mask_array.ndim == 2:
        mask_width = int(roi_mask_array.shape[1])
    else:
        raise ValueError("Corner ROI is missing mask_width for flat roi_mask")

    if _optional_value(corner_roi, "mask_height", None) is not None:
        mask_height = int(_value(corner_roi, "mask_height"))
    elif roi_mask_array.ndim == 2:
        mask_height = int(roi_mask_array.shape[0])
    else:
        raise ValueError("Corner ROI is missing mask_height for flat roi_mask")

    entry = {
        "corner_label": _value(corner_roi, "corner_label"),
        "mask_origin_x": int(_optional_value(corner_roi, "mask_origin_x", 0)),
        "mask_origin_y": int(_optional_value(corner_roi, "mask_origin_y", 0)),
        "mask_width": mask_width,
        "mask_height": mask_height,
        "roi_mask": roi_mask,
        "valid": bool(_optional_value(corner_roi, "valid", True)),
    }
    for name in ("object_id", "track_id", "id"):
        value = _optional_value(obj, name, None)
        if value is not None:
            entry[name] = value
    if _optional_value(obj, "bbox_xyxy", None) is not None:
        entry["bbox_xyxy"] = _value(obj, "bbox_xyxy")
    return entry


def _iter_corner_entries(obj: Any) -> list[Any]:
    corner_rois = _optional_value(obj, "corner_rois", None)
    if corner_rois is None:
        _corner_key(obj)
        if not bool(_optional_value(obj, "valid", True)):
            return []
        return [obj]

    object_id = _object_key(obj)
    del object_id
    return [
        _corner_entry(obj, corner_roi)
        for corner_roi in corner_rois
        if bool(_optional_value(corner_roi, "valid", True))
    ]


def _coarse_bbox_xyxy(obj: Any, corner_entries: Sequence[Any]) -> tuple[float, float, float, float]:
    bbox_xyxy = _optional_value(obj, "bbox_xyxy", None)
    if bbox_xyxy is not None:
        return tuple(float(value) for value in bbox_xyxy)

    if not corner_entries:
        raise AttributeError("Object is missing bbox_xyxy and valid corner_rois")

    x1 = min(float(_value(entry, "mask_origin_x")) for entry in corner_entries)
    y1 = min(float(_value(entry, "mask_origin_y")) for entry in corner_entries)
    x2 = max(float(_value(entry, "mask_origin_x")) + float(_value(entry, "mask_width")) - 1.0 for entry in corner_entries)
    y2 = max(float(_value(entry, "mask_origin_y")) + float(_value(entry, "mask_height")) - 1.0 for entry in corner_entries)
    return (x1, y1, x2, y2)


def _quaternion_to_rotation_matrix(quaternion: Any) -> np.ndarray:
    x = float(_value(quaternion, "x"))
    y = float(_value(quaternion, "y"))
    z = float(_value(quaternion, "z"))
    w = float(_value(quaternion, "w"))
    norm = np.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return np.eye(3, dtype=np.float32)
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )


def _pose_translation_and_rotation(pose: Any) -> tuple[np.ndarray, np.ndarray]:
    if pose is None:
        return np.zeros(3, dtype=np.float32), np.eye(3, dtype=np.float32)

    if isinstance(pose, np.ndarray):
        array = np.asarray(pose, dtype=np.float32)
        if array.shape == (4, 4):
            return array[:3, 3].astype(np.float32), array[:3, :3].astype(np.float32)

    translation = _optional_value(pose, "translation", None)
    rotation_matrix = _optional_value(pose, "rotation_matrix", None)
    if translation is not None and rotation_matrix is not None:
        return (
            np.asarray(translation, dtype=np.float32).reshape(3),
            np.asarray(rotation_matrix, dtype=np.float32).reshape(3, 3),
        )

    translation = _optional_value(pose, "t", None)
    rotation_matrix = _optional_value(pose, "R", None)
    if translation is not None and rotation_matrix is not None:
        return (
            np.asarray(translation, dtype=np.float32).reshape(3),
            np.asarray(rotation_matrix, dtype=np.float32).reshape(3, 3),
        )

    position = _optional_value(pose, "position", None)
    orientation = _optional_value(pose, "orientation", None)
    if position is not None and orientation is not None:
        return (
            np.asarray(
                [
                    float(_value(position, "x")),
                    float(_value(position, "y")),
                    float(_value(position, "z")),
                ],
                dtype=np.float32,
            ),
            _quaternion_to_rotation_matrix(orientation),
        )

    raise TypeError("Unsupported pose format")


def _effective_cloud_frame(frame: Any, cloud_frame_mode: str) -> str:
    frame_id = str(_optional_value(frame, "frame_id", "")).lower()
    if frame_id in {"world", "map", "odom"}:
        return "world"
    if frame_id in {"base", "base_link", "body"}:
        return "base"

    mode = str(cloud_frame_mode).lower()
    if mode in {"world", "map", "odom"}:
        return "world"
    return "base"


def _filter_points_by_range(points_xyz: np.ndarray, min_range: float, max_range: float) -> np.ndarray:
    if points_xyz.size == 0:
        return np.zeros(points_xyz.shape[0], dtype=bool)
    ranges = np.linalg.norm(points_xyz[:, :3], axis=1)
    return (ranges >= float(min_range)) & (ranges <= float(max_range))


def _robust_corner_center(points_cam_current: np.ndarray) -> np.ndarray | None:
    points = np.asarray(points_cam_current, dtype=np.float32).reshape(-1, 3)
    if points.shape[0] == 0:
        return None
    if points.shape[0] <= 2:
        return points.mean(axis=0).astype(np.float32)

    median = np.median(points, axis=0)
    distances = np.linalg.norm(points - median, axis=1)
    cutoff = float(np.percentile(distances, 80.0))
    inliers = points[distances <= cutoff]
    if inliers.shape[0] == 0:
        inliers = points
    return inliers.mean(axis=0).astype(np.float32)


def _robust_corner_support(
    points_cam_current: np.ndarray,
    uv_hits: np.ndarray,
    point_frame_stamps: Iterable[float] = (),
) -> tuple[np.ndarray, np.ndarray, tuple[float, ...]]:
    points = np.asarray(points_cam_current, dtype=np.float32).reshape(-1, 3)
    uv = np.asarray(uv_hits, dtype=np.float32).reshape(-1, 2)
    stamps = tuple(float(stamp) for stamp in point_frame_stamps)
    if points.shape[0] == 0:
        return (
            np.zeros((0, 3), dtype=np.float32),
            np.zeros((0, 2), dtype=np.float32),
            tuple(),
        )
    if points.shape[0] <= 2:
        return points, uv, stamps

    median = np.median(points, axis=0)
    distances = np.linalg.norm(points - median, axis=1)
    cutoff = float(np.percentile(distances, 80.0))
    inlier_mask = distances <= cutoff
    if not np.any(inlier_mask):
        inlier_mask = np.ones(points.shape[0], dtype=bool)
    inlier_stamps = stamps
    if len(stamps) == points.shape[0]:
        inlier_stamps = tuple(stamp for stamp, keep in zip(stamps, inlier_mask) if keep)
    return (
        points[inlier_mask].astype(np.float32, copy=False),
        uv[inlier_mask].astype(np.float32, copy=False),
        inlier_stamps,
    )


def _project_cached_frame(
    *,
    frame: Any,
    pose: Any,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    image_width: int,
    image_height: int,
    t_cb: np.ndarray,
    R_cb: np.ndarray,
    cloud_frame_mode: str,
    min_range: float = 0.0,
    max_range: float = float("inf"),
) -> tuple[np.ndarray, np.ndarray, int, int]:
    legacy_points = _optional_value(frame, "points_cam_current", None)
    legacy_uv = _optional_value(frame, "projected_uv", None)
    if legacy_points is not None or legacy_uv is not None:
        if legacy_points is None or legacy_uv is None:
            raise ValueError("Legacy projected frames require both points_cam_current and projected_uv")
        points_cam_current = np.asarray(legacy_points, dtype=np.float32).reshape(-1, 3)
        projected_uv = np.asarray(legacy_uv, dtype=np.float32).reshape(-1, 2)
        if points_cam_current.shape[0] != projected_uv.shape[0]:
            raise ValueError("points_cam_current and projected_uv must have the same row count")
        return points_cam_current, projected_uv, int(points_cam_current.shape[0]), int(points_cam_current.shape[0])

    points_world_f32 = np.asarray(_value(frame, "points_world_f32"), dtype=np.float32)
    column_count = points_world_f32.shape[1] if points_world_f32.ndim == 2 else 3
    points_world_f32 = points_world_f32.reshape(-1, column_count)
    raw_count = int(points_world_f32.shape[0])
    if raw_count <= 0:
        return (
            np.zeros((0, 3), dtype=np.float32),
            np.zeros((0, 2), dtype=np.float32),
            0,
            0,
        )

    points_xyz = points_world_f32[:, :3].astype(np.float32, copy=False)
    frame_mode = _effective_cloud_frame(frame, cloud_frame_mode)
    if frame_mode == "world":
        t_wb, R_wb = _pose_translation_and_rotation(pose)
        points_base = ((points_xyz - t_wb.reshape(1, 3)) @ R_wb.T).astype(np.float32, copy=False)
    else:
        points_base = points_xyz

    range_mask = _filter_points_by_range(points_base, min_range=min_range, max_range=max_range)
    points_base = points_base[range_mask]
    after_cache_count = int(points_base.shape[0])
    if after_cache_count <= 0:
        return (
            np.zeros((0, 3), dtype=np.float32),
            np.zeros((0, 2), dtype=np.float32),
            raw_count,
            0,
        )

    points_cam = ((points_base - np.asarray(t_cb, dtype=np.float32).reshape(1, 3)) @ np.asarray(R_cb, dtype=np.float32).reshape(3, 3)).astype(np.float32, copy=False)
    positive_depth = points_cam[:, 2] > 0.0
    if not np.any(positive_depth):
        return (
            np.zeros((0, 3), dtype=np.float32),
            np.zeros((0, 2), dtype=np.float32),
            raw_count,
            after_cache_count,
        )

    points_cam = points_cam[positive_depth]
    projected_uv = np.empty((points_cam.shape[0], 2), dtype=np.float32)
    projected_uv[:, 0] = float(fx) * points_cam[:, 0] / points_cam[:, 2] + float(cx)
    projected_uv[:, 1] = float(fy) * points_cam[:, 1] / points_cam[:, 2] + float(cy)
    in_image = (
        (projected_uv[:, 0] >= 0.0)
        & (projected_uv[:, 0] <= float(image_width) - 1.0)
        & (projected_uv[:, 1] >= 0.0)
        & (projected_uv[:, 1] <= float(image_height) - 1.0)
    )
    return (
        points_cam[in_image].astype(np.float32, copy=False),
        projected_uv[in_image].astype(np.float32, copy=False),
        raw_count,
        after_cache_count,
    )


def accumulate_window_hits(
    *,
    obj: Any,
    expanded_bbox: Sequence[float],
    points_cam_current: np.ndarray,
    projected_uv: np.ndarray,
    frame_stamp: float,
    corner_states: dict[Any, CornerAccumState],
    params: LookbackSolveParams,
) -> dict[Any, CornerAccumState]:
    key = _corner_key(obj)
    prev_state = corner_states.get(key, _empty_state())
    if prev_state.frozen:
        return dict(corner_states)

    points_cam_current = np.asarray(points_cam_current, dtype=np.float32).reshape(-1, 3)
    projected_uv = np.asarray(projected_uv, dtype=np.float32).reshape(-1, 2)
    if points_cam_current.shape[0] != projected_uv.shape[0]:
        raise ValueError("points_cam_current and projected_uv must have the same row count")
    count = points_cam_current.shape[0]
    if count <= 0:
        return dict(corner_states)

    _, fine_mask = _coarse_and_fine_masks(obj, expanded_bbox, projected_uv)
    if not np.any(fine_mask):
        return dict(corner_states)

    next_points = np.concatenate(
        [points_cam_current[fine_mask], prev_state.points_cam_current],
        axis=0,
    ).astype(np.float32, copy=False)
    next_uv = np.concatenate(
        [projected_uv[fine_mask], prev_state.uv_hits],
        axis=0,
    ).astype(np.float32, copy=False)
    next_point_frame_stamps = ((float(frame_stamp),) * int(np.count_nonzero(fine_mask))) + prev_state._point_frame_stamps
    next_points = _cap_rows(next_points, params.corner_cap_points)
    next_uv = _cap_rows(next_uv, params.corner_cap_points)
    if int(params.corner_cap_points) > 0 and len(next_point_frame_stamps) > int(params.corner_cap_points):
        next_point_frame_stamps = next_point_frame_stamps[-int(params.corner_cap_points) :]
    support_frames = len(set(next_point_frame_stamps))
    latest_frame_stamp = float(frame_stamp)
    if prev_state.latest_frame_stamp is not None:
        latest_frame_stamp = max(float(prev_state.latest_frame_stamp), latest_frame_stamp)
    next_state = _make_state(
        points_cam_current=next_points,
        uv_hits=next_uv,
        support_frames=support_frames,
        frozen=(
            int(next_points.shape[0]) >= int(params.corner_target_points)
            and support_frames >= int(params.corner_target_frames)
        ),
        latest_frame_stamp=latest_frame_stamp,
        point_frame_stamps=next_point_frame_stamps,
    )

    updated_states = dict(corner_states)
    updated_states[key] = next_state
    return updated_states


def solve_lookback_window(
    *,
    roi_msg: Any,
    frames: Sequence[Any],
    pose: Any,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    image_width: int,
    image_height: int,
    t_cb: np.ndarray,
    R_cb: np.ndarray,
    cloud_frame_mode: str,
    params: LookbackSolveParams,
) -> dict[str, Any]:
    start_time = time.perf_counter()
    roi_stamp = _message_stamp_seconds(roi_msg)
    selected_frames = sorted(
        (frame for frame in frames if _frame_stamp_seconds(frame) <= roi_stamp),
        key=_frame_stamp_seconds,
        reverse=True,
    )
    objects = list(_value(roi_msg, "objects"))
    corner_states: dict[Any, CornerAccumState] = {}
    corners: dict[Any, np.ndarray | None] = {}
    corner_support: dict[Any, CornerAccumState] = {}
    stats = {
        "frames_seen": len(selected_frames),
        "frames_used": 0,
        "stop_reason": "window_exhausted",
        "points_raw_sum": 0,
        "points_after_cache_filter_sum": 0,
        "points_after_bbox_sum": 0,
        "points_after_mask_sum": 0,
        "solve_ms": 0.0,
    }
    corner_entries_with_bbox: list[tuple[Any, tuple[float, float, float, float]]] = []
    for obj in objects:
        corner_entries = _iter_corner_entries(obj)
        if not corner_entries:
            continue
        expanded_bbox = expand_bbox_xyxy(
            bbox_xyxy=_coarse_bbox_xyxy(obj, corner_entries),
            image_width=image_width,
            image_height=image_height,
            ratio=params.bbox_expand_ratio,
        )
        for corner_entry in corner_entries:
            corner_entries_with_bbox.append((corner_entry, expanded_bbox))

    for frame in selected_frames:
        stats["frames_used"] += 1
        frame_points, frame_uv, raw_count, after_cache_count = _project_cached_frame(
            frame=frame,
            pose=pose,
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            image_width=image_width,
            image_height=image_height,
            t_cb=t_cb,
            R_cb=R_cb,
            cloud_frame_mode=cloud_frame_mode,
            min_range=params.min_range,
            max_range=params.max_range,
        )
        stats["points_raw_sum"] += raw_count
        stats["points_after_cache_filter_sum"] += after_cache_count
        frame_stamp = _frame_stamp_seconds(frame)
        for corner_entry, expanded_bbox in corner_entries_with_bbox:
            if corner_states.get(_corner_key(corner_entry), _empty_state()).frozen:
                continue
            coarse_mask, fine_mask = _coarse_and_fine_masks(corner_entry, expanded_bbox, frame_uv)
            stats["points_after_bbox_sum"] += int(np.count_nonzero(coarse_mask))
            stats["points_after_mask_sum"] += int(np.count_nonzero(fine_mask))
            corner_states = accumulate_window_hits(
                obj=corner_entry,
                expanded_bbox=expanded_bbox,
                points_cam_current=frame_points,
                projected_uv=frame_uv,
                frame_stamp=frame_stamp,
                corner_states=corner_states,
                params=params,
            )
        if corner_entries_with_bbox and all(
            corner_states.get(_corner_key(corner_entry), _empty_state()).frozen
            for corner_entry, _ in corner_entries_with_bbox
        ):
            stats["stop_reason"] = "all_corners_frozen"
            break

    for corner_entry, _ in corner_entries_with_bbox:
        key = _corner_key(corner_entry)
        state = corner_states.get(key, _empty_state())
        corners[key] = None
        if corner_ready(state, params):
            support_points, support_uv, support_stamps = _robust_corner_support(
                state.points_cam_current,
                state.uv_hits,
                state._point_frame_stamps,
            )
            if support_points.shape[0] > 0:
                support_frames = state.support_frames
                if support_stamps:
                    support_frames = len(set(support_stamps))
                support_ready = (
                    int(support_points.shape[0]) >= int(params.corner_target_points)
                    and support_frames >= int(params.corner_target_frames)
                )
                corner_support[key] = _make_state(
                    points_cam_current=support_points,
                    uv_hits=support_uv,
                    support_frames=support_frames,
                    frozen=support_ready,
                    latest_frame_stamp=state.latest_frame_stamp,
                    point_frame_stamps=support_stamps,
                )
                if support_ready:
                    corners[key] = support_points.mean(axis=0).astype(np.float32)

    if stats["frames_seen"] == 0 or stats["points_raw_sum"] == 0:
        stats["stop_reason"] = "no_cloud_in_window"
    elif stats["stop_reason"] != "all_corners_frozen":
        if stats["points_after_bbox_sum"] == 0:
            stats["stop_reason"] = "no_points_after_bbox"
        elif stats["points_after_mask_sum"] == 0:
            stats["stop_reason"] = "no_points_after_mask"
        else:
            stats["stop_reason"] = "window_exhausted"

    stats["solve_ms"] = (time.perf_counter() - start_time) * 1000.0

    return {
        "frames_used": stats["frames_used"],
        "corner_states": corner_states,
        "corner_support": corner_support,
        "corners": corners,
        "stats": stats,
    }
