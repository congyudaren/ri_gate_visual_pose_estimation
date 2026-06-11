from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class FilteredStructurePoints:
    uv: np.ndarray
    xyz_cam: np.ndarray
    points_raw: int
    points_kept: int
    points_rear_rejected: int
    front_peak_depth: float | None


def _nearest_hist_peak(depth: np.ndarray, bin_size: float) -> float | None:
    if depth.size == 0:
        return None
    bins = np.floor(depth / float(bin_size)).astype(np.int64)
    unique_bins, counts = np.unique(bins, return_counts=True)
    best_index = np.lexsort((unique_bins, -counts))[0]
    return float((unique_bins[best_index] + 0.5) * float(bin_size))


def filter_structure_points(
    *,
    uv: np.ndarray,
    xyz_cam: np.ndarray,
    front_bin_size: float,
    front_keep_tolerance: float,
    rear_gap_m: float,
    rear_gap_tolerance: float,
) -> FilteredStructurePoints:
    xyz = np.asarray(xyz_cam, dtype=np.float32).reshape(-1, 3)
    uv = np.asarray(uv, dtype=np.float32).reshape(-1, 2)
    depth = xyz[:, 2]
    peak = _nearest_hist_peak(depth, bin_size=front_bin_size)
    if peak is None:
        return FilteredStructurePoints(uv[:0], xyz[:0], 0, 0, 0, None)

    front_mask = np.abs(depth - peak) <= float(front_keep_tolerance)
    rear_mask = np.abs(depth - (peak + float(rear_gap_m))) <= float(rear_gap_tolerance)
    keep_mask = front_mask & ~rear_mask
    return FilteredStructurePoints(
        uv=uv[keep_mask],
        xyz_cam=xyz[keep_mask],
        points_raw=int(depth.shape[0]),
        points_kept=int(np.count_nonzero(keep_mask)),
        points_rear_rejected=int(np.count_nonzero(rear_mask)),
        front_peak_depth=float(peak),
    )
