from __future__ import annotations

import os
from array import array
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np


DEFAULT_IMAGE_NAME = "lidar_projection_exposure_20260423_220029_0p5s.png"
DEFAULT_POINTS_NAME = "lidar_projection_exposure_20260423_221950_20s_points.npz"


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class FixtureAssets:
    image_path: Path
    points_path: Path


@dataclass(frozen=True)
class FixtureData:
    image_bgr: np.ndarray
    uv: np.ndarray
    depth: np.ndarray
    stamp: np.ndarray


DEFAULT_INTRINSICS = CameraIntrinsics(
    width=640,
    height=480,
    fx=603.7161865234375,
    fy=603.680908203125,
    cx=317.5208740234375,
    cy=248.23284912109375,
)

DEFAULT_CAMERA_EXTRINSIC_T = np.array([0.049, 0.29671, 0.01812], dtype=np.float32)
DEFAULT_CAMERA_EXTRINSIC_R = np.array(
    [
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float32,
)


def _candidate_roots() -> Iterable[Path]:
    env_root = os.environ.get("ROI_LIDAR_FIXTURE_ROOT")
    if env_root:
        yield Path(env_root).expanduser()
    yield Path.cwd()
    for parent in Path(__file__).resolve().parents:
        yield parent


def default_assets() -> FixtureAssets:
    relative_dir = Path("analysis_artifacts") / "exposure_from_nx"
    for root in _candidate_roots():
        image_path = root / relative_dir / DEFAULT_IMAGE_NAME
        points_path = root / relative_dir / DEFAULT_POINTS_NAME
        if image_path.exists() and points_path.exists():
            return FixtureAssets(image_path=image_path, points_path=points_path)
    fallback = Path.cwd() / relative_dir
    return FixtureAssets(image_path=fallback / DEFAULT_IMAGE_NAME, points_path=fallback / DEFAULT_POINTS_NAME)


def load_fixture_assets(assets: FixtureAssets) -> FixtureData:
    image = cv2.imread(str(assets.image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(f"failed to read fixture image: {assets.image_path}")

    payload = np.load(assets.points_path)
    uv = np.asarray(payload["uv"], dtype=np.float32).reshape(-1, 2)
    depth = np.asarray(payload["depth"], dtype=np.float32).reshape(-1)
    stamp = np.asarray(payload["stamp"], dtype=np.float64).reshape(-1)
    if uv.shape[0] != depth.shape[0] or uv.shape[0] != stamp.shape[0]:
        raise ValueError(f"fixture point row mismatch: uv={uv.shape[0]} depth={depth.shape[0]} stamp={stamp.shape[0]}")
    return FixtureData(image_bgr=image, uv=uv, depth=depth, stamp=stamp)


def rotate_uv_180(uv: np.ndarray, *, width: int, height: int) -> np.ndarray:
    points = np.asarray(uv, dtype=np.float32).reshape(-1, 2)
    rotated = points.copy()
    rotated[:, 0] = float(width - 1) - rotated[:, 0]
    rotated[:, 1] = float(height - 1) - rotated[:, 1]
    return rotated


def uv_depth_to_xyz_cam(*, uv: np.ndarray, depth: np.ndarray, intrinsics: CameraIntrinsics) -> np.ndarray:
    points = np.asarray(uv, dtype=np.float32).reshape(-1, 2)
    z = np.asarray(depth, dtype=np.float32).reshape(-1)
    if points.shape[0] != z.shape[0]:
        raise ValueError(f"uv/depth row mismatch: uv={points.shape[0]} depth={z.shape[0]}")
    x = (points[:, 0] - float(intrinsics.cx)) * z / float(intrinsics.fx)
    y = (points[:, 1] - float(intrinsics.cy)) * z / float(intrinsics.fy)
    return np.column_stack((x, y, z)).astype(np.float32, copy=False)


def camera_points_to_body(*, xyz_cam: np.ndarray, t_cb: np.ndarray, r_cb: np.ndarray) -> np.ndarray:
    points = np.asarray(xyz_cam, dtype=np.float32).reshape(-1, 3)
    t = np.asarray(t_cb, dtype=np.float32).reshape(1, 3)
    r = np.asarray(r_cb, dtype=np.float32).reshape(3, 3)
    return (points @ r.T + t).astype(np.float32, copy=False)


def points_to_xyz32_bytes(points: np.ndarray) -> tuple[array, int]:
    xyz = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    finite = np.isfinite(xyz).all(axis=1)
    xyz = np.ascontiguousarray(xyz[finite], dtype="<f4")
    return array("B", xyz.tobytes()), int(xyz.shape[0])
