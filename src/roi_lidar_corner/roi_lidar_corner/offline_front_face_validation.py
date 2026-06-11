from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class StaticSceneCase:
    image: np.ndarray
    uv: np.ndarray
    uv_aligned: np.ndarray
    depth: np.ndarray
    stamp: np.ndarray
    intrinsics: CameraIntrinsics | None = None
    xyz_cam: np.ndarray | None = None


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


def load_static_scene_case(
    *,
    image_path: Path,
    npz_path: Path,
    intrinsics: CameraIntrinsics | None = None,
    rotate_180: bool = False,
) -> StaticSceneCase:
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(f"failed to read image: {image_path}")
    payload = np.load(npz_path)
    uv = np.asarray(payload["uv"], dtype=np.float32)
    depth = np.asarray(payload["depth"], dtype=np.float32)
    uv_aligned = rotate_uv_180(uv, width=image.shape[1], height=image.shape[0]) if rotate_180 else uv.copy()
    xyz_cam = uv_depth_to_xyz_cam(uv=uv_aligned, depth=depth, intrinsics=intrinsics) if intrinsics else None
    return StaticSceneCase(
        image=image,
        uv=uv,
        uv_aligned=uv_aligned,
        depth=depth,
        stamp=np.asarray(payload["stamp"], dtype=np.float64),
        intrinsics=intrinsics,
        xyz_cam=xyz_cam,
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--points", type=Path, required=True)
    parser.add_argument("--fx", type=float)
    parser.add_argument("--fy", type=float)
    parser.add_argument("--cx", "--ppx", dest="cx", type=float)
    parser.add_argument("--cy", "--ppy", dest="cy", type=float)
    parser.add_argument("--rotate-180", action="store_true")
    args = parser.parse_args()
    intrinsics = None
    if all(value is not None for value in (args.fx, args.fy, args.cx, args.cy)):
        intrinsics = CameraIntrinsics(
            width=0,
            height=0,
            fx=float(args.fx),
            fy=float(args.fy),
            cx=float(args.cx),
            cy=float(args.cy),
        )
    case = load_static_scene_case(
        image_path=args.image,
        npz_path=args.points,
        intrinsics=intrinsics,
        rotate_180=bool(args.rotate_180),
    )
    xyz_text = f" xyz_cam={case.xyz_cam.shape[0]}" if case.xyz_cam is not None else ""
    print(
        f"image={case.image.shape[1]}x{case.image.shape[0]} "
        f"points={case.uv.shape[0]} depth={case.depth.shape[0]}"
        f" rotate_180={bool(args.rotate_180)}{xyz_text}"
    )


if __name__ == "__main__":
    main()
