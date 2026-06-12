from __future__ import annotations

from typing import Mapping

import cv2
import numpy as np


def build_structure_lines(
    corners: Mapping[str, tuple[float, float]],
) -> dict[str, tuple[tuple[float, float], tuple[float, float]]]:
    return {
        "top_beam": (tuple(corners["TL"]), tuple(corners["TR"])),
        "left_post": (tuple(corners["TL"]), tuple(corners["BL"])),
        "right_post": (tuple(corners["TR"]), tuple(corners["BR"])),
    }


def dilate_line_mask(
    image_shape: tuple[int, int],
    start: tuple[float, float],
    end: tuple[float, float],
    half_width: int,
) -> np.ndarray:
    mask = np.zeros(image_shape, dtype=np.uint8)
    start_xy = (int(round(start[0])), int(round(start[1])))
    end_xy = (int(round(end[0])), int(round(end[1])))
    thickness = max(1, int(half_width) * 2 + 1)
    cv2.line(mask, start_xy, end_xy, 255, thickness=thickness)
    return mask
