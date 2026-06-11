from __future__ import annotations

from typing import Optional, Sequence, Tuple

import cv2
import numpy as np


def line_intersection(
    line1: Sequence[float],
    line2: Sequence[float],
) -> Optional[Tuple[float, float]]:
    x1, y1, x2, y2 = line1
    x3, y3, x4, y4 = line2

    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < 1e-6:
        return None

    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
    x = x1 + t * (x2 - x1)
    y = y1 + t * (y2 - y1)
    return (float(x), float(y))


def resolve_border_size(roi_height: int, roi_width: int, border_size: Optional[int], border_ratio: float) -> int:
    if border_size is not None:
        return max(0, int(border_size))
    return max(1, int(round(min(roi_height, roi_width) * float(border_ratio))))


def refine_corners_inside_bbox(
    rgb_image: np.ndarray,
    bbox: Tuple[float, float, float, float],
    canny_low: int = 50,
    canny_high: int = 200,
    hough_threshold: int = 50,
    min_line_length: int = 50,
    max_line_gap: int = 50,
    blur_kernel_size: int = 5,
    border_size: Optional[int] = None,
    border_ratio: float = 0.15,
) -> Sequence[Tuple[float, float]]:
    x1, y1, x2, y2 = map(int, bbox)

    roi = rgb_image[y1:y2, x1:x2]
    if roi.size == 0:
        raise ValueError(f"empty ROI for bbox={bbox}")

    if blur_kernel_size % 2 == 0:
        blur_kernel_size += 1
    blur_kernel_size = max(1, blur_kernel_size)

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (blur_kernel_size, blur_kernel_size), 0)
    edges = cv2.Canny(blur, canny_low, canny_high)

    roi_height, roi_width = edges.shape
    border_px = resolve_border_size(roi_height, roi_width, border_size, border_ratio)
    mask = np.ones_like(edges, dtype=np.uint8) * 255
    if border_px * 2 < roi_height and border_px * 2 < roi_width:
        mask[border_px:roi_height - border_px, border_px:roi_width - border_px] = 0
    edges = cv2.bitwise_and(edges, mask)

    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180.0,
        threshold=int(hough_threshold),
        minLineLength=int(min_line_length),
        maxLineGap=int(max_line_gap),
    )

    if lines is not None:
        horizontal_lines = []
        vertical_lines = []
        for line in lines:
            x1_l, y1_l, x2_l, y2_l = line[0]
            dx = abs(x2_l - x1_l)
            dy = abs(y2_l - y1_l)
            angle = np.arctan2(dy, dx) * 180 / np.pi
            if angle < 30:
                horizontal_lines.append(line[0])
            elif angle > 60:
                vertical_lines.append(line[0])

        if len(horizontal_lines) >= 2 and len(vertical_lines) >= 2:
            horizontal_lines.sort(key=lambda line: min(line[1], line[3]))
            vertical_lines.sort(key=lambda line: min(line[0], line[2]))

            top_line = horizontal_lines[0]
            bottom_line = horizontal_lines[-1]
            left_line = vertical_lines[0]
            right_line = vertical_lines[-1]

            tl = line_intersection(top_line, left_line)
            tr = line_intersection(top_line, right_line)
            bl = line_intersection(bottom_line, left_line)
            br = line_intersection(bottom_line, right_line)

            if all(value is not None for value in (tl, tr, bl, br)):
                return [
                    (float(tl[0] + x1), float(tl[1] + y1)),
                    (float(tr[0] + x1), float(tr[1] + y1)),
                    (float(bl[0] + x1), float(bl[1] + y1)),
                    (float(br[0] + x1), float(br[1] + y1)),
                ]

    return [
        (float(x1), float(y1)),
        (float(x2), float(y1)),
        (float(x1), float(y2)),
        (float(x2), float(y2)),
    ]
