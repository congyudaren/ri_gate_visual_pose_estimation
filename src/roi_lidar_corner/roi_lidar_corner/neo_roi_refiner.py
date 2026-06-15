from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class CornerRefinementResult:
    corners: Sequence[Tuple[float, float]]
    source: str
    reason: str


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


def _bbox_corners(x1: int, y1: int, x2: int, y2: int) -> Sequence[Tuple[float, float]]:
    return [
        (float(x1), float(y1)),
        (float(x2), float(y1)),
        (float(x1), float(y2)),
        (float(x2), float(y2)),
    ]


def _is_bbox_border_candidate(
    corners: Sequence[Tuple[float, float]],
    bbox: Tuple[int, int, int, int],
    tolerance_px: float,
) -> bool:
    x1, y1, x2, y2 = bbox
    if len(corners) < 4:
        return True
    tl, tr, bl, br = corners[:4]
    tolerance = max(0.0, float(tolerance_px))
    left_like = abs(float(tl[0]) - x1) <= tolerance and abs(float(bl[0]) - x1) <= tolerance
    right_like = abs(float(tr[0]) - x2) <= tolerance and abs(float(br[0]) - x2) <= tolerance
    top_like = abs(float(tl[1]) - y1) <= tolerance and abs(float(tr[1]) - y1) <= tolerance
    return bool(left_like or right_like or top_like)


def _line_x_mid(line: Sequence[float]) -> float:
    return (float(line[0]) + float(line[2])) * 0.5


def _line_y_min(line: Sequence[float]) -> float:
    return min(float(line[1]), float(line[3]))


def _line_y_max(line: Sequence[float]) -> float:
    return max(float(line[1]), float(line[3]))


def _line_length(line: Sequence[float]) -> float:
    return float(np.hypot(float(line[2]) - float(line[0]), float(line[3]) - float(line[1])))


def _vertical_pair_score(left_line: Sequence[float], right_line: Sequence[float]) -> float:
    left_len = _line_length(left_line)
    right_len = _line_length(right_line)
    shorter_len = min(left_len, right_len)
    longer_len = max(left_len, right_len)
    shared_y = max(
        0.0,
        min(_line_y_max(left_line), _line_y_max(right_line))
        - max(_line_y_min(left_line), _line_y_min(right_line)),
    )
    separation = abs(_line_x_mid(right_line) - _line_x_mid(left_line))
    length_similarity = shorter_len / max(1.0, longer_len)
    return 2.0 * shared_y + 0.7 * shorter_len + 0.5 * separation + 25.0 * length_similarity


def _prefer_outer_side_posts(
    pair: Tuple[Sequence[float], Sequence[float]],
    vertical_lines: Sequence[Sequence[float]],
    roi_height: int,
) -> Tuple[Sequence[float], Sequence[float]]:
    min_side_post_len = max(1.0, float(roi_height) * 0.5)
    left_line, right_line = pair
    left_x = _line_x_mid(left_line)
    right_x = _line_x_mid(right_line)

    left_candidates = [
        line
        for line in vertical_lines
        if _line_x_mid(line) <= left_x and _line_length(line) >= min_side_post_len
    ]
    right_candidates = [
        line
        for line in vertical_lines
        if _line_x_mid(line) >= right_x and _line_length(line) >= min_side_post_len
    ]
    if left_candidates:
        left_line = min(left_candidates, key=_line_x_mid)
    if right_candidates:
        right_line = max(right_candidates, key=_line_x_mid)
    return left_line, right_line


def _select_vertical_pair(
    vertical_lines: Sequence[Sequence[float]],
    roi_height: int,
) -> Tuple[Sequence[float], Sequence[float]]:
    ordered = sorted(vertical_lines, key=lambda line: min(line[0], line[2]))
    current_pair = (ordered[0], ordered[-1])
    best_pair: Optional[Tuple[Sequence[float], Sequence[float]]] = None
    best_score: Optional[float] = None
    for left_index, first in enumerate(vertical_lines):
        for second in vertical_lines[left_index + 1 :]:
            left_line, right_line = (first, second)
            if _line_x_mid(left_line) > _line_x_mid(right_line):
                left_line, right_line = right_line, left_line
            score = _vertical_pair_score(left_line, right_line)
            if best_score is None or score > best_score:
                best_pair = (left_line, right_line)
                best_score = score
    if best_pair is None:
        return current_pair

    current_shorter = min(_line_length(current_pair[0]), _line_length(current_pair[1]))
    best_shorter = min(_line_length(best_pair[0]), _line_length(best_pair[1]))
    current_shared_y = max(
        0.0,
        min(_line_y_max(current_pair[0]), _line_y_max(current_pair[1]))
        - max(_line_y_min(current_pair[0]), _line_y_min(current_pair[1])),
    )
    best_shared_y = max(
        0.0,
        min(_line_y_max(best_pair[0]), _line_y_max(best_pair[1]))
        - max(_line_y_min(best_pair[0]), _line_y_min(best_pair[1])),
    )
    current_separation = abs(_line_x_mid(current_pair[1]) - _line_x_mid(current_pair[0]))
    best_separation = abs(_line_x_mid(best_pair[1]) - _line_x_mid(best_pair[0]))
    if (
        best_shorter >= current_shorter * 1.2
        and best_shared_y >= current_shared_y
        and best_separation >= current_separation * 0.75
    ):
        return _prefer_outer_side_posts(best_pair, vertical_lines, roi_height)
    return _prefer_outer_side_posts(current_pair, vertical_lines, roi_height)


def refine_corners_inside_bbox_with_source(
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
    bbox_border_tolerance_px: float = 3.0,
) -> CornerRefinementResult:
    x1, y1, x2, y2 = map(int, bbox)
    fallback = _bbox_corners(x1, y1, x2, y2)

    roi = rgb_image[y1:y2, x1:x2]
    if roi.size == 0:
        return CornerRefinementResult(corners=fallback, source="bbox_fallback", reason="empty_roi")

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

    if lines is None:
        return CornerRefinementResult(corners=fallback, source="bbox_fallback", reason="no_hough_lines")

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

    if len(horizontal_lines) < 2 and len(vertical_lines) < 2:
        return CornerRefinementResult(corners=fallback, source="bbox_fallback", reason="missing_lines")
    if len(horizontal_lines) < 2:
        return CornerRefinementResult(corners=fallback, source="bbox_fallback", reason="missing_horizontal")
    if len(vertical_lines) < 2:
        return CornerRefinementResult(corners=fallback, source="bbox_fallback", reason="missing_vertical")

    horizontal_lines.sort(key=lambda line: min(line[1], line[3]))

    top_line = horizontal_lines[0]
    bottom_line = horizontal_lines[-1]
    left_line, right_line = _select_vertical_pair(vertical_lines, roi_height)
    top_y = (float(top_line[1]) + float(top_line[3])) * 0.5
    bottom_y = (float(bottom_line[1]) + float(bottom_line[3])) * 0.5
    if abs(bottom_y - top_y) < float(min_line_length):
        return CornerRefinementResult(corners=fallback, source="bbox_fallback", reason="collapsed_horizontal")

    tl = line_intersection(top_line, left_line)
    tr = line_intersection(top_line, right_line)
    bl = line_intersection(bottom_line, left_line)
    br = line_intersection(bottom_line, right_line)

    if not all(value is not None for value in (tl, tr, bl, br)):
        return CornerRefinementResult(corners=fallback, source="bbox_fallback", reason="parallel_lines")

    corners = [
        (float(tl[0] + x1), float(tl[1] + y1)),
        (float(tr[0] + x1), float(tr[1] + y1)),
        (float(bl[0] + x1), float(bl[1] + y1)),
        (float(br[0] + x1), float(br[1] + y1)),
    ]
    if _is_bbox_border_candidate(corners, (x1, y1, x2, y2), bbox_border_tolerance_px):
        return CornerRefinementResult(corners=fallback, source="bbox_fallback", reason="bbox_border_like")
    return CornerRefinementResult(corners=corners, source="corner_refined", reason="ok")


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
    return refine_corners_inside_bbox_with_source(
        rgb_image,
        bbox,
        canny_low=canny_low,
        canny_high=canny_high,
        hough_threshold=hough_threshold,
        min_line_length=min_line_length,
        max_line_gap=max_line_gap,
        blur_kernel_size=blur_kernel_size,
        border_size=border_size,
        border_ratio=border_ratio,
    ).corners
