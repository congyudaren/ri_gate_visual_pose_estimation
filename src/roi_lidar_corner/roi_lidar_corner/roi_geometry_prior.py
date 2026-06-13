from __future__ import annotations

from dataclasses import dataclass
from math import atan2, degrees, hypot
from statistics import median
from typing import Mapping, Optional, Sequence, Tuple


Point = Tuple[float, float]
Line = Tuple[Point, Point]


@dataclass(frozen=True)
class GeometryPriorConfig:
    min_post_bbox_height_ratio: float = 0.45
    expected_top_post_ratio: float = 0.5
    top_post_ratio_tolerance: float = 0.25
    border_ratio_tolerance_scale: float = 2.0
    border_post_height_ratio_scale: float = 0.5
    border_post_similarity_ratio_scale: float = 0.5
    top_border_offset_ratio_scale: float = 2.0
    border_relax_px: float = 8.0
    max_vertical_angle_error_deg: float = 30.0
    max_horizontal_angle_error_deg: float = 30.0
    max_top_offset_bbox_ratio: float = 0.35
    min_post_similarity_ratio: float = 0.45


@dataclass(frozen=True)
class TemporalPriorConfig:
    max_line_jump_px: float = 80.0


@dataclass(frozen=True)
class StructureCandidate:
    lines: Mapping[str, Line]


@dataclass(frozen=True)
class ValidationResult:
    valid: bool
    reason: str


def build_bbox_corners(bbox: Sequence[float]) -> list[Point]:
    x1, y1, x2, y2 = [float(value) for value in bbox]
    return [(x1, y1), (x2, y1), (x1, y2), (x2, y2)]


def line_length(line: Line) -> float:
    return float(hypot(float(line[1][0]) - float(line[0][0]), float(line[1][1]) - float(line[0][1])))


def line_midpoint(line: Line) -> Point:
    return ((float(line[0][0]) + float(line[1][0])) * 0.5, (float(line[0][1]) + float(line[1][1])) * 0.5)


def _angle_deg(line: Line) -> float:
    dx = abs(float(line[1][0]) - float(line[0][0]))
    dy = abs(float(line[1][1]) - float(line[0][1]))
    return float(degrees(atan2(dy, dx)))


def _border_clip_flags(
    bbox: Sequence[float],
    image_shape: Tuple[int, int],
    border_relax_px: float,
) -> Tuple[bool, bool, bool, bool]:
    x1, y1, x2, y2 = [float(value) for value in bbox]
    height, width = int(image_shape[0]), int(image_shape[1])
    margin = float(border_relax_px)
    return (
        x1 <= margin,
        y1 <= margin,
        x2 >= float(width) - margin,
        y2 >= float(height) - margin,
    )


def validate_structure_candidate(
    candidate: StructureCandidate,
    bbox: Sequence[float],
    image_shape: Tuple[int, int],
    config: GeometryPriorConfig,
) -> ValidationResult:
    required = ("left_post", "right_post", "top_beam")
    if any(name not in candidate.lines for name in required):
        return ValidationResult(False, "missing_structure")

    left = candidate.lines["left_post"]
    right = candidate.lines["right_post"]
    top = candidate.lines["top_beam"]
    _x1, y1, _x2, y2 = [float(value) for value in bbox]
    bbox_h = max(1.0, y2 - y1)
    left_clipped, top_clipped, right_clipped, bottom_clipped = _border_clip_flags(
        bbox,
        image_shape,
        config.border_relax_px,
    )
    border_clipped = left_clipped or top_clipped or right_clipped or bottom_clipped
    vertical_clipped = top_clipped or bottom_clipped

    if abs(90.0 - _angle_deg(left)) > config.max_vertical_angle_error_deg:
        return ValidationResult(False, "left_not_vertical")
    if abs(90.0 - _angle_deg(right)) > config.max_vertical_angle_error_deg:
        return ValidationResult(False, "right_not_vertical")
    if _angle_deg(top) > config.max_horizontal_angle_error_deg:
        return ValidationResult(False, "top_not_horizontal")

    left_len = line_length(left)
    right_len = line_length(right)
    top_len = line_length(top)
    post_height_scale = float(config.border_post_height_ratio_scale) if vertical_clipped else 1.0
    min_post_len = float(config.min_post_bbox_height_ratio) * bbox_h * post_height_scale
    if left_len < min_post_len or right_len < min_post_len:
        return ValidationResult(False, "post_too_short")

    post_len = max(1.0, float(median([left_len, right_len])))
    shorter = min(left_len, right_len)
    longer = max(left_len, right_len)
    similarity_scale = float(config.border_post_similarity_ratio_scale) if vertical_clipped else 1.0
    min_similarity_ratio = float(config.min_post_similarity_ratio) * similarity_scale
    if shorter / max(1.0, longer) < min_similarity_ratio:
        return ValidationResult(False, "post_length_mismatch")

    ratio = top_len / post_len
    tolerance_scale = float(config.border_ratio_tolerance_scale) if border_clipped else 1.0
    tolerance = float(config.top_post_ratio_tolerance) * tolerance_scale
    expected = float(config.expected_top_post_ratio)
    if abs(ratio - expected) > tolerance:
        return ValidationResult(False, "top_post_ratio")

    _top_mx, top_my = line_midpoint(top)
    top_offset_scale = float(config.top_border_offset_ratio_scale) if top_clipped else 1.0
    max_top_offset = bbox_h * float(config.max_top_offset_bbox_ratio) * top_offset_scale
    if top_my > y1 + max_top_offset:
        return ValidationResult(False, "top_not_near_bbox_top")

    return ValidationResult(True, "accepted")


def temporal_jump_reason(
    previous: Optional[StructureCandidate],
    current: StructureCandidate,
    config: TemporalPriorConfig,
) -> Optional[str]:
    if previous is None:
        return None
    for name, current_line in current.lines.items():
        previous_line = previous.lines.get(name)
        if previous_line is None:
            continue
        prev_mid = line_midpoint(previous_line)
        curr_mid = line_midpoint(current_line)
        midpoint_jump = hypot(curr_mid[0] - prev_mid[0], curr_mid[1] - prev_mid[1])
        length_jump = abs(line_length(current_line) - line_length(previous_line))
        if midpoint_jump > float(config.max_line_jump_px) or length_jump > float(config.max_line_jump_px):
            return f"{name}_temporal_jump"
    return None
