from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class FrontFaceSolution:
    valid: bool
    solution_state: str
    top_left: tuple[float, float, float]
    top_right: tuple[float, float, float]
    bottom_left: tuple[float, float, float]
    bottom_right: tuple[float, float, float]
    top_source: str
    tracking_confidence: float = 0.0


def _invalid_solution() -> FrontFaceSolution:
    zero = (0.0, 0.0, 0.0)
    return FrontFaceSolution(
        valid=False,
        solution_state="invalid",
        top_left=zero,
        top_right=zero,
        bottom_left=zero,
        bottom_right=zero,
        top_source="none",
        tracking_confidence=0.0,
    )


def restore_front_face(track, width_m: float, height_m: float) -> FrontFaceSolution:
    if not track.left_post.initialized or not track.right_post.initialized:
        return _invalid_solution()
    if not track.model_initialized and not track.top_beam.initialized:
        return _invalid_solution()

    y_top = float(track.top_beam.y_top_state)
    top_left = (float(track.left_post.x_state), y_top, float(track.left_post.z_state))
    top_right = (float(track.right_post.x_state), y_top, float(track.right_post.z_state))
    bottom_left = (top_left[0], top_left[1] + float(height_m), top_left[2])
    bottom_right = (top_right[0], top_right[1] + float(height_m), top_right[2])
    confidence = min(
        1.0,
        max(0.0, (float(track.left_post.confidence) + float(track.right_post.confidence)) * 0.5),
    )
    return FrontFaceSolution(
        valid=True,
        solution_state="tracking",
        top_left=top_left,
        top_right=top_right,
        bottom_left=bottom_left,
        bottom_right=bottom_right,
        top_source="beam" if track.top_beam.initialized else "history",
        tracking_confidence=confidence,
    )
