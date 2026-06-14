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


def _bottom_y_from_top(y_top: float, height_m: float, structure_semantics: str) -> float:
    semantics = str(structure_semantics).strip().lower()
    if semantics == "normal":
        return y_top + float(height_m)
    if semantics == "inverted_camera":
        return y_top - float(height_m)
    raise ValueError(f"unsupported structure_semantics: {structure_semantics!r}")


def restore_front_face(
    track,
    width_m: float,
    height_m: float,
    structure_semantics: str = "normal",
) -> FrontFaceSolution:
    if not track.left_post.initialized or not track.right_post.initialized:
        return _invalid_solution()
    if not track.model_initialized and not track.top_beam.initialized:
        return _invalid_solution()

    y_top = float(track.top_beam.y_top_state)
    top_left = (float(track.left_post.x_state), y_top, float(track.left_post.z_state))
    top_right = (float(track.right_post.x_state), y_top, float(track.right_post.z_state))
    y_bottom = _bottom_y_from_top(y_top, height_m, structure_semantics)
    bottom_left = (top_left[0], y_bottom, top_left[2])
    bottom_right = (top_right[0], y_bottom, top_right[2])
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
