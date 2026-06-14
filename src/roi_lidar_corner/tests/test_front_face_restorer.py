import pytest

from roi_lidar_corner.front_face_restorer import restore_front_face
from roi_lidar_corner.structure_tracker import FaceTrack


def test_restore_front_face_requires_top_beam_initialization_once() -> None:
    track = FaceTrack()
    track.left_post.initialized = True
    track.left_post.x_state = -0.5
    track.left_post.z_state = 2.0
    track.right_post.initialized = True
    track.right_post.x_state = 0.5
    track.right_post.z_state = 2.1

    solution = restore_front_face(track, width_m=1.0, height_m=2.0)

    assert solution.valid is False
    assert solution.solution_state == "invalid"


def test_restore_front_face_keeps_tracking_after_beam_drop_if_initialized_before() -> None:
    track = FaceTrack()
    track.left_post.initialized = True
    track.left_post.x_state = -0.5
    track.left_post.z_state = 2.0
    track.right_post.initialized = True
    track.right_post.x_state = 0.5
    track.right_post.z_state = 2.1
    track.top_beam.initialized = True
    track.top_beam.y_top_state = -0.2
    track.model_initialized = True

    solution = restore_front_face(track, width_m=1.0, height_m=2.0)

    assert solution.valid is True
    assert solution.solution_state == "tracking"
    assert solution.top_left == (-0.5, -0.2, 2.0)
    assert solution.bottom_left == (-0.5, 1.8, 2.0)


def test_restore_front_face_inverted_camera_places_bottom_below_physical_top() -> None:
    track = FaceTrack()
    track.left_post.initialized = True
    track.left_post.x_state = -0.5
    track.left_post.z_state = 2.0
    track.right_post.initialized = True
    track.right_post.x_state = 0.5
    track.right_post.z_state = 2.1
    track.top_beam.initialized = True
    track.top_beam.y_top_state = 1.8
    track.model_initialized = True

    solution = restore_front_face(track, width_m=1.0, height_m=2.0, structure_semantics="inverted_camera")

    assert solution.valid is True
    assert solution.top_left == (-0.5, 1.8, 2.0)
    assert solution.bottom_left[0] == -0.5
    assert solution.bottom_left[1] == pytest.approx(-0.2)
    assert solution.bottom_left[2] == 2.0
