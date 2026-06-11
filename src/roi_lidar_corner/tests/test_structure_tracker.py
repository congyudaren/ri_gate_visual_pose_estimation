from roi_lidar_corner.structure_tracker import (
    ObservationStrength,
    PostObservation,
    PostState,
    update_post_state,
)


def test_weak_post_observation_does_not_initialize_empty_state() -> None:
    state = PostState()
    obs = PostObservation(
        strength=ObservationStrength.WEAK,
        stamp=1.0,
        x_obs=0.3,
        z_obs=2.0,
        y_visible_min=0.2,
        y_visible_max=0.8,
        front_peak_confidence=0.4,
        top_side_sample_present=False,
        support_count=3,
    )

    next_state = update_post_state(state, obs, history_window_sec=1.0)

    assert next_state.initialized is False
    assert next_state.confidence == 0.0


def test_observed_post_initializes_and_missing_only_decays() -> None:
    state = PostState()
    observed = PostObservation(
        strength=ObservationStrength.OBSERVED,
        stamp=1.0,
        x_obs=0.3,
        z_obs=2.0,
        y_visible_min=0.2,
        y_visible_max=0.8,
        front_peak_confidence=0.9,
        top_side_sample_present=True,
        support_count=12,
    )

    initialized = update_post_state(state, observed, history_window_sec=1.0)
    missing = PostObservation.missing(stamp=1.2)
    decayed = update_post_state(initialized, missing, history_window_sec=1.0)

    assert initialized.initialized is True
    assert initialized.x_state == 0.3
    assert decayed.initialized is True
    assert decayed.lost_age > 0.0


def test_observed_post_depth_jump_is_rejected_after_initialization() -> None:
    state = update_post_state(
        PostState(),
        PostObservation(
            strength=ObservationStrength.OBSERVED,
            stamp=1.0,
            x_obs=0.3,
            z_obs=3.4,
            y_visible_min=0.2,
            y_visible_max=0.8,
            front_peak_confidence=1.0,
            top_side_sample_present=True,
            support_count=30,
        ),
        history_window_sec=1.0,
    )

    jumped = update_post_state(
        state,
        PostObservation(
            strength=ObservationStrength.OBSERVED,
            stamp=1.5,
            x_obs=0.86,
            z_obs=9.06,
            y_visible_min=0.2,
            y_visible_max=0.8,
            front_peak_confidence=1.0,
            top_side_sample_present=True,
            support_count=37,
        ),
        history_window_sec=1.0,
    )

    assert jumped.initialized is True
    assert jumped.x_state == 0.3
    assert jumped.z_state == 3.4
    assert jumped.confidence < state.confidence
    assert jumped.lost_age > 0.0
