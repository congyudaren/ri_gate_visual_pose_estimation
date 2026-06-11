from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Deque


class ObservationStrength(str, Enum):
    OBSERVED = "observed"
    WEAK = "weak"
    MISSING = "missing"


@dataclass(frozen=True)
class PostObservation:
    strength: ObservationStrength
    stamp: float
    x_obs: float | None = None
    z_obs: float | None = None
    y_visible_min: float | None = None
    y_visible_max: float | None = None
    front_peak_confidence: float = 0.0
    top_side_sample_present: bool = False
    support_count: int = 0

    @classmethod
    def missing(cls, stamp: float) -> "PostObservation":
        return cls(strength=ObservationStrength.MISSING, stamp=stamp)


@dataclass
class PostState:
    observations: Deque[PostObservation] = field(default_factory=deque)
    initialized: bool = False
    x_state: float = 0.0
    z_state: float = 0.0
    confidence: float = 0.0
    freshness: float = 0.0
    lost_age: float = 0.0


@dataclass
class TopBeamState:
    initialized: bool = False
    y_top_state: float = 0.0
    confidence: float = 0.0


@dataclass
class FaceTrack:
    left_post: PostState = field(default_factory=PostState)
    right_post: PostState = field(default_factory=PostState)
    top_beam: TopBeamState = field(default_factory=TopBeamState)
    model_initialized: bool = False


def update_post_state(
    state: PostState,
    obs: PostObservation,
    history_window_sec: float,
    max_z_jump_m: float = 0.8,
) -> PostState:
    next_state = PostState(
        observations=deque(state.observations),
        initialized=state.initialized,
        x_state=state.x_state,
        z_state=state.z_state,
        confidence=state.confidence,
        freshness=state.freshness,
        lost_age=state.lost_age,
    )

    def decay_missing(missing_stamp: float) -> PostState:
        previous_stamp = next_state.observations[-1].stamp if next_state.observations else float(missing_stamp)
        next_state.lost_age += max(0.0, float(missing_stamp) - float(previous_stamp))
        next_state.freshness = max(0.0, next_state.freshness - 0.2)
        next_state.confidence = max(0.0, next_state.confidence - 0.1)
        return next_state

    if obs.strength == ObservationStrength.MISSING:
        return decay_missing(obs.stamp)

    if (
        obs.strength == ObservationStrength.OBSERVED
        and next_state.initialized
        and float(max_z_jump_m) > 0.0
        and obs.z_obs is not None
        and abs(float(obs.z_obs) - float(next_state.z_state)) > float(max_z_jump_m)
    ):
        return decay_missing(obs.stamp)

    next_state.observations.append(obs)
    while next_state.observations and obs.stamp - next_state.observations[0].stamp > history_window_sec:
        next_state.observations.popleft()

    if obs.strength == ObservationStrength.OBSERVED:
        next_state.initialized = True
        next_state.x_state = float(obs.x_obs)
        next_state.z_state = float(obs.z_obs)
        next_state.confidence = min(1.0, max(next_state.confidence, obs.front_peak_confidence))
        next_state.freshness = 1.0
        next_state.lost_age = 0.0
    return next_state
