"""Pure, deterministic safety state machine for the parking barrier."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class BarrierState(str, Enum):
    CLOSED = 'CLOSED'
    OPENING = 'OPENING'
    OPEN = 'OPEN'
    PASSING = 'VEHICLE_PASSING'
    CLOSING = 'CLOSING'
    FAULT = 'FAULT'


@dataclass(frozen=True)
class BarrierParameters:
    open_angle: float = 1.57
    closed_angle: float = 0.0
    angle_tolerance: float = 0.08
    opening_timeout: float = 5.0
    closing_timeout: float = 5.0
    clearance_delay: float = 1.0


class BarrierStateMachine:
    """Safety-first state machine driven only by sensed occupancy and joint feedback."""

    def __init__(self, parameters: BarrierParameters | None = None) -> None:
        self.parameters = parameters or BarrierParameters()
        self.state = BarrierState.CLOSED
        self._entered_at = 0.0
        self._passage_clear_since: float | None = None

    @property
    def target_angle(self) -> float:
        return self.parameters.open_angle if self.state in {
            BarrierState.OPENING, BarrierState.OPEN, BarrierState.PASSING, BarrierState.FAULT
        } else self.parameters.closed_angle

    def _transition(self, state: BarrierState, now: float) -> tuple[BarrierState, BarrierState] | None:
        if self.state == state:
            return None
        previous = self.state
        self.state = state
        self._entered_at = now
        self._passage_clear_since = None
        return previous, state

    def update(
        self,
        now: float,
        approach_present: bool,
        passage_present: bool,
        joint_angle: float,
    ) -> tuple[BarrierState, BarrierState] | None:
        """Advance one step and return a transition, if one occurred.

        A passage detection always takes priority over closing.  A timeout enters
        ``FAULT`` with an open target, which is the safe physical configuration.
        """
        p = self.parameters
        if self.state == BarrierState.CLOSED:
            if approach_present:
                return self._transition(BarrierState.OPENING, now)
        elif self.state == BarrierState.OPENING:
            if joint_angle >= p.open_angle - p.angle_tolerance:
                return self._transition(BarrierState.OPEN, now)
            if now - self._entered_at > p.opening_timeout:
                return self._transition(BarrierState.FAULT, now)
        elif self.state == BarrierState.OPEN:
            if passage_present:
                return self._transition(BarrierState.PASSING, now)
        elif self.state == BarrierState.PASSING:
            if passage_present:
                self._passage_clear_since = None
            elif self._passage_clear_since is None:
                self._passage_clear_since = now
            elif now - self._passage_clear_since >= p.clearance_delay:
                return self._transition(BarrierState.CLOSING, now)
        elif self.state == BarrierState.CLOSING:
            if approach_present or passage_present:
                return self._transition(BarrierState.OPENING, now)
            if joint_angle <= p.closed_angle + p.angle_tolerance:
                return self._transition(BarrierState.CLOSED, now)
            if now - self._entered_at > p.closing_timeout:
                return self._transition(BarrierState.FAULT, now)
        return None
