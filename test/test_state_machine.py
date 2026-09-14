import unittest

from smart_parking_barrier.state_machine import BarrierParameters, BarrierState, BarrierStateMachine


def machine() -> BarrierStateMachine:
    return BarrierStateMachine(BarrierParameters(clearance_delay=0.5, opening_timeout=2.0, closing_timeout=2.0))


class TestBarrierStateMachine(unittest.TestCase):
    def test_complete_safe_cycle(self) -> None:
        sm = machine()
        self.assertEqual(sm.state, BarrierState.CLOSED)
        self.assertEqual(sm.update(0.1, True, False, 0.0)[1], BarrierState.OPENING)
        self.assertEqual(sm.update(0.2, True, False, 1.57)[1], BarrierState.OPEN)
        self.assertEqual(sm.update(0.3, False, True, 1.57)[1], BarrierState.PASSING)
        self.assertIsNone(sm.update(0.5, False, False, 1.57))
        self.assertEqual(sm.state, BarrierState.PASSING)
        self.assertEqual(sm.update(1.1, False, False, 1.57)[1], BarrierState.CLOSING)
        self.assertEqual(sm.update(1.2, False, False, 0.0)[1], BarrierState.CLOSED)


    def test_barrier_never_closes_while_passage_is_occupied(self) -> None:
        sm = machine()
        sm.update(0.0, True, False, 0.0)
        sm.update(0.1, True, False, 1.57)
        sm.update(0.2, False, True, 1.57)
        for now in (0.8, 1.4, 2.0):
            self.assertIsNone(sm.update(now, False, True, 1.57))
            self.assertEqual(sm.state, BarrierState.PASSING)


    def test_closing_reopens_for_new_vehicle(self) -> None:
        sm = machine()
        sm.update(0.0, True, False, 0.0)
        sm.update(0.1, True, False, 1.57)
        sm.update(0.2, False, True, 1.57)
        sm.update(0.3, False, False, 1.57)
        sm.update(0.9, False, False, 1.57)
        self.assertEqual(sm.state, BarrierState.CLOSING)
        self.assertEqual(sm.update(1.0, True, False, 1.2)[1], BarrierState.OPENING)


    def test_motion_timeout_fails_open(self) -> None:
        sm = machine()
        sm.update(0.0, True, False, 0.0)
        self.assertEqual(sm.update(2.1, True, False, 0.0)[1], BarrierState.FAULT)
        self.assertEqual(sm.target_angle, sm.parameters.open_angle)
