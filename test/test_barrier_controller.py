import unittest

from smart_parking_barrier.barrier_controller import DebouncedOccupancy


class TestDebouncedOccupancy(unittest.TestCase):
    def test_debounce_rejects_short_sensor_noise(self) -> None:
        detector = DebouncedOccupancy(debounce_duration=0.2)
        self.assertFalse(detector.update(True, 0.0))
        self.assertFalse(detector.update(False, 0.1))
        self.assertFalse(detector.update(True, 0.15))
        self.assertTrue(detector.update(True, 0.36))
        self.assertTrue(detector.update(False, 0.40))
        self.assertFalse(detector.update(False, 0.61))
