"""Sensor-driven ROS 2 controller for a Gazebo parking barrier."""

from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, String

from .state_machine import BarrierParameters, BarrierState, BarrierStateMachine


@dataclass
class DebouncedOccupancy:
    """Turns noisy lidar measurements into a stable occupied / clear signal."""

    debounce_duration: float
    value: bool = False
    _candidate: bool = False
    _changed_at: float = 0.0

    def update(self, raw_value: bool, now: float) -> bool:
        if raw_value != self._candidate:
            self._candidate = raw_value
            self._changed_at = now
        if self.value != self._candidate and now - self._changed_at >= self.debounce_duration:
            self.value = self._candidate
        return self.value


class BarrierController(Node):
    """Consumes two real Gazebo lidar feeds and commands a revolute joint."""

    def __init__(self) -> None:
        super().__init__('barrier_controller')
        for name, default in {
            'approach_threshold': 0.8, 'passage_threshold': 0.8,
            'sensor_timeout': 1.0, 'open_angle': 1.57, 'closed_angle': 0.0,
            'opening_timeout': 5.0, 'closing_timeout': 5.0,
            'debounce_duration': 0.15, 'clearance_delay': 1.0,
            'barrier_motion_duration': 0.9,
        }.items():
            self.declare_parameter(name, default)

        self.approach_threshold = float(self.get_parameter('approach_threshold').value)
        self.passage_threshold = float(self.get_parameter('passage_threshold').value)
        self.sensor_timeout = float(self.get_parameter('sensor_timeout').value)
        self.motion_duration = float(self.get_parameter('barrier_motion_duration').value)
        params = BarrierParameters(
            open_angle=float(self.get_parameter('open_angle').value),
            closed_angle=float(self.get_parameter('closed_angle').value),
            opening_timeout=float(self.get_parameter('opening_timeout').value),
            closing_timeout=float(self.get_parameter('closing_timeout').value),
            clearance_delay=float(self.get_parameter('clearance_delay').value),
        )
        self.machine = BarrierStateMachine(params)
        debounce = float(self.get_parameter('debounce_duration').value)
        self.approach = DebouncedOccupancy(debounce)
        self.passage = DebouncedOccupancy(debounce)
        self._raw_approach = False
        self._raw_passage = False
        self._reported_approach: bool | None = None
        self._reported_passage: bool | None = None
        self._approach_at = -math.inf
        self._passage_at = -math.inf
        self._last_tick = 0.0
        self._estimated_angle = params.closed_angle

        self.create_subscription(LaserScan, '/parking/approach_scan', self._approach_scan, 10)
        self.create_subscription(LaserScan, '/parking/passage_scan', self._passage_scan, 10)
        self.command_pub = self.create_publisher(Float64, '/parking/barrier_command', 10)
        self.state_pub = self.create_publisher(String, '/parking/barrier_state', 10)
        self.status_pub = self.create_publisher(String, '/parking/system_status', 10)
        self.create_timer(0.05, self._tick)
        self.get_logger().info('Barrier controller ready; waiting for approach lidar.')

    @staticmethod
    def _minimum_range(scan: LaserScan) -> float:
        finite = [r for r in scan.ranges if math.isfinite(r)]
        return min(finite) if finite else math.inf

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _approach_scan(self, scan: LaserScan) -> None:
        minimum = self._minimum_range(scan)
        self._raw_approach = scan.range_min < minimum < self.approach_threshold
        self._approach_at = self._now()
        if self._reported_approach != self._raw_approach:
            self._reported_approach = self._raw_approach
            self.get_logger().info(f'Approach lidar: occupied={self._raw_approach}, min_range={minimum:.2f} m')

    def _passage_scan(self, scan: LaserScan) -> None:
        minimum = self._minimum_range(scan)
        self._raw_passage = scan.range_min < minimum < self.passage_threshold
        self._passage_at = self._now()
        if self._reported_passage != self._raw_passage:
            self._reported_passage = self._raw_passage
            self.get_logger().info(f'Passage lidar: occupied={self._raw_passage}, min_range={minimum:.2f} m')

    def _tick(self) -> None:
        now = self._now()
        if now <= 0.0:  # Gazebo has not published /clock yet.
            return
        dt = max(0.0, now - self._last_tick) if self._last_tick else 0.0
        self._last_tick = now
        approach_raw = self._raw_approach and now - self._approach_at <= self.sensor_timeout
        passage_raw = self._raw_passage and now - self._passage_at <= self.sensor_timeout
        approach = self.approach.update(approach_raw, now)
        passage = self.passage.update(passage_raw, now)

        # The Gazebo JointPositionController receives the same target.  This
        # bounded local estimate supplies deterministic state timing without
        # using vehicle pose or model coordinates for sensing decisions.
        target = self.machine.target_angle
        rate = abs(self.machine.parameters.open_angle - self.machine.parameters.closed_angle) / self.motion_duration
        if self._estimated_angle < target:
            self._estimated_angle = min(target, self._estimated_angle + rate * dt)
        else:
            self._estimated_angle = max(target, self._estimated_angle - rate * dt)

        transition = self.machine.update(now, approach, passage, self._estimated_angle)
        if transition:
            old, new = transition
            self.get_logger().info(f'State: {old.value} -> {new.value}')
            self.status_pub.publish(String(data=f'{new.value}; approach={approach}; passage={passage}'))

        self.command_pub.publish(Float64(data=self.machine.target_angle))
        self.state_pub.publish(String(data=self.machine.state.value))


def main() -> None:
    rclpy.init()
    node = BarrierController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
