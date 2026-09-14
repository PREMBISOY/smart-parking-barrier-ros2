"""Small observability node that reports an end-to-end successful cycle."""

from __future__ import annotations


import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimulationMonitor(Node):
    def __init__(self) -> None:
        super().__init__('simulation_monitor')
        self.seen: list[str] = []
        self.complete = False
        self.create_subscription(String, '/parking/barrier_state', self._on_state, 10)

    def _on_state(self, message: String) -> None:
        if not self.seen or self.seen[-1] != message.data:
            self.seen.append(message.data)
            self.get_logger().info(f'Observed {message.data}')
        expected = ['OPENING', 'OPEN', 'VEHICLE_PASSING', 'CLOSING', 'CLOSED']
        if not self.complete and all(state in self.seen for state in expected):
            self.complete = True
            self.get_logger().info('SMART PARKING BARRIER TEST: COMPLETE CYCLE PASS')


def main() -> None:
    rclpy.init()
    node = SimulationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
