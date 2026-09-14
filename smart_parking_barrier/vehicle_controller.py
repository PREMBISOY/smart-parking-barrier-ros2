"""Deterministic, autonomous vehicle command source."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


class VehicleController(Node):
    """Begins after a short delay and drives one vehicle through exactly once."""

    def __init__(self) -> None:
        super().__init__('vehicle_controller')
        self.declare_parameter('vehicle_speed', 0.9)
        self.declare_parameter('start_delay', 2.0)
        self.declare_parameter('drive_duration', 13.0)
        self.speed = float(self.get_parameter('vehicle_speed').value)
        self.start_delay = float(self.get_parameter('start_delay').value)
        self.drive_duration = float(self.get_parameter('drive_duration').value)
        self.sim_started_at: float | None = None
        self.started_at: float | None = None
        self.finished = False
        self._last_motion_log = -1.0
        self._last_odom_log = -1.0
        self.create_subscription(String, '/parking/barrier_state', self._state_callback, 10)
        self.pub = self.create_publisher(Twist, '/model/vehicle/cmd_vel', 10)
        self.create_subscription(Odometry, '/parking/vehicle_odom', self._odom_callback, 10)
        self.create_timer(0.05, self._tick)
        self.get_logger().info('Vehicle armed; it begins its approach after the configured delay.')

    def _state_callback(self, message: String) -> None:
        if message.data == 'OPEN':
            self.get_logger().debug('Barrier is open; vehicle may pass safely.')

    def _tick(self) -> None:
        command = Twist()
        now = self.get_clock().now().nanoseconds / 1e9
        if self.sim_started_at is None and now > 0.0:
            self.sim_started_at = now
        if self.started_at is None and self.sim_started_at is not None and now - self.sim_started_at >= self.start_delay:
            self.started_at = now
            self.get_logger().info('Vehicle approaching the gate.')
        if self.started_at is not None and not self.finished:
            if now - self.started_at < self.drive_duration:
                # The vehicle starts at world x=-3.0 while the gate and passage
                # zone are at increasing x. Positive body-x is the physically
                # verified direction toward the gate in this integrated world.
                command.linear.x = self.speed
            else:
                self.finished = True
                self.get_logger().info('Vehicle stopped beyond the barrier.')
        self.pub.publish(command)
        if now > 0.0 and now - self._last_motion_log >= 1.0:
            self._last_motion_log = now
            self.get_logger().info(
                f'sim={now:.1f}s command_linear_x={command.linear.x:.2f} m/s '
                f'active={self.started_at is not None and not self.finished}')

    def _odom_callback(self, message: Odometry) -> None:
        """Log native Gazebo odometry for integration evidence only."""
        now = self.get_clock().now().nanoseconds / 1e9
        if now > 0.0 and now - self._last_odom_log >= 1.0:
            self._last_odom_log = now
            pose = message.pose.pose.position
            velocity = message.twist.twist.linear
            self.get_logger().info(
                f'sim={now:.1f}s odom x={pose.x:.2f} y={pose.y:.2f} '
                f'vx={velocity.x:.2f} m/s')


def main() -> None:
    rclpy.init()
    node = VehicleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
