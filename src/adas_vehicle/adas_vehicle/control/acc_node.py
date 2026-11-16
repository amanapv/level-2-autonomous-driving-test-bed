#!/usr/bin/env python3
"""Adaptive Cruise Control (ACC) using the nearest forward LiDAR return."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class AccNode(Node):

    def __init__(self):
        super().__init__('acc_node')

        self.declare_parameter('cruise_speed_mps', 3.0)
        self.declare_parameter('time_headway_s', 1.5)
        self.declare_parameter('min_gap_m', 1.5)

        self.cruise_speed = self.get_parameter('cruise_speed_mps').value
        self.time_headway = self.get_parameter('time_headway_s').value
        self.min_gap = self.get_parameter('min_gap_m').value

        self.obstacle_distance = float('inf')

        self.create_subscription(
            Float32, '/adas/obstacle_distance', self._distance_cb, 10)

        self.speed_pub = self.create_publisher(Float32, '/adas/acc_speed_cmd', 10)

        self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            f'ACC active: cruise {self.cruise_speed} m/s, '
            f'headway {self.time_headway} s, min gap {self.min_gap} m')

    def _distance_cb(self, msg: Float32):
        self.obstacle_distance = msg.data

    def _control_loop(self):
        # The distance detector publishes the sensor's maximum range when clear.
        gap = max(self.obstacle_distance - self.min_gap, 0.0)
        desired_gap_at_cruise = self.cruise_speed * self.time_headway
        if desired_gap_at_cruise <= 1e-3:
            target_speed = 0.0
        else:
            ratio = max(0.0, min(1.0, gap / desired_gap_at_cruise))
            target_speed = self.cruise_speed * ratio

        self.speed_pub.publish(Float32(data=float(target_speed)))


def main(args=None):
    rclpy.init(args=args)
    node = AccNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
