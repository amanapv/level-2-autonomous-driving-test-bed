#!/usr/bin/env python3
"""
Obstacle detection from the simulated 2D LiDAR.

Scans the forward sector of the LaserScan and publishes the nearest obstacle's
range and bearing, and raises a boolean flag when something is inside the
safety zone directly ahead. This feeds both ACC (distance keeping) and
obstacle avoidance (steer-around) downstream.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


class ObstacleDetectionNode(Node):

    def __init__(self):
        super().__init__('obstacle_detection_node')

        self.declare_parameter('forward_half_angle_deg', 35.0)
        self.declare_parameter('safety_distance_m', 1.2)
        self.declare_parameter('scan_topic', '/scan')

        self.forward_half_angle = math.radians(
            self.get_parameter('forward_half_angle_deg').value)
        self.safety_distance = self.get_parameter('safety_distance_m').value
        scan_topic = self.get_parameter('scan_topic').value

        self.distance_pub = self.create_publisher(
            Float32, '/adas/obstacle_distance', 10)
        self.angle_pub = self.create_publisher(
            Float32, '/adas/obstacle_angle', 10)
        self.detected_pub = self.create_publisher(
            Bool, '/adas/obstacle_detected', 10)

        self.create_subscription(
            LaserScan, scan_topic, self._scan_callback,
            qos_profile_sensor_data)

        self.get_logger().info(
            f'Obstacle detection watching +/-{math.degrees(self.forward_half_angle):.0f} deg, '
            f'safety distance {self.safety_distance} m')

    def _scan_callback(self, msg: LaserScan):
        best_range = float('inf')
        best_angle = 0.0
        angle = msg.angle_min
        for r in msg.ranges:
            if abs(angle) <= self.forward_half_angle:
                if msg.range_min <= r <= msg.range_max and r < best_range:
                    best_range = r
                    best_angle = angle
            angle += msg.angle_increment

        if math.isinf(best_range):
            # nothing seen in the forward sector within sensor range
            self.distance_pub.publish(Float32(data=float(msg.range_max)))
            self.angle_pub.publish(Float32(data=0.0))
            self.detected_pub.publish(Bool(data=False))
            return

        self.distance_pub.publish(Float32(data=float(best_range)))
        self.angle_pub.publish(Float32(data=float(best_angle)))
        self.detected_pub.publish(Bool(data=best_range <= self.safety_distance))


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
