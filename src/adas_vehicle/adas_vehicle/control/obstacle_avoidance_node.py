#!/usr/bin/env python3
"""
Obstacle avoidance — reactive steering bias from the 2D LiDAR.

Looks at the full forward hemisphere of the scan, and when something is
inside the avoidance trigger distance directly ahead, steers toward the
scan direction with the greatest measured clearance. The vehicle controller
uses this bias while avoidance is active.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        self.declare_parameter('trigger_distance_m', 1.8)
        self.declare_parameter('hemisphere_half_angle_deg', 70.0)
        self.declare_parameter('max_bias_rad', 0.6)
        self.declare_parameter('scan_topic', '/scan')

        self.trigger_distance = self.get_parameter('trigger_distance_m').value
        self.half_angle = math.radians(
            self.get_parameter('hemisphere_half_angle_deg').value)
        self.max_bias = self.get_parameter('max_bias_rad').value
        scan_topic = self.get_parameter('scan_topic').value

        self.bias_pub = self.create_publisher(Float32, '/adas/avoidance_steering_bias', 10)
        self.active_pub = self.create_publisher(Bool, '/adas/avoidance_active', 10)

        self.create_subscription(
            LaserScan, scan_topic, self._scan_callback, qos_profile_sensor_data)

        self.get_logger().info(
            f'Obstacle avoidance armed: trigger {self.trigger_distance} m, '
            f'hemisphere +/-{math.degrees(self.half_angle):.0f} deg')

    def _scan_callback(self, msg: LaserScan):
        # narrow forward cone used only to decide whether avoidance should engage
        narrow_half_angle = math.radians(20.0)
        angle = msg.angle_min
        forward_min = float('inf')
        ranges_in_hemisphere = []  # (angle, range)
        for r in msg.ranges:
            if abs(angle) <= self.half_angle:
                valid_r = r if msg.range_min <= r <= msg.range_max else msg.range_max
                ranges_in_hemisphere.append((angle, valid_r))
                if abs(angle) <= narrow_half_angle:
                    forward_min = min(forward_min, valid_r)
            angle += msg.angle_increment

        engage = forward_min <= self.trigger_distance
        self.active_pub.publish(Bool(data=bool(engage)))

        if not engage or not ranges_in_hemisphere:
            self.bias_pub.publish(Float32(data=0.0))
            return

        # pick the sample with the largest clearance; steer toward its angle
        best_angle, _ = max(ranges_in_hemisphere, key=lambda a_r: a_r[1])
        # normalize angle (-half_angle..half_angle) into a steering bias
        bias = max(-1.0, min(1.0, best_angle / self.half_angle)) * self.max_bias
        self.bias_pub.publish(Float32(data=float(bias)))


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
