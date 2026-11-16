#!/usr/bin/env python3
"""Convert ACC and obstacle-avoidance outputs into simulated vehicle commands."""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class VehicleControllerNode(Node):

    def __init__(self):
        super().__init__('vehicle_controller_node')

        self.declare_parameter('wheelbase_m', 0.33)
        self.declare_parameter('max_speed_mps', 4.0)
        self.declare_parameter('speed_kp', 0.8)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.wheelbase = self.get_parameter('wheelbase_m').value
        self.max_speed = self.get_parameter('max_speed_mps').value
        self.speed_kp = self.get_parameter('speed_kp').value
        odom_topic = self.get_parameter('odom_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        # ADAS module outputs
        self.acc_speed_cmd = 0.0
        self.avoidance_bias = 0.0
        self.avoidance_active = False
        self.current_speed = 0.0

        # Publish high-level actuator commands for inspection.
        self.throttle_pub = self.create_publisher(Float32, '/adas/throttle_cmd', 10)
        self.brake_pub = self.create_publisher(Float32, '/adas/brake_cmd', 10)
        self.steering_pub = self.create_publisher(Float32, '/adas/steering_cmd', 10)

        # low-level actuator command published to the sim (servo steering +
        # DC-motor drive are abstracted by the vehicle's Ackermann/Twist plugin)
        from geometry_msgs.msg import Twist
        self._twist_type = Twist
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.create_subscription(Float32, '/adas/acc_speed_cmd', self._acc_cb, 10)
        self.create_subscription(Float32, '/adas/avoidance_steering_bias', self._bias_cb, 10)
        self.create_subscription(Bool, '/adas/avoidance_active', self._avoid_active_cb, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)

        self.create_timer(0.05, self._control_loop)

        self.get_logger().info('Vehicle controller arbitrating ADAS outputs')

    def _acc_cb(self, msg: Float32):
        self.acc_speed_cmd = msg.data

    def _bias_cb(self, msg: Float32):
        self.avoidance_bias = msg.data

    def _avoid_active_cb(self, msg: Bool):
        self.avoidance_active = msg.data

    def _odom_cb(self, msg: Odometry):
        v = msg.twist.twist.linear
        self.current_speed = math.hypot(v.x, v.y)

    def _control_loop(self):
        steering = self.avoidance_bias if self.avoidance_active else 0.0
        steering = max(-0.6, min(0.6, steering))

        # --- speed target ---
        target_speed = min(self.acc_speed_cmd, self.max_speed)

        # --- speed -> throttle/brake via simple P controller on speed error ---
        speed_error = target_speed - self.current_speed
        if speed_error >= 0.0:
            throttle = max(0.0, min(1.0, self.speed_kp * speed_error))
            brake = 0.0
        else:
            throttle = 0.0
            brake = max(0.0, min(1.0, -self.speed_kp * speed_error))
        if target_speed <= 1e-3:
            # force full stop command regardless of residual speed error sign
            throttle = 0.0
            brake = max(brake, 0.3)

        self.throttle_pub.publish(Float32(data=float(throttle)))
        self.brake_pub.publish(Float32(data=float(brake)))
        self.steering_pub.publish(Float32(data=float(steering)))

        # --- convert to Twist for the sim's Ackermann-steering plugin ---
        commanded_speed = target_speed * (1.0 - brake)
        twist = self._twist_type()
        twist.linear.x = float(commanded_speed)
        if commanded_speed > 1e-3:
            twist.angular.z = float(commanded_speed * math.tan(steering) / self.wheelbase)
        else:
            twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
