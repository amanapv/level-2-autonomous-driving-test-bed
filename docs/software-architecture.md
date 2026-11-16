# Simulation software and ROS topics

The simulation runs four ROS 2 nodes over the Gazebo bridge:

1. Gazebo publishes the virtual 2D LiDAR scan on `/scan` and odometry on
   `/odom`.
2. `obstacle_detection_node` finds the nearest valid LiDAR return within
   +/-35 degrees and publishes `/adas/obstacle_distance`,
   `/adas/obstacle_angle`, and `/adas/obstacle_detected`.
3. `acc_node` uses obstacle distance to publish an ACC speed target on
   `/adas/acc_speed_cmd`. It scales the cruise target down as the gap shrinks
   toward the configured minimum gap.
4. `obstacle_avoidance_node` checks a +/-20-degree forward cone for an object
   within its trigger distance. When active, it searches scan samples within
   +/-70 degrees and publishes `/adas/avoidance_steering_bias` and
   `/adas/avoidance_active`.
5. `vehicle_controller_node` uses the avoidance bias while active, otherwise
   commands straight steering. It uses odometry and the ACC target to publish
   `/adas/throttle_cmd`, `/adas/brake_cmd`, and `/adas/steering_cmd`, then
   converts the target into a `geometry_msgs/msg/Twist` on `/cmd_vel` for
   Gazebo's Ackermann plugin.

The Gazebo bridge also carries `/clock` and `/tf`. The RViz configuration
displays the robot model, TF frames, and `/scan`. Default thresholds and
controller settings are in
[`../src/adas_vehicle/config/adas_vehicle_params.yaml`](../src/adas_vehicle/config/adas_vehicle_params.yaml).

This is a research simulation, not a safety-certified driver-assistance
system. It does not include physical actuator interfaces.
