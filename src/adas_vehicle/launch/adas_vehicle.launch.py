#!/usr/bin/env python3
"""Launch the Gazebo vehicle, ROS bridge, ADAS nodes, and RViz."""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('adas_vehicle')
    world_path = os.path.join(pkg_share, 'worlds', 'adas_test_track.sdf')
    xacro_path = os.path.join(pkg_share, 'urdf', 'vehicle.urdf.xacro')
    rviz_path = os.path.join(pkg_share, 'rviz', 'adas_vehicle.rviz')
    params_path = os.path.join(pkg_share, 'config', 'adas_vehicle_params.yaml')
    robot_description = {
        'robot_description': xacro.process_file(xacro_path).toxml()}

    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')

    gz_sim = ExecuteProcess(
        cmd=[
            'ign', 'gazebo', '-r', '-s', '--headless-rendering',
            '--render-engine', 'ogre', world_path,
        ],
        output='screen')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}])

    spawn_vehicle = TimerAction(
        period=5.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_adas_vehicle',
            arguments=['-topic', '/robot_description',
                       '-name', 'adas_vehicle', '-z', '0.05'],
            output='screen')])

    bridge = TimerAction(
        period=6.0,
        actions=[Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='adas_ros_gz_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')])

    adas_nodes = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='adas_vehicle',
                executable='obstacle_detection_node',
                name='obstacle_detection_node',
                output='screen',
                parameters=[params_path, {'use_sim_time': use_sim_time}]),
            Node(
                package='adas_vehicle',
                executable='acc_node',
                name='acc_node',
                output='screen',
                parameters=[params_path, {'use_sim_time': use_sim_time}]),
            Node(
                package='adas_vehicle',
                executable='obstacle_avoidance_node',
                name='obstacle_avoidance_node',
                output='screen',
                parameters=[params_path, {'use_sim_time': use_sim_time}]),
            Node(
                package='adas_vehicle',
                executable='vehicle_controller_node',
                name='vehicle_controller_node',
                output='screen',
                parameters=[params_path, {'use_sim_time': use_sim_time}]),
        ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_rviz),
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo simulation clock'),
        DeclareLaunchArgument(
            'launch_rviz', default_value='true',
            description='Start RViz for the robot model and LiDAR scan'),
        gz_sim,
        robot_state_publisher,
        spawn_vehicle,
        bridge,
        adas_nodes,
        rviz,
    ])
