#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_slam = PathJoinSubstitution([
        FindPackageShare('robot_bringup'), 'config', 'config_slam.yaml'
    ])

    # Argument : démarrer Foxglove ou non (dev uniquement)
    foxglove_arg = DeclareLaunchArgument(
        'foxglove',
        default_value='false',
        description='Pont Foxglove pour la visu temps reel (dev uniquement)',
    )

    # 1. LiDAR RPLIDAR A2M8
    lidar = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('sllidar_ros2'),
                'launch', 'sllidar_a2m8_launch.py',
            ])
        ),
        launch_arguments={
            'serial_port': '/dev/rplidar',
            'serial_baudrate': '115200',
        }.items(),
    )

    # 2. TF statique : odom -> base_link
    tf_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_odom_base_link',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'odom', '--child-frame-id', 'base_link',
        ],
    )

    # 3. TF statique : base_link -> laser
    tf_base_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_link_laser',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'laser',
        ],
    )

    # 4. SLAM Toolbox
    slam = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch', 'online_async_launch.py',
            ])
        ),
        launch_arguments={
            'slam_params_file': config_slam,
            'use_sim_time': 'false',
        }.items(),
    )

    # 5. Foxglove Bridge (optionnel)
    foxglove = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('foxglove_bridge'),
                'launch', 'foxglove_bridge_launch.xml',
            ])
        ),
        condition=IfCondition(LaunchConfiguration('foxglove')),
    )

    return LaunchDescription([
        foxglove_arg,
        lidar,
        tf_odom_base,
        tf_base_laser,
        slam,
        foxglove,
    ])
