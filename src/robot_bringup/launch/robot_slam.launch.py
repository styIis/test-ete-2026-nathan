#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    config_slam = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'config', 'config_slam.yaml',
    ])

    # Argument : demarrer Foxglove ou non (dev uniquement)
    foxglove_arg = DeclareLaunchArgument(
        'foxglove',
        default_value='false',
        description='Pont Foxglove pour la visu temps reel (dev uniquement)',
    )

    # Argument : detection d'obstacle (/scan -> /obstacle_direction)
    detector_arg = DeclareLaunchArgument(
        'detector',
        default_value='false',
        description='Detection d obstacle a partir du /scan (agnostique du materiel)',
    )

    # Argument : pont serie vers le Nucleo (necessite detector:=true)
    nucleo_arg = DeclareLaunchArgument(
        'nucleo',
        default_value='false',
        description='Pont serie /obstacle_direction -> Nucleo (matrice LED)',
    )

    # Argument : LiDAR haut LD06 + detection des adversaires
    ld06_arg = DeclareLaunchArgument(
        'ld06',
        default_value='false',
        description='LiDAR haut LD06 (/scan_high) + detection adversaires (/opponents)',
    )
    # Argument : odometrie reelle depuis le Nucleo
    odom_arg = DeclareLaunchArgument(
        'odom',
        default_value='false',
        description='Noeud odometrie Nucleo (remplace le TF statique odom->base_link)',
    )
    # 1. LiDAR RPLIDAR A2M8 (bas : SLAM + obstacles)
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

    # 2a. Odometrie reelle (Nucleo) -- publie le TF odom -> base_link
    # Incompatible avec nucleo:=true : meme port serie.
    odom_node = Node(
        package='robot_bringup',
        executable='odom_node.py',
        name='odom_node',
        output='screen',
        parameters=[{
            'port': '/dev/esp32',
            'baudrate': 115200,
        }],
        condition=IfCondition(LaunchConfiguration('odom')),
    )

    # 2b. Repli : TF statique odom -> base_link (si odom:=false)
    tf_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_odom_base_link',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'odom', '--child-frame-id', 'base_link',
        ],
        condition=UnlessCondition(LaunchConfiguration('odom')),
    )

    # 3. TF statique : base_link -> laser (robot CdF 35 cm)
    #    base_link = milieu de l'essieu des roues motrices, au sol
    #    x    = 0.175 m (centre du cube a +0.115, lidar 6 cm devant le centre)
    #    z    = 0.035 m (lidar a ~3,5 cm du sol)
    #    roll = pi : lidar monte TETE EN BAS
    #    yaw  = pi : le zero du RPLIDAR pointe vers l'arriere (comme cet ete)
    #    A VERIFIER : objet devant-gauche -> doit apparaitre devant-gauche dans Foxglove
    tf_base_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_link_laser',
        arguments=[
            '--x', '0.175', '--y', '0', '--z', '0.035',
            '--roll', '3.14159', '--pitch', '0', '--yaw', '3.14159',
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

    # 6. Detection d'obstacle (optionnel) : /scan -> /obstacle_direction
    obstacle_detector = Node(
        package='robot_bringup',
        executable='obstacle_detector.py',
        name='obstacle_detector',
        output='screen',
        condition=IfCondition(LaunchConfiguration('detector')),
        parameters=[{
            'angle_offset': 0.0,      # rotation portee par la TF (cf. noeud 3)
        }],
    )

    # 7. Pont serie vers le Nucleo (optionnel) : /obstacle_direction -> UART
    nucleo_bridge = Node(
        package='robot_bringup',
        executable='nucleo_bridge.py',
        name='nucleo_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('nucleo')),
    )

    # 8. LiDAR LD06 (haut : detection des robots adverses)
    ld06 = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='ld06',
        output='screen',
        condition=IfCondition(LaunchConfiguration('ld06')),
        parameters=[{
            'product_name': 'LDLiDAR_LD06',
            'topic_name': 'scan_high',
            'frame_id': 'laser_high',
            'port_name': '/dev/ld06',
            'port_baudrate': 230400,
            'laser_scan_dir': True,
            'enable_angle_crop_func': False,
        }],
    )

    # 9. TF statique : base_link -> laser_high
    #    z    = hauteur de montage du LD06 (PROVISOIRE : a mesurer au montage reel)
    #    yaw  = -1.5708 rad (-90 deg) : orientation mecanique du LD06.
    #           C'est l'ex-ANGLE_OFFSET, desormais porte par la TF et non par le code.
    tf_base_laser_high = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_link_laser_high',
        condition=IfCondition(LaunchConfiguration('ld06')),
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.35',
            '--roll', '0', '--pitch', '0', '--yaw', '-1.5708',
            '--frame-id', 'base_link', '--child-frame-id', 'laser_high',
        ],
    )

    laser_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter',
        parameters=[PathJoinSubstitution([
            FindPackageShare('robot_bringup'),
            'config', 'laser_filter.yaml',
        ])],
    )

    # 10. Detection des adversaires (optionnel) : /scan_high -> /opponents
    opponent_detector = Node(
        package='robot_bringup',
        executable='opponent_detector.py',
        name='opponent_detector',
        output='screen',
        condition=IfCondition(LaunchConfiguration('ld06')),
        parameters=[{
            'angle_offset': 0.0,      # rotation portee par la TF (cf. noeud 9)
            'range_max': 3.0,
            'range_min': 0.10,
            'cluster_gap': 0.15,
            'min_points': 3,
            'min_width': 0.05,
            'max_width': 0.60,
        }],
    )

    return LaunchDescription([
        foxglove_arg,
        detector_arg,
        nucleo_arg,
        ld06_arg,
        odom_arg,
        lidar,
        odom_node,
        tf_odom_base,
        tf_base_laser,
        slam,
        foxglove,
        obstacle_detector,
        nucleo_bridge,
        ld06,
        tf_base_laser_high,
        opponent_detector,
        laser_filter,
    ])  