#!/usr/bin/env python3
# ============================================================
#  Lancement de Nav2 pour le robot (navigation seule).
#
#  Pas d'AMCL ni de map_server ici : la carte (/map) et le TF map -> odom
#  viennent de slam_toolbox, lance par robot_slam.launch.py.
#
#  Ordre de lancement :
#    terminal 1 : ros2 launch robot_bringup robot_slam.launch.py odom:=true foxglove:=true
#    terminal 2 : ros2 launch robot_bringup nav2.launch.py
#
#  Chaine de commande finale :
#    controller_server -> velocity_smoother -> collision_monitor -> /cmd_vel -> odom_node.py
#
#  A placer dans : ~/ros2_ws/src/robot_bringup/launch/
#  (puis : cd ~/ros2_ws && colcon build --packages-select robot_bringup --symlink-install)
# ============================================================

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('robot_bringup'), 'config', 'nav2_params.yaml',
    ])

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py',
            ])
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': 'false',
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
    )

    return LaunchDescription([nav2])
