import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            parameters=[{
                'camera': '/base/axi/pcie@120000/rp1/i2c@80000/imx708@1a',
                'format': 'RGB888',
                'width': 640,
                'height': 480,
            }],
            additional_env={
                'LD_LIBRARY_PATH': '/usr/local/lib/aarch64-linux-gnu:'
                                   + os.environ.get('LD_LIBRARY_PATH', '')
            },
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf',
            arguments=['0.10', '0.0', '0.12', '0', '0', '0',
                       'base_link', 'camera_link'],
        ),
        Node(
            package='robot_bringup',
            executable='aruco_detector.py',
            name='aruco_detector',
            output='screen',
        ),
    ])