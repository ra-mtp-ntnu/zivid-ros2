import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    rviz_config_dir = os.path.join(get_package_share_directory(
        'zivid_samples'), 'config', 'capture_demo.rviz')
    assert os.path.exists(rviz_config_dir)

    config = '/home/grans/dev_ws/src/zivid-ros/zivid_camera/config/zivid_camera.yml'


    return LaunchDescription([
        Node(package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        ),
        Node(package='zivid_camera',
            node_executable='manual_composition',
            node_name='zivid_camera',
            output='screen',
            parameters=[config]
            
        ),
        Node(package='zivid_samples',
            node_executable='capture_request',
            node_name='capture_request_node',
            output='screen',
        ),
    ])

