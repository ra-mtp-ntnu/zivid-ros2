from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions.node import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    settings_path = PathJoinSubstitution(
        [FindPackageShare("zivid_camera"), "config", "zivid_camera_settings.yml"]
    )
    settings2d_path = PathJoinSubstitution(
        [FindPackageShare("zivid_camera"), "config", "zivid_camera_settings2d.yml"]
    )

    zivid_camera_standalone = Node(
        package="zivid_camera",
        executable="zivid_camera_standalone",
        name="zivid_camera",
        namespace="",
        output="screen",
        parameters=[
            {"frame_id": "zivid_camera_frame"},
            {"settings_path": settings_path},
            {"settings2d_path": settings2d_path},
            {"file_camera_path": ""},
        ],
    )

    return LaunchDescription([zivid_camera_standalone])
