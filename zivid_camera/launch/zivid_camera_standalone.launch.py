# Copyright (c) 2020 Norwegian University of Science and Technology
# Use of this source code is governed by the BSD 3-Clause license, see LICENSE

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
        name="camera",
        namespace="zivid",
        output="screen",
        parameters=[
            {"frame_id": "zivid_camera_frame"},
            {"settings_path": settings_path},
            {"settings2d_path": settings2d_path},
            {"file_camera_path": ""},
            {"update_firmware_automatically": False},
            {"use_latched_publisher_for_points_xyz": True},
        ],
    )

    return LaunchDescription([zivid_camera_standalone])
