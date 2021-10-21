#!/usr/bin/env python3

# Author: Brighten Lee

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="mi_ros2",
                executable="mi_driver_node",
                name="mi_driver_node",
                output="screen",
                parameters=[
                    {
                        "port": "/dev/mi",
                        "frame_id": "imu_link",
                    }
                ],
            ),
        ]
    )
