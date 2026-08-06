#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for uv_ament_cmake_example."""
    return LaunchDescription(
        [
            Node(
                package="uv_ament_cmake_example",
                executable="uv_ament_cmake_example_node",
                name="uv_ament_cmake_example_node",
                output="screen",
                parameters=[{"use_sim_time": False}],
            )
        ]
    )
