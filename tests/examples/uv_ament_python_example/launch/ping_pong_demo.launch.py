#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch both ping and pong nodes for the ping-pong demo."""

    return LaunchDescription(
        [
            # Launch the ping node (from uv_ament_python_example)
            Node(
                package="uv_ament_python_example",
                executable="ping_node",
                name="ping_node",
                output="screen",
                parameters=[],
            ),
            # Launch the pong node (from uv_ament_cmake_example)
            Node(
                package="uv_ament_cmake_example",
                executable="pong_node",
                name="pong_node",
                output="screen",
                parameters=[],
            ),
        ]
    )
