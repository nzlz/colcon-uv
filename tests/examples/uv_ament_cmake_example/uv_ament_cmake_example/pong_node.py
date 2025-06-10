#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import requests
import sys
import os


class PongNode(Node):
    """A pong node that responds to ping messages with numpy and requests demos."""

    def __init__(self):
        super().__init__("pong_node")

        # Create subscriber
        self.subscription = self.create_subscription(
            String, "ping_pong", self.ping_callback, 10
        )

        # Create publisher for responses
        self.publisher = self.create_publisher(String, "pong_response", 10)

        # Counter for responses
        self.response_counter = 0

        # Log startup info with library versions and venv info
        venv_path = os.environ.get("VIRTUAL_ENV", sys.prefix)
        package_name = "uv_ament_cmake_example"
        self.get_logger().info(
            f"Pong Node started from {package_name} venv: {venv_path}"
        )
        self.get_logger().info(f"Using numpy version: {np.__version__}")
        self.get_logger().info(f"Using requests version: {requests.__version__}")

    def ping_callback(self, msg):
        """Handle incoming ping messages and respond with pong."""
        self.get_logger().info(f"Received: {msg.data}")

        # Generate response with numpy calculation
        random_array = np.random.rand(5)
        mean_value = np.mean(random_array)

        # Create pong response
        response_msg = String()
        response_msg.data = f"PONG #{self.response_counter} from uv_ament_cmake_example venv (numpy {np.__version__}, mean: {mean_value:.3f})"

        self.publisher.publish(response_msg)
        self.get_logger().info(f"Responded: {response_msg.data}")

        self.response_counter += 1


def main(args=None):
    """Main entry point for the pong node."""
    rclpy.init(args=args)

    pong_node = PongNode()

    try:
        rclpy.spin(pong_node)
    except KeyboardInterrupt:
        pass
    finally:
        pong_node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            # Ignore shutdown errors (common in launch files)
            pass


if __name__ == "__main__":
    main()
