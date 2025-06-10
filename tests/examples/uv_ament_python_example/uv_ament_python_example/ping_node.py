#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import sys
import os


class PingNode(Node):
    """A simple ping node that publishes ping messages with numpy-generated data."""

    def __init__(self):
        super().__init__("ping_node")

        # Create publisher
        self.publisher = self.create_publisher(String, "ping_pong", 10)

        # Create timer to publish every 2 seconds
        self.timer = self.create_timer(2.0, self.publish_ping)

        # Counter for messages
        self.counter = 0

        # Log startup info with numpy version and venv info
        venv_path = os.environ.get("VIRTUAL_ENV", sys.prefix)
        package_name = "uv_ament_python_example"
        self.get_logger().info(
            f"Ping Node started from {package_name} venv: {venv_path}"
        )
        self.get_logger().info(f"Using numpy version: {np.__version__}")

    def publish_ping(self):
        """Publish a ping message with numpy-generated random data."""
        # Generate some random data with numpy to show dependency works
        random_value = np.random.random()

        msg = String()
        msg.data = f"PING #{self.counter} from uv_ament_python_example venv (numpy {np.__version__}, random: {random_value:.3f})"

        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

        self.counter += 1


def main(args=None):
    """Main entry point for the ping node."""
    rclpy.init(args=args)

    ping_node = PingNode()

    try:
        rclpy.spin(ping_node)
    except KeyboardInterrupt:
        pass
    finally:
        ping_node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            # Ignore shutdown errors (common in launch files)
            pass


if __name__ == "__main__":
    main()
