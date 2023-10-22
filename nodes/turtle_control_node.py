#!/usr/bin/env python3

import rclpy
from turtle_catch.turtle_control import TurtleControl


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControl()
    node.get_logger().info("turtle_control_node is initialized.")
    rclpy.spin(node=node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
