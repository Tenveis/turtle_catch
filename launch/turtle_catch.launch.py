#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    turtlesim = Node(package="turtlesim", executable="turtlesim_node")
    turtle_spawn = Node(package="turtle_catch", executable="turtle_spawn_node")

    ld.add_action(turtlesim)
    ld.add_action(turtle_spawn)

    return ld
