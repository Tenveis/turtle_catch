#!/usr/bin/env python3


from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtle_catch.msg import TurtleArray

import math
import random


class TurtleControl(Node):
    def __init__(self):
        super().__init__("turtle_control_py")
        self.init_all()
        self.get_logger().info("object of TurtleControl class has been created.")

    def init_all(self):
        self.upper_bound = 11.0
        self.lower_bound = 0.0

        # self.target_x_ = 3.3
        # self.target_y_ = 1.0

        self.t1_pose_ = None
        self.turtle_to_catch_ = None
        self.turtle_counter_ = 0

        self.lin_pid = [1.0, 0.0, 0.0]
        self.ang_pid = [4.0, 0.0, 0.0]

        self.t1_vel_pub_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.t1_pose_sub_ = self.create_subscription(
            Pose, "turtle1/pose", self.t1_poseCallback, 10
        )
        self.alive_turtles_sub_ = self.create_subscription(
            TurtleArray, "alive_turtles",self.alive_turtlesCallback, 5
        )

        self.control_loop_timer_ = self.create_timer(1 / 100, self.cotrol_loop_callback)

    def t1_poseCallback(self, pose: Pose):
        self.t1_pose_ = pose
        self.get_logger().info(
            "\n\t\tx: {}, \n\t\ty: {}, \n\t\ttheta: {}, ".format(
                self.t1_pose_.x, self.t1_pose_.y, self.t1_pose_.theta
            )
        )

    def alive_turtlesCallback(self, msg: TurtleArray):
        self.turtle_to_catch_ = msg.turtles[self.turtle_counter_]

    def cotrol_loop_callback(self):
        if (
            (self.t1_pose_ == None)
            or (self.turtle_to_catch_ == None)
        ):
            return

        dist_x = self.turtle_to_catch_.x - self.t1_pose_.x
        dist_y = self.turtle_to_catch_.y - self.t1_pose_.y
        distance = math.sqrt(dist_x**2 + dist_y**2)

        t1_vel = Twist()

        if distance > 0.1:
            t1_vel.linear.x = self.lin_pid[0] * distance

            end_theta = math.atan2(dist_y, dist_x)
            diff_theta = end_theta - self.t1_pose_.theta

            if diff_theta > math.pi:
                diff_theta -= 2 * math.pi
            elif diff_theta < -math.pi:
                diff_theta += 2 * math.pi

            t1_vel.angular.z = self.ang_pid[0] * diff_theta
        else:
            t1_vel.linear.x = 0.0
            t1_vel.angular.z = 0.0

            self.turtle_counter_ += 1
            self.get_logger().info(f"Counter value: {self.turtle_counter_}")

            self.turtle_to_catch_=None

            # self.target_x_ = random.uniform(self.lower_bound, self.upper_bound)
            # self.target_y_ = random.uniform(self.lower_bound, self.upper_bound)

        self.t1_vel_pub_.publish(t1_vel)
