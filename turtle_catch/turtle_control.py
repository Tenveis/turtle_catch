#!/usr/bin/env python3

# ros2 import
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

# custom import
from turtle_catch.msg import TurtleArray
from turtle_catch.srv import CatchTurtle

# std import
import math
from functools import partial


class TurtleControl(Node):
    def __init__(self):
        super().__init__("turtle_control_py")
        self.init_all()
        self.get_logger().info("object of TurtleControl class has been created.")

    def init_all(self):
        self.upper_bound = 11.0
        self.lower_bound = 0.0

        self.t1_pose_ = None
        self.turtle_to_catch_ = None

        self.lin_pid = [1.0, 0.0, 0.0]
        self.ang_pid = [4.0, 0.0, 0.0]

        self.t1_vel_pub_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.t1_pose_sub_ = self.create_subscription(
            Pose, "turtle1/pose", self.t1_poseCallback, 10
        )
        self.alive_turtles_sub_ = self.create_subscription(
            TurtleArray, "alive_turtles", self.alive_turtlesCallback, 5
        )

        self.control_loop_timer_ = self.create_timer(1 / 100, self.cotrol_loop_callback)

    def t1_poseCallback(self, pose: Pose):
        self.t1_pose_ = pose
        # self.get_logger().info(
        #     "\n\t\tx: {}, \n\t\ty: {}, \n\t\ttheta: {}, ".format(
        #         self.t1_pose_.x, self.t1_pose_.y, self.t1_pose_.theta
        #     )
        # )

    def alive_turtlesCallback(self, msg: TurtleArray):
        short_distance = None
        near_turtle = None

        for turtle in msg.turtles:
            dist_x = turtle.x - self.t1_pose_.x
            dist_y = turtle.y - self.t1_pose_.y
            distance = math.sqrt(dist_x**2 + dist_y**2)
            if near_turtle==None or distance < short_distance:
                short_distance = distance
                near_turtle = turtle

        self.turtle_to_catch_ = near_turtle

    def catch_turtle_serviceCallback(self, turtle_name):
        client = self.create_client(CatchTurtle, "catch_turtle")

        while not client.wait_for_service(1.0):
            self.get_logger().warn("waiting for server: /catch_turtle")

        req = CatchTurtle.Request()
        req.name = turtle_name

        future = client.call_async(req)
        future.add_done_callback(partial(self.catch_callback, t_name=turtle_name))

    def catch_callback(self, future, t_name):
        try:
            resp = future.result()
            self.get_logger().info("%s removed." % t_name)
        except Exception as e:
            self.get_logger().error("Service call failed. %r" % e)

    def cotrol_loop_callback(self):
        if (self.t1_pose_ == None) or (self.turtle_to_catch_ == None):
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

            self.catch_turtle_serviceCallback(self.turtle_to_catch_.name)
            self.turtle_to_catch_ = None

        self.t1_vel_pub_.publish(t1_vel)
