#!/usr/bin/python3
"""Simulation backend for /robot/dock/detected_pose (stand-in for an AprilTag/vision detector).

Publishes the dock pose as a detector would see it: ground-truth relative pose of the dock seen
from the robot, re-expressed in the odom frame so it is independent of AMCL. On hardware the
camera/AprilTag pipeline publishes the same topic in the same frame.
"""

import math
import random

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SimDockDetector(Node):
    def __init__(self):
        super().__init__('sim_dock_detector')
        d = self.declare_parameter
        d('dock_x', 0.0)
        d('dock_y', 0.0)
        d('dock_yaw', 0.0)
        d('detector_enabled', True)
        d('detector_range', 3.0)
        d('detector_fov', 1.2)
        d('detector_noise_m', 0.0)

        self.gt_pose = None
        self.odom_pose = None
        self.sim_time = None
        self.pub = self.create_publisher(PoseStamped, '/robot/dock/detected_pose', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Odometry, '/sim/ground_truth/odom', self.on_gt, 10)
        self.create_timer(0.1, self.on_tick)

    def p(self, name):
        return self.get_parameter(name).value

    def on_odom(self, msg):
        self.sim_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose
        self.odom_pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_gt(self, msg):
        p = msg.pose.pose
        self.gt_pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_tick(self):
        if (self.gt_pose is None or self.odom_pose is None or self.sim_time is None
                or not self.p('detector_enabled')):
            return
        gx, gy, gyaw = self.gt_pose
        dx, dy = self.p('dock_x') - gx, self.p('dock_y') - gy
        c, s = math.cos(gyaw), math.sin(gyaw)
        rx, ry = c * dx + s * dy, -s * dx + c * dy
        ryaw = self.p('dock_yaw') - gyaw
        if (math.hypot(rx, ry) > self.p('detector_range')
                or abs(math.atan2(ry, rx)) > self.p('detector_fov')):
            return
        noise = self.p('detector_noise_m')
        if noise > 0.0:
            rx += random.gauss(0.0, noise)
            ry += random.gauss(0.0, noise)
        ox, oy, oyaw = self.odom_pose
        co, so = math.cos(oyaw), math.sin(oyaw)
        msg = PoseStamped()
        msg.header.frame_id = 'odom'
        msg.header.stamp.sec = int(self.sim_time)
        msg.header.stamp.nanosec = int((self.sim_time % 1.0) * 1e9)
        msg.pose.position.x = ox + co * rx - so * ry
        msg.pose.position.y = oy + so * rx + co * ry
        yaw = oyaw + ryaw
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SimDockDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
