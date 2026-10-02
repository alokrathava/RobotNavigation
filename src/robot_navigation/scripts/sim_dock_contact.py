#!/usr/bin/python3
"""Simulation backend for /robot/dock/contact (std_msgs/Bool, latched).

On hardware the dock contact driver publishes the same topic. Gazebo ground truth is used only
to decide whether the charging contacts touch the dock; the tolerance is a simulation parameter.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SimDockContact(Node):
    def __init__(self):
        super().__init__('sim_dock_contact')
        d = self.declare_parameter
        d('dock_x', 0.0)
        d('dock_y', 0.0)
        d('dock_yaw', 0.0)
        d('contact_tol_x', 0.08)
        d('contact_tol_y', 0.06)
        d('contact_tol_yaw', 0.20)
        d('contact_fault', False)       # contacts never report touching

        self.gt_pose = None
        self.last_published = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Bool, '/robot/dock/contact', latched)
        self.create_subscription(Odometry, '/sim/ground_truth/odom', self.on_gt, 10)
        self.create_timer(0.1, self.on_tick)

    def p(self, name):
        return self.get_parameter(name).value

    def on_gt(self, msg):
        p = msg.pose.pose
        self.gt_pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def compute_contact(self):
        if self.gt_pose is None or self.p('contact_fault'):
            return False
        x, y, yaw = self.gt_pose
        dx, dy = x - self.p('dock_x'), y - self.p('dock_y')
        c, s = math.cos(self.p('dock_yaw')), math.sin(self.p('dock_yaw'))
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        dyaw = abs(math.atan2(math.sin(yaw - self.p('dock_yaw')),
                              math.cos(yaw - self.p('dock_yaw'))))
        return (abs(lx) < self.p('contact_tol_x') and abs(ly) < self.p('contact_tol_y')
                and dyaw < self.p('contact_tol_yaw'))

    def on_tick(self):
        contact = self.compute_contact()
        if contact != self.last_published:
            self.pub.publish(Bool(data=contact))
            self.last_published = contact


def main():
    rclpy.init()
    node = SimDockContact()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
