#!/usr/bin/python3
"""Publishes the travelled path and the active navigation goal for operators."""

import math

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener

TERMINAL = (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)
NAV_ACTIONS = ('navigate_to_pose', 'navigate_through_poses')


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class NavMonitor(Node):
    def __init__(self):
        super().__init__('nav_monitor')
        self.global_frame = self.declare_parameter('global_frame', 'map').value
        self.robot_frame = self.declare_parameter('robot_frame', 'base_footprint').value
        self.min_distance = self.declare_parameter('min_distance', 0.05).value
        self.min_angle = self.declare_parameter('min_angle', 0.1).value
        self.max_poses = self.declare_parameter('max_poses', 5000).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.path_pub = self.create_publisher(Path, 'travelled_path', latched)
        self.goal_pub = self.create_publisher(PoseStamped, 'navigation/goal', latched)

        self.create_subscription(Path, 'plan', self.on_plan, 10)
        self.active_goals = {action: None for action in NAV_ACTIONS}
        for action in NAV_ACTIONS:
            self.create_subscription(
                GoalStatusArray, f'{action}/_action/status',
                lambda msg, a=action: self.on_status(a, msg), latched)
        self.create_timer(0.2, self.on_timer)

        self.path = Path()
        self.path.header.frame_id = self.global_frame

    @property
    def goal_active(self):
        return any(self.active_goals.values())

    def on_plan(self, msg):
        if msg.poses and self.goal_active:
            self.goal_pub.publish(msg.poses[-1])

    def on_status(self, action, msg):
        executing = [s for s in msg.status_list
                     if s.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)]
        if executing:
            goal_id = bytes(executing[-1].goal_info.goal_id.uuid)
            if goal_id != self.active_goals[action]:
                self.active_goals[action] = goal_id
                self.path.poses.clear()
                self.get_logger().info(f'New {action} goal: travelled path reset')
        elif self.active_goals[action] and any(s.status in TERMINAL for s in msg.status_list):
            self.active_goals[action] = None
            if not self.goal_active:
                self.get_logger().info('Navigation goal finished')

    def on_timer(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.0))
        except TransformException:
            return

        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = tf.header.stamp
        t, r = tf.transform.translation, tf.transform.rotation
        pose.pose.position.x, pose.pose.position.y = t.x, t.y
        pose.pose.orientation = r

        if self.path.poses:
            last = self.path.poses[-1].pose
            dist = math.hypot(t.x - last.position.x, t.y - last.position.y)
            dyaw = abs(math.atan2(math.sin(yaw_of(r) - yaw_of(last.orientation)),
                                  math.cos(yaw_of(r) - yaw_of(last.orientation))))
            if dist < self.min_distance and dyaw < self.min_angle:
                return

        self.path.poses.append(pose)
        if len(self.path.poses) > self.max_poses:
            del self.path.poses[0]
        self.path.header.stamp = pose.header.stamp
        self.path_pub.publish(self.path)


def main():
    rclpy.init()
    node = NavMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
