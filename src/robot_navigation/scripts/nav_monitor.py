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
STATUS_NAMES = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_CANCELED: 'CANCELED',
                GoalStatus.STATUS_ABORTED: 'ABORTED'}


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
        # Navigation identity is (action_type, goal_uuid); one shared boolean cannot tell
        # the two action status topics apart.
        self.active = None
        self.seen_status = set()
        self.tf_ready = False
        self.timer_ticked = False
        self.tf_wait_logged = 0.0
        for action in NAV_ACTIONS:
            self.create_subscription(
                GoalStatusArray, f'{action}/_action/status',
                lambda msg, a=action: self.on_status(a, msg), latched)
            self.get_logger().info(f'Subscribed to {action}/_action/status')
        self.create_timer(0.2, self.on_timer)
        self.get_logger().info('Timer created (0.2 s)')

        self.path = Path()
        self.path.header.frame_id = self.global_frame
        self.get_logger().info('nav_monitor ready')

    def on_plan(self, msg):
        if msg.poses and self.active is not None:
            self.goal_pub.publish(msg.poses[-1])

    def on_status(self, action, msg):
        if action not in self.seen_status:
            self.seen_status.add(action)
            self.get_logger().info(
                f'First {action} status message received ({len(msg.status_list)} entries)')

        by_uuid = {bytes(s.goal_info.goal_id.uuid): s.status for s in msg.status_list}

        if self.active is not None:
            active_action, active_uuid = self.active
            status = by_uuid.get(active_uuid) if active_action == action else None
            if status in TERMINAL:
                self.get_logger().info(
                    f'Navigation finished: type={active_action} goal_uuid={active_uuid.hex()} '
                    f'status={STATUS_NAMES[status]}')
                self.active = None

        running = [s for s in msg.status_list
                   if s.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)]
        if running:
            goal_id = bytes(running[-1].goal_info.goal_id.uuid)
            if self.active != (action, goal_id):
                self.active = (action, goal_id)
                self.path.poses.clear()
                self.get_logger().info(
                    f'Navigation started: type={action} goal_uuid={goal_id.hex()}')

    def on_timer(self):
        if not self.timer_ticked:
            self.timer_ticked = True
            self.get_logger().info('Timer running (first tick)')
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.0))
        except TransformException as exc:
            now = self.get_clock().now().nanoseconds * 1e-9
            if not self.tf_ready and now - self.tf_wait_logged >= 10.0:
                self.tf_wait_logged = now
                self.get_logger().warning(
                    f'TF {self.global_frame}->{self.robot_frame} not available yet: {exc}')
            return
        if not self.tf_ready:
            self.tf_ready = True
            self.get_logger().info(f'TF {self.global_frame}->{self.robot_frame} available')

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
