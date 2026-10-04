#!/usr/bin/python3
"""Aggregates robot state into /robot/telemetry (robot_interfaces/RobotTelemetry).

Everything here is derived from real ROS state (TF, /odom, /scan, Nav2 action status/feedback,
AMCL, safety/docking/battery topics). A value that cannot be grounded is NaN / empty / false
rather than a placeholder. Paths stay on their dedicated topics (/plan, /local_plan,
/travelled_path, /navigation/goal).
"""

import math

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import DockingState as DS
from robot_interfaces.msg import RobotBatteryState as RB
from robot_interfaces.msg import MapState, RobotTelemetry, SafetyState
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

NAN = float('nan')
ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING)
NAV_ERRORS = {
    0: '', 1: 'GOAL_REJECTED', 2: 'SEND_GOAL_FAILURE', 9000: 'UNKNOWN',
    9001: 'FAILED_TO_LOAD_BEHAVIOR_TREE', 9002: 'TF_ERROR', 9003: 'TIMEOUT',
}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RobotStateManager(Node):
    def __init__(self):
        super().__init__('robot_state_manager')
        d = self.declare_parameter
        self.map_frame = d('map_frame', 'map').value
        self.base_frame = d('base_frame', 'base_footprint').value
        self.rate = d('publish_rate', 5.0).value
        self.moving_v = d('moving_linear_threshold', 0.02).value
        self.moving_w = d('moving_angular_threshold', 0.05).value
        self.odom_timeout = d('odom_timeout', 1.0).value
        self.replan_window = d('replanning_window', 3.0).value
        self.terminal_hold = d('terminal_state_hold', 5.0).value
        # Localization confidence from AMCL covariance, expressed as standard deviations:
        #   position_stddev_m = sqrt(max(var_x, var_y)),  yaw_stddev_rad = sqrt(var_yaw)
        # LOCALIZED at or below the localized_* limits, LOST at or above the lost_* limits,
        # DEGRADED in between. Tunable; to be set after physical testing.
        self.loc_ok_pos = d('localized_max_position_stddev_m', 0.5).value
        self.loc_ok_yaw = d('localized_max_yaw_stddev_rad', 0.387).value
        self.loc_lost_pos = d('lost_position_stddev_m', 1.0).value
        self.loc_lost_yaw = d('lost_yaw_stddev_rad', 0.707).value
        # Self-hit rejection: scan points inside this base-frame rectangle are the robot itself.
        self.body_x = d('body_half_length', 0.27).value
        self.body_y = d('body_half_width', 0.32).value
        self.body_margin = d('body_margin', 0.03).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(RobotTelemetry, '/robot/telemetry', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_amcl, latched)
        # Both navigation actions drive the same state machine; the feedback fields read
        # below (distance, recoveries, elapsed time) exist on both.
        self.nav_actions = {
            'navigate_to_pose': NavigateToPose,
            'navigate_through_poses': NavigateThroughPoses,
        }
        for name, action in self.nav_actions.items():
            self.create_subscription(
                GoalStatusArray, f'{name}/_action/status',
                lambda msg, n=name: self.on_nav_status(msg, n), latched)
            self.create_subscription(
                action.Impl.FeedbackMessage, f'{name}/_action/feedback',
                self.on_nav_feedback, 10)
        self.create_subscription(PoseStamped, 'navigation/goal', self.on_goal, latched)
        self.create_subscription(SafetyState, '/robot/safety/state', self.on_safety, latched)
        self.create_subscription(DS, '/robot/docking/state', self.on_docking, latched)
        self.create_subscription(RB, '/robot/battery/state', self.on_battery, latched)
        self.create_subscription(MapState, '/robot/map/state', self.on_map_state, latched)
        self.result_clients = {
            name: self.create_client(action.Impl.GetResultService, f'{name}/_action/get_result')
            for name, action in self.nav_actions.items()}

        self.odom = None
        self.odom_rx = None
        self.nearest = (NAN, NAN)
        self.amcl_stddev = None      # (position_stddev_m, yaw_stddev_rad)
        self.had_pose = False
        self.safety = None
        self.docking = None
        self.battery = None
        self.map_id = ''
        self.goal_pose = None

        self.nav_state = 'IDLE'
        self.nav_goal_id = None
        self.nav_goal_action = None
        self.nav_error = (0, '')
        self.nav_terminal_at = None
        self.nav_feedback = None
        self.nav_got_feedback = False
        self.nav_recoveries = 0
        self.nav_replan_at = None
        self.nav_start = None
        self.lidar_tf = None

        self.create_timer(1.0 / self.rate, self.on_tick)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- subscriptions ----
    def on_odom(self, msg):
        self.odom = msg
        self.odom_rx = self.now_s()

    def on_amcl(self, msg):
        c = msg.pose.covariance
        if min(c[0], c[7], c[35]) < 0.0:
            self.amcl_stddev = None
            return
        self.amcl_stddev = (math.sqrt(max(c[0], c[7])), math.sqrt(c[35]))

    def on_safety(self, msg):
        self.safety = msg

    def on_docking(self, msg):
        self.docking = msg

    def on_battery(self, msg):
        self.battery = msg

    def on_map_state(self, msg):
        self.map_id = msg.active_map_id

    def on_goal(self, msg):
        self.goal_pose = msg

    def on_scan(self, msg):
        try:
            if self.lidar_tf is None or self.lidar_tf[0] != msg.header.frame_id:
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame, msg.header.frame_id, rclpy.time.Time())
                t = tf.transform
                self.lidar_tf = (msg.header.frame_id, t.translation.x, t.translation.y,
                                 yaw_of(t.rotation))
        except TransformException:
            return
        _, lx, ly, lyaw = self.lidar_tf
        r = np.asarray(msg.ranges, dtype=np.float64)
        ang = msg.angle_min + np.arange(r.size) * msg.angle_increment
        ok = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        if not ok.any():
            self.nearest = (NAN, NAN)
            return
        r, ang = r[ok], ang[ok] + lyaw
        x, y = lx + r * np.cos(ang), ly + r * np.sin(ang)
        outside = ~((np.abs(x) < self.body_x + self.body_margin)
                    & (np.abs(y) < self.body_y + self.body_margin))
        if not outside.any():
            self.nearest = (NAN, NAN)
            return
        x, y = x[outside], y[outside]
        dist = np.hypot(x, y)
        i = int(np.argmin(dist))
        self.nearest = (float(dist[i]), float(math.atan2(y[i], x[i])))

    # ---- navigation state ----
    def on_nav_status(self, msg, action_name):
        active = [s for s in msg.status_list if s.status in ACTIVE]
        if active:
            gid = bytes(active[-1].goal_info.goal_id.uuid)
            if gid != self.nav_goal_id:
                self.nav_goal_id = gid
                self.nav_goal_action = action_name
                self.nav_feedback = None
                self.nav_got_feedback = False
                self.nav_recoveries = 0
                self.nav_replan_at = None
                self.nav_error = (0, '')
                self.goal_pose = None
                self.nav_start = self.now_s()
            self.nav_state = 'PLANNING'
            self.nav_terminal_at = None
        elif self.nav_goal_id is not None:
            mine = [s for s in msg.status_list
                    if bytes(s.goal_info.goal_id.uuid) == self.nav_goal_id]
            if not mine:
                return
            status = mine[-1].status
            self.nav_state = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                              GoalStatus.STATUS_CANCELED: 'CANCELLED'}.get(status, 'FAILED')
            self.nav_terminal_at = self.now_s()
            result_client = self.result_clients.get(self.nav_goal_action)
            if self.nav_state == 'FAILED' and result_client and result_client.service_is_ready():
                req = self.nav_actions[self.nav_goal_action].Impl.GetResultService.Request()
                req.goal_id.uuid = list(self.nav_goal_id)
                result_client.call_async(req).add_done_callback(self.on_nav_result)
            self.nav_goal_id = None

    def on_nav_result(self, future):
        res = future.result()
        if res is not None:
            self.nav_error = (int(res.result.error_code), res.result.error_msg)

    def on_nav_feedback(self, msg):
        if self.nav_goal_id is None or bytes(msg.goal_id.uuid) != self.nav_goal_id:
            return
        fb = msg.feedback
        self.nav_feedback = fb
        self.nav_got_feedback = True
        if fb.number_of_recoveries > self.nav_recoveries:
            self.nav_replan_at = self.now_s()
        self.nav_recoveries = fb.number_of_recoveries

    def navigation_state(self):
        if self.nav_goal_id is not None:
            if not self.nav_got_feedback:
                return 'PLANNING'
            if self.nav_replan_at is not None and self.now_s() - self.nav_replan_at < self.replan_window:
                return 'REPLANNING'
            return 'NAVIGATING'
        if (self.nav_terminal_at is not None
                and self.now_s() - self.nav_terminal_at > self.terminal_hold):
            self.nav_state = 'IDLE'
            self.nav_terminal_at = None
        return self.nav_state

    # ---- localization ----
    def localization_state(self, pose_ok):
        if self.amcl_stddev is None:
            return 'UNINITIALIZED' if not self.had_pose else 'LOST'
        if not pose_ok:
            return 'LOST'
        pos, yaw = self.amcl_stddev
        if pos >= self.loc_lost_pos or yaw >= self.loc_lost_yaw:
            return 'LOST'
        if pos > self.loc_ok_pos or yaw > self.loc_ok_yaw:
            return 'DEGRADED'
        return 'LOCALIZED'

    # ---- publish ----
    def on_tick(self):
        t = RobotTelemetry()
        t.stamp = self.get_clock().now().to_msg()

        # POSE
        t.frame = self.map_frame
        pose_ok = False
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.0))
            tr = tf.transform
            t.x, t.y, t.yaw = tr.translation.x, tr.translation.y, yaw_of(tr.rotation)
            pose_ok = True
            self.had_pose = True
        except TransformException:
            t.x = t.y = t.yaw = NAN
        t.pose_valid = pose_ok

        # MOTION
        t.linear_velocity = t.angular_velocity = NAN
        if self.odom is not None and self.now_s() - self.odom_rx < self.odom_timeout:
            v, w = self.odom.twist.twist.linear.x, self.odom.twist.twist.angular.z
            t.linear_velocity, t.angular_velocity = v, w
            t.is_moving = abs(v) > self.moving_v or abs(w) > self.moving_w

        # NAVIGATION
        t.navigation_state = self.navigation_state()
        t.distance_remaining = NAN
        t.navigation_elapsed_s = 0.0
        active = self.nav_goal_id is not None
        if active and self.nav_feedback is not None:
            fb = self.nav_feedback
            t.distance_remaining = fb.distance_remaining
            t.navigation_elapsed_s = fb.navigation_time.sec + fb.navigation_time.nanosec * 1e-9
        elif active and self.nav_start is not None:
            t.navigation_elapsed_s = self.now_s() - self.nav_start
        if active and self.goal_pose is not None:
            t.has_goal = True
            t.active_goal = self.goal_pose
        t.navigation_error_code, t.navigation_error_msg = self.nav_error[0], self.nav_error[1]

        # LOCALIZATION
        t.localization_state = self.localization_state(pose_ok)
        t.localization_position_stddev_m, t.localization_yaw_stddev_rad = \
            self.amcl_stddev if self.amcl_stddev is not None else (NAN, NAN)

        # SAFETY
        if self.safety is not None:
            t.safety_state = self.safety.state
            t.emergency_stopped = self.safety.emergency_stop_active
            t.collision_stopped = self.safety.state == SafetyState.COLLISION_STOP
            t.control_source = self.safety.control_source

        # OBSTACLE
        t.nearest_obstacle_distance, t.nearest_obstacle_bearing = self.nearest

        # DOCKING
        if self.docking is not None:
            d = self.docking
            t.docking_state = d.state
            t.is_docked = d.is_docked
            t.docking_time_s = d.docking_time_s
            t.docking_retries = d.num_retries
            t.docking_error_code = d.error_code
            t.docking_error_name = d.error_name

        # BATTERY
        t.battery_percentage = t.battery_voltage = t.battery_temperature = NAN
        if self.battery is not None:
            b = self.battery
            t.battery_state = b.state
            t.battery_percentage = b.percentage
            t.charging = b.is_charging
            t.battery_voltage = b.voltage
            t.battery_temperature = b.temperature

        t.active_map = self.map_id
        self.pub.publish(t)


def main():
    rclpy.init()
    node = RobotStateManager()
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
