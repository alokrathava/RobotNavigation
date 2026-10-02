#!/usr/bin/python3
"""Robot-level docking state machine on top of the Nav2 docking server.

Services (std_srvs/Trigger): /robot/dock, /robot/undock, /robot/dock/cancel
Topic: /robot/docking/state (robot_interfaces/DockingState, latched)

Docking itself is done by nav2 opennav_docking (DockRobot / UndockRobot actions). This node runs
the preflight checks, maps the server's feedback/result onto the robot docking states and
confirms DOCKED only when the contacts report charging.
"""

import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import DockRobot, UndockRobot
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import DockingState as DS
from robot_interfaces.msg import SafetyState
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

ERROR_NAMES = {
    0: '', 1: 'GOAL_REJECTED', 2: 'SEND_GOAL_FAILURE', 901: 'DOCK_NOT_IN_DB',
    902: 'DOCK_NOT_VALID', 903: 'FAILED_TO_STAGE', 904: 'FAILED_TO_DETECT_DOCK',
    905: 'FAILED_TO_CONTROL', 906: 'FAILED_TO_CHARGE', 907: 'TIMEOUT', 999: 'UNKNOWN',
}
BUSY = (DS.NAVIGATING_TO_DOCK, DS.PRESTAGING, DS.ALIGNING, DS.DOCKING, DS.UNDOCKING)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class DockingManager(Node):
    def __init__(self):
        super().__init__('docking_manager')
        self.dock_id = self.declare_parameter('dock_id', 'DOCK_MAIN').value
        self.dock_type = self.declare_parameter('dock_type', 'simple_charging_dock').value
        self.dock_x = self.declare_parameter('dock_x', 0.0).value
        self.dock_y = self.declare_parameter('dock_y', 0.0).value
        self.dock_yaw = self.declare_parameter('dock_yaw', 0.0).value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.docked_pos_tol = self.declare_parameter('docked_position_tolerance', 0.35).value
        self.docked_yaw_tol = self.declare_parameter('docked_yaw_tolerance', 0.20).value
        self.align_lateral_tol = self.declare_parameter('align_lateral_tolerance', 0.03).value
        self.align_yaw_tol = self.declare_parameter('align_yaw_tolerance', 0.10).value
        self.max_cov_xy = self.declare_parameter('max_amcl_xy_variance', 1.0).value
        self.max_cov_yaw = self.declare_parameter('max_amcl_yaw_variance', 0.5).value
        self.charge_current = self.declare_parameter('charging_current_threshold', 0.2).value
        self.confirm_timeout = self.declare_parameter('charge_confirm_timeout', 4.0).value
        self.sensor_timeout = self.declare_parameter('sensor_timeout', 3.0).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.state_pub = self.create_publisher(DS, '/robot/docking/state', latched)
        self.create_subscription(SafetyState, '/robot/safety/state', self.on_safety, latched)
        self.create_subscription(BatteryState, '/robot/battery', self.on_battery, 10)
        self.create_subscription(Bool, '/robot/dock/contact', self.on_contact, latched)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_amcl, latched)

        self.dock_client = ActionClient(self, DockRobot, 'dock_robot')
        self.undock_client = ActionClient(self, UndockRobot, 'undock_robot')
        self.create_service(Trigger, '/robot/dock', self.srv_dock)
        self.create_service(Trigger, '/robot/undock', self.srv_undock)
        self.create_service(Trigger, '/robot/dock/cancel', self.srv_cancel)

        self.safety = None
        self.battery = None
        self.battery_rx = None
        self.contact = False
        self.amcl_cov = None

        self.state = DS.UNDOCKED
        self.initialised = False
        self.op = None                 # 'dock' | 'undock'
        self.goal_handle = None
        self.cancel_requested = False
        self.op_start = None
        self.retries = 0
        self.err_code = 0
        self.err_msg = ''
        self.detail = ''
        self.confirm_since = None
        self.last_published = None

        self.create_timer(0.2, self.on_tick)

    # ---- inputs ----
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_safety(self, msg):
        self.safety = msg

    def on_battery(self, msg):
        self.battery = msg
        self.battery_rx = self.now_s()

    def on_contact(self, msg):
        self.contact = msg.data

    def on_amcl(self, msg):
        c = msg.pose.covariance
        self.amcl_cov = (c[0], c[7], c[35])

    def map_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.0))
        except TransformException:
            return None
        t = tf.transform
        return t.translation.x, t.translation.y, yaw_of(t.rotation)

    def charging(self):
        b = self.battery
        if b is None or self.now_s() - (self.battery_rx or 0.0) > self.sensor_timeout:
            return False
        full = b.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_FULL
        chg = b.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
        return self.contact and (full or (chg and b.current > self.charge_current))

    def dock_frame_error(self, pose):
        dx, dy = pose[0] - self.dock_x, pose[1] - self.dock_y
        c, s = math.cos(self.dock_yaw), math.sin(self.dock_yaw)
        return c * dx + s * dy, -s * dx + c * dy, wrap(pose[2] - self.dock_yaw)

    def sensed_docked(self):
        """Contacts report charging AND the map pose is at the dock."""
        pose = self.map_pose()
        if pose is None or not self.charging():
            return False
        ex, ey, eyaw = self.dock_frame_error(pose)
        return math.hypot(ex, ey) < self.docked_pos_tol and abs(eyaw) < self.docked_yaw_tol

    # ---- preflight ----
    def preflight(self):
        s = self.safety
        if s is None:
            return 'safety state unavailable'
        if s.emergency_stop_active or s.state == SafetyState.EMERGENCY_STOP:
            return 'emergency stop active'
        if s.motion_inhibited:
            return 'motion inhibited (emergency stop release pending)'
        if s.state == SafetyState.FAULT:
            return f'safety fault: {s.detail}'
        return None

    def localization_problem(self):
        if self.map_pose() is None:
            return 'map->base transform unavailable'
        if self.amcl_cov is None:
            return 'localization not initialised'
        vx, vy, vyaw = self.amcl_cov
        if vx > self.max_cov_xy or vy > self.max_cov_xy or vyaw > self.max_cov_yaw:
            return 'localization uncertain'
        return None

    def reset_op(self):
        self.op = None
        self.goal_handle = None
        self.cancel_requested = False
        self.op_start = None

    # ---- services ----
    def srv_dock(self, _req, res):
        res.success = False
        if self.state in BUSY or self.op is not None:
            res.message = f'docking operation already in progress ({self.state_name()})'
            return res
        problem = self.preflight()
        if problem:
            res.message = problem
            return res
        problem = self.localization_problem()
        if problem:
            res.message = problem
            return res
        if self.sensed_docked():
            self.set_state(DS.DOCKED, 'already docked')
            res.success = True
            res.message = 'already docked'
            return res
        if not self.dock_client.server_is_ready():
            res.message = 'docking server unavailable'
            return res

        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = self.dock_id
        goal.navigate_to_staging_pose = True
        self.start_op('dock', DS.NAVIGATING_TO_DOCK, 'navigating to staging pose')
        self.dock_client.send_goal_async(goal, feedback_callback=self.on_dock_feedback) \
            .add_done_callback(self.on_goal_response)
        res.success = True
        res.message = 'docking started'
        return res

    def srv_undock(self, _req, res):
        res.success = False
        if self.state in BUSY or self.op is not None:
            res.message = f'docking operation already in progress ({self.state_name()})'
            return res
        problem = self.preflight()
        if problem:
            res.message = problem
            return res
        if not self.sensed_docked():
            res.message = 'robot is not docked'
            return res
        if not self.undock_client.server_is_ready():
            res.message = 'docking server unavailable'
            return res
        goal = UndockRobot.Goal()
        goal.dock_type = self.dock_type
        self.start_op('undock', DS.UNDOCKING, 'moving away from dock')
        self.undock_client.send_goal_async(goal).add_done_callback(self.on_goal_response)
        res.success = True
        res.message = 'undocking started'
        return res

    def srv_cancel(self, _req, res):
        if self.op is None:
            res.success = True
            res.message = 'no docking operation active'
            return res
        self.cancel_requested = True
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        res.success = True
        res.message = 'cancel requested'
        return res

    def start_op(self, op, state, detail):
        self.op = op
        self.op_start = self.now_s()
        self.retries = 0
        self.err_code = 0
        self.err_msg = ''
        self.cancel_requested = False
        self.confirm_since = None
        self.set_state(state, detail)

    # ---- action callbacks ----
    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.finish(DS.FAILED, 1, 'goal rejected by docking server')
            return
        self.goal_handle = handle
        if self.cancel_requested:
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(self.on_result)

    def on_dock_feedback(self, msg):
        fb = msg.feedback
        self.retries = fb.num_retries
        if fb.state == DockRobot.Feedback.NAV_TO_STAGING_POSE:
            self.set_state(DS.NAVIGATING_TO_DOCK, 'navigating to staging pose')
        elif fb.state == DockRobot.Feedback.INITIAL_PERCEPTION:
            self.set_state(DS.PRESTAGING, 'at staging pose, perceiving dock')
        elif fb.state == DockRobot.Feedback.CONTROLLING:
            self.set_state(self.approach_state(), 'final approach')
        elif fb.state == DockRobot.Feedback.WAIT_FOR_CHARGE:
            self.set_state(DS.DOCKING, 'waiting for charge')
        elif fb.state == DockRobot.Feedback.RETRY:
            self.set_state(self.state, f'retrying (attempt {fb.num_retries})')

    def approach_state(self):
        pose = self.map_pose()
        if pose is None:
            return DS.DOCKING
        ex, ey, eyaw = self.dock_frame_error(pose)
        if abs(ey) > self.align_lateral_tol or abs(eyaw) > self.align_yaw_tol:
            return DS.ALIGNING
        return DS.DOCKING

    def on_result(self, future):
        wrapped = future.result()
        status, result = wrapped.status, wrapped.result
        if status == GoalStatus.STATUS_CANCELED:
            detail = ('cancelled by operator' if self.cancel_requested
                      else 'cancelled externally (e-stop or manual input)')
            self.finish(DS.CANCELLED, 0, '', detail)
            return
        if status == GoalStatus.STATUS_SUCCEEDED and result.success:
            if self.op == 'undock':
                self.finish(DS.UNDOCKED, 0, '', 'at staging pose')
            else:
                # Dock controller succeeded; DOCKED additionally needs contact confirmation.
                self.goal_handle = None
                self.confirm_since = self.now_s()
                self.set_state(DS.DOCKING, 'confirming charging')
            return
        code = int(result.error_code) or 999
        self.finish(DS.FAILED, code, result.error_msg)

    def finish(self, state, code, msg, detail=''):
        self.err_code, self.err_msg = code, msg
        if state == DS.FAILED:
            detail = f'{self.op or "dock"} failed: {ERROR_NAMES.get(code, "UNKNOWN")}'
        self.reset_op_keep_time()
        self.set_state(state, detail)

    def reset_op_keep_time(self):
        self.last_elapsed = self.elapsed()
        self.reset_op()
        self.confirm_since = None

    def elapsed(self):
        return 0.0 if self.op_start is None else self.now_s() - self.op_start

    # ---- state ----
    def state_name(self):
        return next((k for k, v in vars(DS).items()
                     if k.isupper() and isinstance(v, int) and v == self.state), '?')

    def set_state(self, state, detail=''):
        self.state = state
        self.detail = detail

    def on_tick(self):
        # Charge confirmation after the dock controller reported success.
        if self.confirm_since is not None:
            if self.sensed_docked():
                self.last_elapsed = self.elapsed()
                self.reset_op()
                self.confirm_since = None
                self.err_code, self.err_msg = 0, ''
                self.set_state(DS.DOCKED, 'docked and charging')
            elif self.now_s() - self.confirm_since > self.confirm_timeout:
                self.err_code, self.err_msg = 906, 'charging not detected after docking'
                self.reset_op_keep_time()
                self.set_state(DS.FAILED, 'dock failed: FAILED_TO_CHARGE')
        elif self.op is None:
            docked = self.sensed_docked()
            if not self.initialised:
                if self.battery is not None and self.map_pose() is not None:
                    self.initialised = True
                    self.set_state(DS.DOCKED if docked else DS.UNDOCKED,
                                   'docked at startup' if docked else '')
            elif docked and self.state != DS.DOCKED:
                self.err_code, self.err_msg = 0, ''
                self.set_state(DS.DOCKED, 'docked and charging')
            elif not docked and self.state == DS.DOCKED:
                self.set_state(DS.UNDOCKED, 'contacts lost')
        self.publish_state()

    def publish_state(self):
        msg = DS()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state = self.state
        msg.dock_id = self.dock_id
        msg.dock_pose = PoseStamped()
        msg.dock_pose.header.frame_id = self.map_frame
        msg.dock_pose.pose.position.x = self.dock_x
        msg.dock_pose.pose.position.y = self.dock_y
        msg.dock_pose.pose.orientation.z = math.sin(self.dock_yaw / 2.0)
        msg.dock_pose.pose.orientation.w = math.cos(self.dock_yaw / 2.0)
        msg.is_docked = self.state == DS.DOCKED
        msg.is_charging = self.charging()
        msg.docking_time_s = float(self.elapsed() if self.op is not None
                                   else getattr(self, 'last_elapsed', 0.0))
        msg.num_retries = int(self.retries)
        msg.error_code = int(self.err_code)
        msg.error_name = ERROR_NAMES.get(self.err_code, 'UNKNOWN') if self.err_code else ''
        msg.error_msg = self.err_msg
        msg.detail = self.detail
        self.state_pub.publish(msg)


def main():
    rclpy.init()
    node = DockingManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
