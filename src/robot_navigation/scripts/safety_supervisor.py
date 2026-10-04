#!/usr/bin/python3
"""Final stage of the velocity pipeline: E-stop gate, watchdog and safety state.

/cmd_vel_safe (collision monitor output) -> this node -> /cmd_vel (drivetrain).
This is the only publisher of /cmd_vel.
"""

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import TwistStamped
from nav2_msgs.msg import CollisionMonitorState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import SafetyState
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

NAV_ACTIONS = ('navigate_to_pose', 'navigate_through_poses', 'dock_robot', 'undock_robot')
ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING)


def is_zero(msg):
    t = msg.twist
    return t.linear.x == 0.0 and t.linear.y == 0.0 and t.angular.z == 0.0


class SafetySupervisor(Node):
    def __init__(self):
        super().__init__('safety_supervisor')
        self.watchdog_timeout = self.declare_parameter('watchdog_timeout', 0.5).value
        # Dead-man: no manual update for this long -> zero is injected on /cmd_vel_manual.
        self.manual_timeout = self.declare_parameter('manual_timeout', 0.4).value
        self.nav_timeout = self.declare_parameter('nav_timeout', 0.5).value
        self.scan_timeout = self.declare_parameter('scan_timeout', 1.0).value

        self.estop = False
        self.release_hold = False       # E-stop released, waiting for manual stream to go quiet
        self.transition_hold = False    # map lifecycle inhibits all commanded motion
        self.last_out_zero = True
        self.last_safe_rx = None
        self.last_manual_rx = None
        self.last_manual_zero = True
        self.synthetic_zeros = 0        # own zero publications to ignore in on_manual
        self.last_nav_rx = None
        self.last_scan_rx = None
        self.collision_action = CollisionMonitorState.DO_NOTHING
        self.fault = False
        self.dead_man_count = 0
        self.last_dead_man = None
        self.active_goals = {a: False for a in NAV_ACTIONS}
        self.last_published = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.manual_pub = self.create_publisher(TwistStamped, 'cmd_vel_manual', 10)
        self.lock_pub = self.create_publisher(Bool, '/robot/safety/mux_lock', 10)
        self.state_pub = self.create_publisher(SafetyState, '/robot/safety/state', latched)

        self.create_subscription(TwistStamped, 'cmd_vel_safe', self.on_safe, 10)
        self.create_subscription(TwistStamped, 'cmd_vel_manual', self.on_manual, 10)
        self.create_subscription(TwistStamped, 'cmd_vel_nav', self.on_nav, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.create_subscription(CollisionMonitorState, 'collision_monitor_state',
                                 self.on_collision, 10)
        self.cancel_clients = {}
        for action in NAV_ACTIONS:
            self.cancel_clients[action] = self.create_client(
                CancelGoal, f'{action}/_action/cancel_goal')
            self.create_subscription(
                GoalStatusArray, f'{action}/_action/status',
                lambda msg, a=action: self.on_status(a, msg), latched)

        self.create_service(Trigger, '/robot/safety/emergency_stop', self.srv_estop)
        self.create_service(Trigger, '/robot/safety/release_emergency_stop', self.srv_release)
        self.create_service(Trigger, '/robot/manual/stop', self.srv_manual_stop)
        self.create_service(SetBool, '/robot/safety/transition_hold', self.srv_transition_hold)

        self.create_timer(0.05, self.on_tick)
        self.create_timer(0.5, self.publish_state)

    # ---- helpers ----
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def age(self, stamp):
        return float('inf') if stamp is None else self.now_s() - stamp

    def manual_active(self):
        return self.age(self.last_manual_rx) < self.manual_timeout and not self.last_manual_zero

    def zero_cmd(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        return msg

    def publish_cmd(self, msg):
        self.last_out_zero = is_zero(msg)
        self.cmd_pub.publish(msg)

    def cancel_navigation(self):
        for action, client in self.cancel_clients.items():
            if self.active_goals[action] and client.service_is_ready():
                client.call_async(CancelGoal.Request())  # zero goal id = cancel all

    # ---- subscriptions ----
    def on_safe(self, msg):
        self.last_safe_rx = self.now_s()
        self.fault = False
        if self.estop or self.release_hold or self.transition_hold:
            self.publish_cmd(self.zero_cmd())
        else:
            self.publish_cmd(msg)

    def on_manual(self, msg):
        if self.synthetic_zeros:
            self.synthetic_zeros -= 1
            return
        self.last_manual_rx = self.now_s()
        self.last_manual_zero = is_zero(msg)
        if not self.estop and any(self.active_goals.values()):
            self.get_logger().info('Manual input: cancelling active navigation goal')
            self.cancel_navigation()

    def on_nav(self, _msg):
        self.last_nav_rx = self.now_s()

    def on_scan(self, _msg):
        self.last_scan_rx = self.now_s()

    def on_collision(self, msg):
        self.collision_action = msg.action_type

    def on_status(self, action, msg):
        self.active_goals[action] = any(s.status in ACTIVE for s in msg.status_list)
        if self.estop and self.active_goals[action]:
            self.cancel_navigation()

    # ---- services ----
    def srv_estop(self, _req, res):
        self.estop = True
        self.release_hold = False
        self.publish_cmd(self.zero_cmd())
        self.lock_pub.publish(Bool(data=True))
        self.cancel_navigation()
        self.get_logger().warning('EMERGENCY STOP engaged')
        res.success = True
        res.message = 'Emergency stop engaged'
        self.publish_state()
        return res

    def srv_release(self, _req, res):
        if not self.estop:
            res.success = True
            res.message = 'Emergency stop was not active'
            return res
        self.estop = False
        self.release_hold = True   # mux lock stays until the manual stream is quiet
        self.get_logger().warning('Emergency stop released; fresh command required')
        res.success = True
        res.message = 'Emergency stop released'
        self.publish_state()
        return res

    def srv_manual_stop(self, _req, res):
        self.manual_pub.publish(self.zero_cmd())
        if any(self.active_goals.values()):
            self.cancel_navigation()
        res.success = True
        res.message = 'Zero manual velocity sent'
        return res

    def srv_transition_hold(self, req, res):
        self.transition_hold = bool(req.data)
        if self.transition_hold:
            self.publish_cmd(self.zero_cmd())
            self.lock_pub.publish(Bool(data=True))
            self.cancel_navigation()
        res.success = True
        res.message = 'map transition motion hold enabled' if self.transition_hold else \
            'map transition motion hold released'
        self.publish_state()
        return res

    # ---- periodic ----
    def on_tick(self):
        if self.estop or self.transition_hold:
            self.publish_cmd(self.zero_cmd())
            self.lock_pub.publish(Bool(data=True))
            return

        if self.release_hold:
            if self.manual_active():
                self.lock_pub.publish(Bool(data=True))
                self.publish_cmd(self.zero_cmd())
                return
            self.release_hold = False
            self.get_logger().info('Manual stream quiet: motion re-enabled')
        self.lock_pub.publish(Bool(data=False))

        # Dead-man: take over the mux with zero so the smoother decelerates at once
        # instead of waiting for the mux and smoother timeouts to stack up.
        if not self.last_manual_zero and self.age(self.last_manual_rx) > self.manual_timeout:
            self.last_manual_zero = True
            self.synthetic_zeros += 1
            self.manual_pub.publish(self.zero_cmd())
            self.dead_man_count += 1
            self.last_dead_man = self.get_clock().now().to_msg()
            self.get_logger().info('Manual dead-man timeout: zero velocity')

        # DiffDrive holds its last command, so zero it if the pipeline goes silent.
        if not self.last_out_zero and self.age(self.last_safe_rx) > self.watchdog_timeout:
            self.get_logger().error('Velocity pipeline silent while moving: forcing zero')
            self.publish_cmd(self.zero_cmd())
            self.fault = True

    def publish_state(self):
        state = SafetyState()
        state.stamp = self.get_clock().now().to_msg()
        state.emergency_stop_active = self.estop
        state.dead_man_stop_count = self.dead_man_count
        if self.last_dead_man is not None:
            state.last_dead_man_stop = self.last_dead_man
        state.motion_inhibited = self.estop or self.release_hold or self.transition_hold
        if self.age(self.last_manual_rx) < self.manual_timeout:
            state.control_source = 'manual'
        elif self.age(self.last_nav_rx) < self.nav_timeout:
            state.control_source = 'nav'
        else:
            state.control_source = 'none'

        scan_stale = self.age(self.last_scan_rx) > self.scan_timeout
        if self.estop:
            state.state, state.detail = SafetyState.EMERGENCY_STOP, 'emergency stop engaged'
        elif self.transition_hold:
            state.state, state.detail = SafetyState.NORMAL, 'map transition motion hold'
        elif self.fault or scan_stale:
            state.state = SafetyState.FAULT
            state.detail = 'scan stale' if scan_stale else 'velocity pipeline silent'
        elif self.collision_action == CollisionMonitorState.STOP:
            state.state, state.detail = SafetyState.COLLISION_STOP, 'obstacle in stop zone'
        elif self.collision_action == CollisionMonitorState.SLOWDOWN:
            state.state, state.detail = SafetyState.CAUTION, 'obstacle in slow zone'
        else:
            state.state = SafetyState.NORMAL
            state.detail = 'waiting for manual input to go quiet' if self.release_hold else ''
        self.state_pub.publish(state)


def main():
    rclpy.init()
    node = SafetySupervisor()
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
