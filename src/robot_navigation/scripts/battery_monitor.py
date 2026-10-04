#!/usr/bin/python3
"""Semantic robot battery state + critical-battery return-to-dock policy.

/robot/battery (sensor_msgs/BatteryState) + /robot/dock/contact -> /robot/battery/state.
Hardware-agnostic: the sim power system and a real BMS publish the same two inputs.
This is the ROBOT battery; it has nothing to do with any controller/tablet battery.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import DockingState as DS
from robot_interfaces.msg import RobotBatteryState as RB
from robot_interfaces.msg import SafetyState
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

NOT_DOCKABLE = (DS.NAVIGATING_TO_DOCK, DS.PRESTAGING, DS.ALIGNING, DS.DOCKING,
                DS.UNDOCKING, DS.DOCKED)


class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        self.low = self.declare_parameter('low_threshold', 20.0).value
        self.critical = self.declare_parameter('critical_threshold', 10.0).value
        self.hysteresis = self.declare_parameter('hysteresis', 5.0).value
        self.max_temp = self.declare_parameter('max_temperature_c', 60.0).value
        self.stale_s = self.declare_parameter('stale_timeout', 3.0).value
        self.charge_current = self.declare_parameter('charging_current_threshold', 0.2).value
        self.charge_fault_s = self.declare_parameter('charge_fault_time', 10.0).value
        self.retry_cooldown = self.declare_parameter('dock_request_cooldown', 30.0).value
        self.auto_dock = self.declare_parameter('auto_dock_on_critical', True).value

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(RB, '/robot/battery/state', latched)
        self.create_subscription(BatteryState, '/robot/battery', self.on_battery, 10)
        self.create_subscription(Bool, '/robot/dock/contact', self.on_contact, latched)
        self.create_subscription(DS, '/robot/docking/state', self.on_docking, latched)
        self.create_subscription(SafetyState, '/robot/safety/state', self.on_safety, latched)
        self.dock_client = self.create_client(Trigger, '/robot/dock')

        self.battery = None
        self.rx = None
        self.contact = False
        self.docking = None
        self.safety = None
        self.contact_no_charge_since = None
        self.dock_requested = False      # latched until docked or recovered past hysteresis
        self.last_request = None
        self.pending = False
        self.create_timer(0.5, self.on_tick)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_battery(self, msg):
        self.battery = msg
        self.rx = self.now_s()

    def on_contact(self, msg):
        self.contact = msg.data

    def on_docking(self, msg):
        self.docking = msg

    def on_safety(self, msg):
        self.safety = msg

    def classify(self):
        """Returns (state, is_charging, detail)."""
        b = self.battery
        age = float('inf') if self.rx is None else self.now_s() - self.rx
        if b is None or age > self.stale_s:
            return RB.UNKNOWN, False, 'battery data stale' if b is not None else 'no battery data'

        pct = b.percentage * 100.0
        charging = (self.contact and b.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
                    and b.current > self.charge_current)
        full = self.contact and b.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_FULL

        # Contacts touching without charge current is a charging failure, not normal charging.
        if self.contact and not charging and not full:
            if self.contact_no_charge_since is None:
                self.contact_no_charge_since = self.now_s()
        else:
            self.contact_no_charge_since = None

        if b.temperature > self.max_temp:
            return RB.FAULT, False, f'over temperature {b.temperature:.1f} C'
        if b.power_supply_health not in (BatteryState.POWER_SUPPLY_HEALTH_GOOD,
                                         BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN):
            return RB.FAULT, False, f'battery health code {b.power_supply_health}'
        if (self.contact_no_charge_since is not None
                and self.now_s() - self.contact_no_charge_since > self.charge_fault_s):
            return RB.FAULT, False, 'dock contact without charge current'
        if full:
            return RB.FULL, False, ''
        if charging:
            return RB.CHARGING, True, ''
        if pct <= self.critical:
            return RB.CRITICAL, False, ''
        if pct <= self.low:
            return RB.LOW, False, ''
        return RB.DISCHARGING, False, ''

    def on_tick(self):
        state, charging, detail = self.classify()
        self.apply_policy(state)

        msg = RB()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state = state
        msg.is_charging = charging
        msg.detail = detail
        if self.battery is not None:
            msg.percentage = self.battery.percentage * 100.0
            msg.voltage = self.battery.voltage
            msg.current = self.battery.current
            msg.temperature = self.battery.temperature
            msg.data_age_s = self.now_s() - self.rx
        self.pub.publish(msg)

    def apply_policy(self, state):
        pct = self.battery.percentage * 100.0 if self.battery else 100.0
        docked = self.docking is not None and self.docking.state == DS.DOCKED
        if self.dock_requested and (docked or pct > self.critical + self.hysteresis):
            self.dock_requested = False
        if state != RB.CRITICAL or not self.auto_dock or self.dock_requested or self.pending:
            return
        if docked or self.docking is None or self.docking.state in NOT_DOCKABLE:
            return
        if self.last_request is not None and self.now_s() - self.last_request < self.retry_cooldown:
            return
        s = self.safety
        if s is None or s.emergency_stop_active or s.motion_inhibited or s.state == SafetyState.FAULT:
            return   # safety takes precedence; retried after cooldown once it clears
        if not self.dock_client.service_is_ready():
            return
        self.last_request = self.now_s()
        self.pending = True
        self.get_logger().warning('Critical battery: requesting return to dock')
        self.dock_client.call_async(Trigger.Request()).add_done_callback(self.on_dock_response)

    def on_dock_response(self, future):
        self.pending = False
        res = future.result()
        if res is not None and res.success:
            self.dock_requested = True   # do not ask again until docked / recovered
        else:
            self.get_logger().warning(
                f'Dock request refused: {res.message if res else "no response"}')


def main():
    rclpy.init()
    node = BatteryMonitor()
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
