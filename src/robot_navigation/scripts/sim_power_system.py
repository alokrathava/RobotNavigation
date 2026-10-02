#!/usr/bin/python3
"""Simulation-only power system: battery model, charging-contact and dock-detector emulation.

Publishes /robot/battery (sensor_msgs/BatteryState), /robot/dock/contact (Bool) and
/detected_dock_pose (stand-in for an AprilTag/vision dock detector, in the odom frame).
On hardware these come from the BMS and the dock contact/current sensing instead; everything
downstream (docking server, battery_monitor, telemetry) only sees those two topics.

Charge/discharge integrate on /odom header-stamp deltas (simulation time) while the node itself runs on a
wall-time timer. Gazebo ground truth is used only to decide whether the contacts touch the dock.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SimPowerSystem(Node):
    def __init__(self):
        super().__init__('sim_power_system')
        d = self.declare_parameter
        self.initial_percentage = d('initial_percentage', 80.0).value
        d('capacity_ah', 20.0)
        d('idle_discharge_pct_per_min', 0.1)
        d('moving_discharge_pct_per_min', 0.6)
        d('charge_pct_per_min', 6.0)
        d('idle_current_a', -0.8)
        d('moving_current_a', -3.0)
        d('charge_current_a', 5.0)
        d('float_current_a', 0.3)
        d('temperature_baseline_c', 28.0)
        d('moving_speed_threshold', 0.02)
        d('moving_rate_threshold', 0.05)
        # Dock contact region, in the dock frame.
        d('dock_x', 0.0)
        d('dock_y', 0.0)
        d('dock_yaw', 0.0)
        d('contact_tol_x', 0.08)
        d('contact_tol_y', 0.06)
        d('contact_tol_yaw', 0.20)
        # Fault injection.
        d('set_percentage', -1.0)       # >=0 jumps the battery to this value once
        d('charger_fault', False)       # contacts touch but no charge current flows
        d('contact_fault', False)       # contacts never report touching
        d('temperature_offset_c', 0.0)
        d('publish_enabled', True)
        d('detector_enabled', True)
        d('detector_range', 3.0)
        d('detector_fov', 1.2)

        self.percentage = float(self.initial_percentage)
        self.sim_time = None
        self.last_sim_time = None
        self.moving = False
        self.gt_pose = None
        self.contact = False
        self.add_on_set_parameters_callback(self.on_params)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.batt_pub = self.create_publisher(BatteryState, '/robot/battery', 10)
        self.contact_pub = self.create_publisher(Bool, '/robot/dock/contact', latched)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Odometry, '/sim/ground_truth/odom', self.on_gt, 10)
        self.odom_pose = None
        self.detect_pub = self.create_publisher(PoseStamped, '/detected_dock_pose', 10)
        self.create_timer(0.2, self.on_tick)
        self.create_timer(0.1, self.on_detect)
        self.last_contact_pub = None

    def p(self, name):
        return self.get_parameter(name).value

    def on_params(self, params):
        for prm in params:
            if prm.name == 'set_percentage' and prm.value is not None and prm.value >= 0.0:
                self.percentage = min(100.0, float(prm.value))
                self.get_logger().info(f'Battery set to {self.percentage:.1f}%')
        return SetParametersResult(successful=True)

    def on_odom(self, msg):
        self.sim_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose
        self.odom_pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        v, w = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        self.moving = (abs(v) > self.p('moving_speed_threshold')
                       or abs(w) > self.p('moving_rate_threshold'))

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

    def on_detect(self):
        """Dock pose as a vision detector would report it: ground-truth relative pose of the
        dock seen from the robot, re-expressed in the odom frame so it is independent of AMCL."""
        if (self.gt_pose is None or self.odom_pose is None or self.sim_time is None
                or not self.p('detector_enabled')):
            return
        gx, gy, gyaw = self.gt_pose
        dx, dy = self.p('dock_x') - gx, self.p('dock_y') - gy
        c, s = math.cos(gyaw), math.sin(gyaw)
        rx, ry = c * dx + s * dy, -s * dx + c * dy
        ryaw = self.p('dock_yaw') - gyaw
        dist = math.hypot(rx, ry)
        if dist > self.p('detector_range') or abs(math.atan2(ry, rx)) > self.p('detector_fov'):
            return
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
        self.detect_pub.publish(msg)

    def on_tick(self):
        dt = 0.0
        if self.sim_time is not None and self.last_sim_time is not None:
            dt = max(0.0, min(self.sim_time - self.last_sim_time, 5.0))
        self.last_sim_time = self.sim_time

        self.contact = self.compute_contact()
        if self.last_contact_pub != self.contact:
            self.contact_pub.publish(Bool(data=self.contact))
            self.last_contact_pub = self.contact

        charging = self.contact and not self.p('charger_fault')
        if charging:
            if self.percentage < 100.0:
                self.percentage = min(100.0, self.percentage
                                      + self.p('charge_pct_per_min') * dt / 60.0)
        else:
            rate = self.p('moving_discharge_pct_per_min' if self.moving
                          else 'idle_discharge_pct_per_min')
            self.percentage = max(0.0, self.percentage - rate * dt / 60.0)

        full = self.percentage >= 100.0
        if charging:
            current = self.p('float_current_a') if full else self.p('charge_current_a')
            status = (BatteryState.POWER_SUPPLY_STATUS_FULL if full
                      else BatteryState.POWER_SUPPLY_STATUS_CHARGING)
        else:
            current = self.p('moving_current_a' if self.moving else 'idle_current_a')
            status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        temp = (self.p('temperature_baseline_c') + 0.1 * current * current
                + self.p('temperature_offset_c'))
        health = (BatteryState.POWER_SUPPLY_HEALTH_OVERHEAT if temp > 60.0
                  else BatteryState.POWER_SUPPLY_HEALTH_GOOD)

        if not self.p('publish_enabled'):
            return
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.voltage = 21.0 + 4.2 * self.percentage / 100.0 + (0.3 if charging else 0.0)
        msg.temperature = temp
        msg.current = current
        msg.charge = self.p('capacity_ah') * self.percentage / 100.0
        msg.capacity = self.p('capacity_ah')
        msg.design_capacity = self.p('capacity_ah')
        msg.percentage = self.percentage / 100.0
        msg.power_supply_status = status
        msg.power_supply_health = health
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True
        self.batt_pub.publish(msg)


def main():
    rclpy.init()
    node = SimPowerSystem()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
