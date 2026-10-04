#!/usr/bin/python3
"""Simulation backend for /robot/battery (sensor_msgs/BatteryState).

On hardware the BMS driver publishes this topic. The model charges whenever /robot/dock/contact
reports contact (it knows nothing about Gazebo or the dock pose). Charge/discharge integrate on
/odom header-stamp deltas (simulation time) while the node itself runs on a wall-time timer.
"""

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool


class SimBattery(Node):
    def __init__(self):
        super().__init__('sim_battery')
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
        # Fault injection.
        d('set_percentage', -1.0)       # >=0 jumps the battery to this value once
        d('charger_fault', False)       # contacts touch but no charge current flows
        d('temperature_offset_c', 0.0)
        d('publish_enabled', True)

        self.percentage = float(self.initial_percentage)
        self.sim_time = None
        self.last_sim_time = None
        self.moving = False
        self.contact = False
        self.add_on_set_parameters_callback(self.on_params)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(BatteryState, '/robot/battery', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Bool, '/robot/dock/contact', self.on_contact, latched)
        self.create_timer(0.2, self.on_tick)

    def p(self, name):
        return self.get_parameter(name).value

    def on_params(self, params):
        for prm in params:
            if prm.name == 'set_percentage' and prm.value is not None and prm.value >= 0.0:
                self.percentage = min(100.0, float(prm.value))
                self.get_logger().info(f'Battery set to {self.percentage:.1f}%')
        return SetParametersResult(successful=True)

    def on_contact(self, msg):
        self.contact = msg.data

    def on_odom(self, msg):
        self.sim_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        v, w = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        self.moving = (abs(v) > self.p('moving_speed_threshold')
                       or abs(w) > self.p('moving_rate_threshold'))

    def on_tick(self):
        dt = 0.0
        if self.sim_time is not None and self.last_sim_time is not None:
            dt = max(0.0, min(self.sim_time - self.last_sim_time, 5.0))
        self.last_sim_time = self.sim_time

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
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SimBattery()
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
