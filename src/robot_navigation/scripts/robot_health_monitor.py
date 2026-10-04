#!/usr/bin/python3
"""Robot diagnostics, aggregate health and structured errors.

/diagnostics          diagnostic_msgs/DiagnosticArray, one status per subsystem ("robot/<name>")
/robot/health         robot_interfaces/RobotHealth (overall OK/DEGRADED/ERROR/EMERGENCY + errors)

Health comes from observed data (message freshness, TF lookups, lifecycle states, Nav2 result
codes, safety/docking/battery state), never from "the process exists".
"""

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import DockingState as DS
from robot_interfaces.msg import RobotBatteryState as RB
from robot_interfaces.msg import MapState, RobotError, RobotHealth, RobotTelemetry, SafetyState
from sensor_msgs.msg import CameraInfo, Imu, JointState, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from nav_msgs.msg import Odometry

# Same numeric values as diagnostic_msgs/DiagnosticStatus (whose constants are bytes in rclpy).
OK, WARN, ERR, STALE = 0, 1, 2, 3

# Subsystems whose failure stops the robot from operating safely -> overall ERROR.
CRITICAL = {'lidar', 'odometry', 'tf', 'localization', 'map', 'navigation',
            'velocity_control', 'collision_monitor', 'emergency_stop'}
LIFECYCLE_NODES = ('bt_navigator', 'planner_server', 'controller_server', 'behavior_server',
                   'velocity_smoother', 'collision_monitor', 'docking_server', 'amcl',
                   'map_server')

# Nav2 result code -> stable robot error code (planner 2xx, controller 1xx, navigator 9xxx).
NAV_CODE_MAP = {
    203: 'GOAL_UNREACHABLE', 204: 'GOAL_UNREACHABLE', 206: 'GOAL_UNREACHABLE',
    208: 'GOAL_UNREACHABLE',
    205: 'PATH_BLOCKED', 104: 'PATH_BLOCKED', 105: 'PATH_BLOCKED', 106: 'PATH_BLOCKED',
    107: 'NAVIGATION_TIMEOUT', 108: 'NAVIGATION_TIMEOUT', 207: 'NAVIGATION_TIMEOUT',
    9003: 'NAVIGATION_TIMEOUT',
}
DOCK_CODE_MAP = {
    901: 'DOCK_NOT_FOUND', 902: 'DOCK_NOT_FOUND', 903: 'STAGING_FAILED',
    904: 'ALIGNMENT_FAILED', 905: 'ALIGNMENT_FAILED', 906: 'CHARGING_NOT_DETECTED',
    907: 'DOCKING_TIMEOUT',
}
# code -> (severity, recoverable)
ERROR_META = {
    'LOCALIZATION_UNAVAILABLE': (RobotError.ERROR, True),
    'LOCALIZATION_LOST': (RobotError.ERROR, True),
    'MAP_UNAVAILABLE': (RobotError.ERROR, False),
    'MAP_LOAD_FAILED': (RobotError.ERROR, True),
    'MAP_SAVE_FAILED': (RobotError.ERROR, True),
    'MAP_METADATA_INVALID': (RobotError.ERROR, True),
    'MAPPING_FAILED': (RobotError.ERROR, True),
    'NAVIGATION_UNAVAILABLE': (RobotError.ERROR, False),
    'NAVIGATION_FAILED': (RobotError.WARN, True),
    'NAVIGATION_TIMEOUT': (RobotError.WARN, True),
    'GOAL_UNREACHABLE': (RobotError.WARN, True),
    'PATH_BLOCKED': (RobotError.WARN, True),
    'ODOMETRY_UNAVAILABLE': (RobotError.ERROR, False),
    'LIDAR_UNAVAILABLE': (RobotError.ERROR, False),
    'IMU_UNAVAILABLE': (RobotError.WARN, False),
    'CAMERA_UNAVAILABLE': (RobotError.WARN, False),
    'DRIVETRAIN_UNAVAILABLE': (RobotError.WARN, False),
    'TF_UNAVAILABLE': (RobotError.ERROR, False),
    'MANUAL_CONTROL_TIMEOUT': (RobotError.INFO, True),
    'COLLISION_STOP': (RobotError.WARN, True),
    'EMERGENCY_STOP': (RobotError.FATAL, True),
    'SAFETY_COMPONENT_FAULT': (RobotError.ERROR, False),
    'VELOCITY_CONTROL_UNAVAILABLE': (RobotError.ERROR, False),
    'DOCKING_FAILED': (RobotError.WARN, True),
    'DOCK_NOT_FOUND': (RobotError.WARN, True),
    'STAGING_FAILED': (RobotError.WARN, True),
    'ALIGNMENT_FAILED': (RobotError.WARN, True),
    'DOCKING_TIMEOUT': (RobotError.WARN, True),
    'CHARGING_NOT_DETECTED': (RobotError.WARN, True),
    'UNDOCK_FAILED': (RobotError.WARN, True),
    'DOCKING_CANCELLED': (RobotError.WARN, True),
    'LOW_BATTERY': (RobotError.WARN, True),
    'CRITICAL_BATTERY': (RobotError.ERROR, True),
    'BATTERY_OVER_TEMPERATURE': (RobotError.ERROR, True),
    'CHARGING_FAILURE': (RobotError.WARN, True),
    'BATTERY_DATA_STALE': (RobotError.WARN, False),
    'BATTERY_FAULT': (RobotError.ERROR, False),
    'INTERNAL_ERROR': (RobotError.ERROR, False),
}


class Result:
    def __init__(self, level, message, code=None, source_code=0, severity=None, **kv):
        self.level, self.message, self.code = level, message, code
        self.source_code, self.severity, self.kv = source_code, severity, kv


class HealthMonitor(Node):
    def __init__(self):
        super().__init__('robot_health_monitor')
        d = self.declare_parameter
        self.grace = d('startup_grace', 8.0).value
        self.timeouts = {
            'lidar': d('scan_timeout', 1.0).value,
            'odometry': d('odom_timeout', 0.5).value,
            'imu': d('imu_timeout', 0.5).value,
            'camera': d('camera_timeout', 2.0).value,
            'drivetrain': d('joint_state_timeout', 1.0).value,
            'battery': d('battery_timeout', 3.0).value,
            'safety': d('safety_timeout', 2.0).value,
            'telemetry': d('telemetry_timeout', 2.0).value,
            'map_manager': d('map_manager_timeout', 5.0).value,
        }
        self.nav_error_hold = d('navigation_error_hold', 30.0).value
        self.dead_man_hold = d('dead_man_hold', 5.0).value
        self.lifecycle_period = d('lifecycle_poll_period', 2.0).value
        self.lifecycle_timeout = d('lifecycle_timeout', 6.0).value

        self.start = self.now_s()
        self.rx = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.health_pub = self.create_publisher(RobotHealth, '/robot/health', latched)

        # Freshness-only inputs use raw subscriptions: no deserialisation cost.
        for key, typ, topic in (('lidar', LaserScan, 'scan'), ('odometry', Odometry, 'odom'),
                                ('imu', Imu, 'imu'), ('camera', CameraInfo, 'camera/camera_info'),
                                ('drivetrain', JointState, 'joint_states')):
            self.create_subscription(typ, topic, lambda _m, k=key: self.touch(k), 10, raw=True)
        self.create_subscription(OccupancyGrid, '/map', lambda _m: self.touch('map'),
                                 latched, raw=True)
        self.telemetry = None
        self.safety = None
        self.docking = None
        self.battery_state = None
        self.map_state = None
        self.create_subscription(RobotTelemetry, '/robot/telemetry', self.on_telemetry, 10)
        self.create_subscription(SafetyState, '/robot/safety/state', self.on_safety, latched)
        self.create_subscription(DS, '/robot/docking/state', self.on_docking, latched)
        self.create_subscription(RB, '/robot/battery/state', self.on_battery, latched)
        self.create_subscription(MapState, '/robot/map/state', self.on_map_state, latched)

        self.life = {n: None for n in LIFECYCLE_NODES}   # name -> (state label, rx time)
        self.life_clients = {n: self.create_client(GetState, f'/{n}/get_state')
                             for n in LIFECYCLE_NODES}
        self.life_pending = set()

        self.nav_fail = None             # (robot code, nav2 code, message, time)
        self.last_nav_state = None
        self.dead_man_count = None
        self.dead_man_at = None

        self.create_timer(1.0, self.on_tick)
        self.create_timer(self.lifecycle_period, self.poll_lifecycle)

    # ---- inputs ----
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def touch(self, key):
        self.rx[key] = self.now_s()

    def age(self, key):
        t = self.rx.get(key)
        return None if t is None else self.now_s() - t

    def on_telemetry(self, msg):
        self.telemetry = msg
        self.touch('telemetry')
        if msg.navigation_state != self.last_nav_state:
            if msg.navigation_state == 'FAILED':
                code = NAV_CODE_MAP.get(msg.navigation_error_code, 'NAVIGATION_FAILED')
                self.nav_fail = (code, msg.navigation_error_code,
                                 msg.navigation_error_msg or 'navigation failed', self.now_s())
            elif msg.navigation_state in ('PLANNING', 'SUCCEEDED'):
                self.nav_fail = None
            self.last_nav_state = msg.navigation_state

    def on_safety(self, msg):
        self.safety = msg
        self.touch('safety')
        if self.dead_man_count is None:
            self.dead_man_count = msg.dead_man_stop_count
        elif msg.dead_man_stop_count > self.dead_man_count:
            self.dead_man_count = msg.dead_man_stop_count
            self.dead_man_at = self.now_s()

    def on_docking(self, msg):
        self.docking = msg
        self.touch('docking')

    def on_battery(self, msg):
        self.battery_state = msg
        self.touch('battery')

    def on_map_state(self, msg):
        self.map_state = msg
        self.touch('map_manager')

    def poll_lifecycle(self):
        for name, client in self.life_clients.items():
            if name in self.life_pending or not client.service_is_ready():
                continue
            self.life_pending.add(name)
            client.call_async(GetState.Request()).add_done_callback(
                lambda f, n=name: self.on_lifecycle(n, f))

    def on_lifecycle(self, name, future):
        self.life_pending.discard(name)
        res = future.result()
        if res is not None:
            self.life[name] = (res.current_state.label, self.now_s())

    def lifecycle(self, name):
        """-> (active: bool, description)"""
        entry = self.life.get(name)
        if entry is None or self.now_s() - entry[1] > self.lifecycle_timeout:
            return False, 'not responding'
        return entry[0] == 'active', entry[0]

    # ---- helpers ----
    def starting(self):
        return self.now_s() - self.start < self.grace

    def freshness(self, key, label, code):
        age = self.age(key)
        timeout = self.timeouts[key]
        if age is None:
            if self.starting():
                return Result(WARN, f'{label}: waiting for first message')
            return Result(STALE, f'{label}: no data received', code)
        if age > timeout:
            return Result(STALE, f'{label}: stale ({age:.1f} s)', code, age_s=f'{age:.2f}')
        return Result(OK, f'{label}: OK', age_s=f'{age:.2f}')

    def tf_ok(self, target, source):
        try:
            self.tf_buffer.lookup_transform(target, source, rclpy.time.Time(),
                                            timeout=Duration(seconds=0.0))
            return True
        except TransformException:
            return False

    # ---- evaluation ----
    def evaluate(self):
        r = {}
        r['lidar'] = self.freshness('lidar', '/scan', 'LIDAR_UNAVAILABLE')
        r['odometry'] = self.freshness('odometry', '/odom', 'ODOMETRY_UNAVAILABLE')
        r['imu'] = self.freshness('imu', '/imu', 'IMU_UNAVAILABLE')
        r['camera'] = self.freshness('camera', 'camera', 'CAMERA_UNAVAILABLE')
        r['drivetrain'] = self.freshness('drivetrain', 'joint states', 'DRIVETRAIN_UNAVAILABLE')
        r['tf'] = self.eval_tf()
        r['map'] = self.eval_map()
        r['localization'] = self.eval_localization()
        r['navigation'] = self.eval_navigation()
        r['velocity_control'] = self.eval_velocity()
        r['collision_monitor'] = self.eval_collision_monitor()
        r['emergency_stop'] = self.eval_estop()
        r['docking'] = self.eval_docking()
        r['battery'] = self.eval_battery()
        r['telemetry'] = self.eval_telemetry()
        return r

    def eval_tf(self):
        missing = [f'{a}->{b}' for a, b in (('map', 'odom'), ('odom', 'base_footprint'),
                                            ('base_footprint', 'lidar_link'))
                   if not self.tf_ok(a, b)]
        if missing:
            if self.starting():
                return Result(WARN, 'TF: waiting for transforms')
            return Result(ERR, 'TF unavailable: ' + ', '.join(missing), 'TF_UNAVAILABLE',
                          missing=','.join(missing))
        return Result(OK, 'TF: map->odom->base_footprint->lidar_link available')

    def eval_map(self):
        age = self.age('map_manager')
        if age is None or age > self.timeouts['map_manager']:
            if age is None and self.starting():
                return Result(WARN, 'map manager: starting')
            return Result(STALE, 'map manager state stale', 'MAP_UNAVAILABLE')
        state = self.map_state
        if state.state == MapState.ERROR:
            code = state.error_code if state.error_code in ERROR_META else 'MAP_LOAD_FAILED'
            return Result(ERR, state.detail or 'map lifecycle error', code,
                          active_map=state.active_map_id)
        if state.error_code in ('MAP_LOAD_FAILED', 'MAP_SAVE_FAILED',
                                'MAP_METADATA_INVALID', 'MAPPING_FAILED'):
            return Result(WARN, state.detail, state.error_code,
                          active_map=state.active_map_id)
        if state.mode == MapState.MODE_MAPPING:
            return Result(OK, 'SLAM mapping active', mode='MAPPING')
        if state.state != MapState.ACTIVE:
            return Result(WARN, state.detail or 'map transition active',
                          active_map=state.active_map_id)
        active, desc = self.lifecycle('map_server')
        if self.age('map') is None:
            if self.starting():
                return Result(WARN, 'map: waiting')
            return Result(ERR, 'no /map received', 'MAP_UNAVAILABLE', map_server=desc)
        if not active and not self.starting():
            return Result(WARN, f'map_server {desc}; last map retained', map_server=desc)
        name = state.active_map_id
        return Result(OK, f'map: {name or "loaded"}', active_map=name)

    def eval_localization(self):
        if self.map_state is not None and self.map_state.mode == MapState.MODE_MAPPING:
            return Result(OK, 'mapping mode; AMCL intentionally inactive', mode='MAPPING')
        active, desc = self.lifecycle('amcl')
        if not active:
            if self.starting():
                return Result(WARN, 'localization: starting')
            return Result(ERR, f'AMCL {desc}', 'LOCALIZATION_UNAVAILABLE', amcl=desc)
        t = self.telemetry
        if t is None or self.age('telemetry') is None or self.age('telemetry') > self.timeouts['telemetry']:
            return Result(STALE, 'localization state unknown (telemetry stale)')
        s = t.localization_state
        if s == 'LOCALIZED':
            return Result(OK, 'localized', state=s)
        if s == 'DEGRADED':
            return Result(WARN, 'localization degraded (high AMCL covariance)', state=s)
        if s == 'UNINITIALIZED':
            if self.starting():
                return Result(WARN, 'localization: initialising', state=s)
            return Result(ERR, 'localization not initialised', 'LOCALIZATION_UNAVAILABLE', state=s)
        return Result(ERR, 'localization lost', 'LOCALIZATION_LOST', state=s)

    def eval_navigation(self):
        down = []
        for n in ('bt_navigator', 'planner_server', 'controller_server', 'behavior_server'):
            active, desc = self.lifecycle(n)
            if not active:
                down.append(f'{n} {desc}')
        if down:
            if self.starting():
                return Result(WARN, 'navigation: starting')
            return Result(ERR, 'Nav2 unavailable: ' + '; '.join(down), 'NAVIGATION_UNAVAILABLE',
                          inactive=','.join(down))
        if self.nav_fail and self.now_s() - self.nav_fail[3] < self.nav_error_hold:
            code, nav_code, msg, _ = self.nav_fail
            return Result(WARN, f'last goal failed: {msg}', code, source_code=nav_code,
                          nav2_error_code=str(nav_code))
        return Result(OK, 'Nav2 active',
                      state=self.telemetry.navigation_state if self.telemetry else 'unknown')

    def eval_velocity(self):
        active, desc = self.lifecycle('velocity_smoother')
        nodes = set(self.get_node_names())
        missing = [n for n in ('twist_mux', 'safety_supervisor') if n not in nodes]
        if not active or missing:
            if self.starting():
                return Result(WARN, 'velocity control: starting')
            what = ([f'velocity_smoother {desc}'] if not active else []) + [f'{m} missing' for m in missing]
            return Result(ERR, 'velocity pipeline broken: ' + '; '.join(what),
                          'VELOCITY_CONTROL_UNAVAILABLE')
        s = self.safety
        if s is not None and s.state == SafetyState.FAULT and 'pipeline' in s.detail:
            return Result(ERR, f'velocity pipeline fault: {s.detail}', 'SAFETY_COMPONENT_FAULT')
        if self.dead_man_at is not None and self.now_s() - self.dead_man_at < self.dead_man_hold:
            return Result(WARN, 'manual dead-man timeout stop', 'MANUAL_CONTROL_TIMEOUT',
                          dead_man_stops=str(self.dead_man_count))
        return Result(OK, 'velocity pipeline OK',
                      control_source=s.control_source if s else 'unknown')

    def eval_collision_monitor(self):
        active, desc = self.lifecycle('collision_monitor')
        if not active:
            if self.starting():
                return Result(WARN, 'collision monitor: starting')
            return Result(ERR, f'collision monitor {desc}', 'SAFETY_COMPONENT_FAULT')
        age = self.age('safety')
        if age is None or age > self.timeouts['safety']:
            return Result(STALE, 'safety supervisor state stale', 'SAFETY_COMPONENT_FAULT')
        s = self.safety
        if s.state == SafetyState.COLLISION_STOP:
            return Result(WARN, 'protective stop: obstacle in stop zone (collision monitor working)',
                          'COLLISION_STOP', state='COLLISION_STOP')
        if s.state == SafetyState.CAUTION:
            return Result(OK, 'slowing for obstacle', state='CAUTION')
        if s.state == SafetyState.FAULT and 'scan' in s.detail:
            return Result(ERR, f'safety fault: {s.detail}', 'LIDAR_UNAVAILABLE')
        return Result(OK, 'collision monitor active', state='NORMAL')

    def eval_estop(self):
        age = self.age('safety')
        if age is None or age > self.timeouts['safety']:
            if self.starting():
                return Result(WARN, 'e-stop: waiting for safety supervisor')
            return Result(STALE, 'safety supervisor not reporting', 'SAFETY_COMPONENT_FAULT')
        s = self.safety
        if s.emergency_stop_active:
            return Result(ERR, 'EMERGENCY STOP engaged', 'EMERGENCY_STOP', severity=RobotError.FATAL)
        if s.motion_inhibited:
            return Result(WARN, 'e-stop released; waiting for manual input to go quiet')
        return Result(OK, 'e-stop not engaged')

    def eval_docking(self):
        age = self.age('docking')
        if age is None:
            return Result(WARN if self.starting() else STALE, 'docking manager not reporting',
                          None if self.starting() else 'DOCKING_FAILED')
        d = self.docking
        active, desc = self.lifecycle('docking_server')
        kv = dict(state=str(d.state), error_code=str(d.error_code), docked=str(d.is_docked))
        if not active and not self.starting():
            return Result(ERR, f'docking server {desc}', 'DOCKING_FAILED', **kv)
        if d.state == DS.FAILED:
            if d.detail.startswith('undock'):
                code = 'UNDOCK_FAILED'
            else:
                code = DOCK_CODE_MAP.get(d.error_code, 'DOCKING_FAILED')
            return Result(WARN, d.error_msg or d.detail, code, source_code=d.error_code, **kv)
        if d.state == DS.CANCELLED:
            if 'externally' in d.detail:
                return Result(WARN, d.detail, 'DOCKING_CANCELLED', **kv)
            return Result(OK, d.detail, **kv)
        return Result(OK, d.detail or ('docked' if d.is_docked else 'idle'), **kv)

    def eval_battery(self):
        age = self.age('battery')
        if age is None or age > self.timeouts['battery']:
            if age is None and self.starting():
                return Result(WARN, 'battery: waiting for data')
            return Result(STALE, 'battery data stale', 'BATTERY_DATA_STALE')
        b = self.battery_state
        if b is None:
            return Result(STALE, 'battery state unavailable', 'BATTERY_DATA_STALE')
        kv = dict(percentage=f'{b.percentage:.1f}', voltage=f'{b.voltage:.2f}',
                  current=f'{b.current:.2f}', temperature=f'{b.temperature:.1f}')
        if b.state == RB.FAULT:
            if 'temperature' in b.detail:
                return Result(ERR, b.detail, 'BATTERY_OVER_TEMPERATURE', **kv)
            if 'contact' in b.detail:
                return Result(WARN, b.detail, 'CHARGING_FAILURE', **kv)
            return Result(ERR, b.detail or 'battery fault', 'BATTERY_FAULT', **kv)
        if b.state == RB.CRITICAL:
            return Result(ERR, f'critical battery {b.percentage:.0f}%', 'CRITICAL_BATTERY', **kv)
        if b.state == RB.LOW:
            return Result(WARN, f'low battery {b.percentage:.0f}%', 'LOW_BATTERY', **kv)
        if b.state == RB.UNKNOWN:
            return Result(STALE, b.detail or 'battery state unknown', 'BATTERY_DATA_STALE', **kv)
        names = {RB.CHARGING: 'charging', RB.FULL: 'full', RB.DISCHARGING: 'discharging'}
        return Result(OK, f'{names.get(b.state, "ok")} {b.percentage:.0f}%', **kv)

    def eval_telemetry(self):
        age = self.age('telemetry')
        if age is None or age > self.timeouts['telemetry']:
            if age is None and self.starting():
                return Result(WARN, 'telemetry: starting')
            return Result(STALE, 'robot_state_manager not publishing', 'INTERNAL_ERROR')
        return Result(OK, 'telemetry OK', age_s=f'{age:.2f}')

    # ---- publish ----
    def on_tick(self):
        results = self.evaluate()
        now = self.get_clock().now().to_msg()

        diag = DiagnosticArray()
        diag.header.stamp = now
        health = RobotHealth()
        health.stamp = now
        overall = RobotHealth.OK

        for name, res in results.items():
            st = DiagnosticStatus()
            st.level = bytes([res.level])
            st.name = f'robot/{name}'
            st.message = res.message
            st.hardware_id = 'service_robot'
            kv = dict(res.kv)
            if res.code:
                kv['error_code'] = res.code
            st.values = [KeyValue(key=k, value=str(v)) for k, v in kv.items()]
            diag.status.append(st)

            health.subsystems.append(name)
            health.subsystem_levels.append(res.level)

            if res.code and res.level >= WARN:
                sev, rec = ERROR_META.get(res.code, (RobotError.ERROR, False))
                e = RobotError()
                e.code, e.subsystem, e.message = res.code, name, res.message
                e.severity = res.severity if res.severity is not None else sev
                e.recoverable, e.stamp, e.source_code = rec, now, res.source_code
                health.active_errors.append(e)

            if name == 'emergency_stop' and res.code == 'EMERGENCY_STOP':
                overall = RobotHealth.EMERGENCY
            elif res.level >= ERR and name in CRITICAL:
                overall = max(overall, RobotHealth.ERROR) if overall != RobotHealth.EMERGENCY else overall
            elif res.level >= WARN and overall == RobotHealth.OK:
                overall = RobotHealth.DEGRADED
            elif res.level >= ERR and overall == RobotHealth.OK:
                overall = RobotHealth.DEGRADED

        health.overall = overall
        self.diag_pub.publish(diag)
        self.health_pub.publish(health)


def main():
    rclpy.init()
    node = HealthMonitor()
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
