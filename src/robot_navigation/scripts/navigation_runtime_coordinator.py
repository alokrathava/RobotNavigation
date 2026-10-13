#!/usr/bin/python3
"""Starts the Nav2 frontend only after every BT dependency is actually discoverable."""

import time

import rclpy
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import (
    BackUp, ComputePathThroughPoses, ComputePathToPose, DockRobot, DriveOnHeading,
    FollowPath, NavigateThroughPoses, NavigateToPose, Spin, UndockRobot, Wait,
)
from nav2_msgs.srv import ClearEntireCostmap, IsPathValid
from rclpy.action import ActionClient
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import MapState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger


LOCALIZATION_NODES = ('map_server', 'amcl')
CORE_NODES = (
    'controller_server', 'planner_server', 'behavior_server',
    'velocity_smoother', 'collision_monitor')
FRONTEND_NODES = ('bt_navigator', 'docking_server')
REQUIRED_ACTIONS = {
    '/compute_path_to_pose': ComputePathToPose,
    '/compute_path_through_poses': ComputePathThroughPoses,
    '/follow_path': FollowPath,
    '/spin': Spin,
    '/drive_on_heading': DriveOnHeading,
    '/wait': Wait,
    '/backup': BackUp,
}
FRONTEND_ACTIONS = {
    '/navigate_to_pose': NavigateToPose,
    '/navigate_through_poses': NavigateThroughPoses,
    '/dock_robot': DockRobot,
    '/undock_robot': UndockRobot,
}
REQUIRED_SERVICES = {
    '/is_path_valid': IsPathValid,
    '/global_costmap/clear_entirely_global_costmap': ClearEntireCostmap,
    '/local_costmap/clear_entirely_local_costmap': ClearEntireCostmap,
}


class NavigationRuntimeCoordinator(Node):
    def __init__(self):
        super().__init__('navigation_runtime_coordinator')
        self.enabled = True
        self.frontend_ever_started = False
        self.manage_pending = None
        self.completed_commands = {}
        self.lifecycle_failed = False
        self.state_pending = {}
        self.states = {}
        self.last_ready = None
        self.last_wait_log = 0.0
        self.bootstrap_ok = False
        self.bootstrap_pending = None
        self.map_state = None

        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.ready_pub = self.create_publisher(
            Bool, '/robot/navigation/runtime_ready', latched)
        self.create_service(SetBool, '/robot/navigation/set_enabled', self.set_enabled)
        self.bootstrap_client = self.create_client(
            Trigger, '/robot/maps/bootstrap_ready')
        self.create_subscription(MapState, '/robot/map/state', self.on_map_state, latched)
        all_nodes = LOCALIZATION_NODES + CORE_NODES + FRONTEND_NODES
        self.state_clients = {
            name: self.create_client(GetState, f'/{name}/get_state') for name in all_nodes}
        self.manage_clients = {
            'localization': self.create_client(
                ManageLifecycleNodes, '/lifecycle_manager_localization/manage_nodes'),
            'core': self.create_client(
                ManageLifecycleNodes, '/lifecycle_manager_navigation_core/manage_nodes'),
            'frontend': self.create_client(
                ManageLifecycleNodes, '/lifecycle_manager_navigation_frontend/manage_nodes'),
        }
        self.action_clients = {
            name: ActionClient(self, action_type, name)
            for name, action_type in REQUIRED_ACTIONS.items()
        }
        self.frontend_action_clients = {
            name: ActionClient(self, action_type, name)
            for name, action_type in FRONTEND_ACTIONS.items()
        }
        self.dependency_service_clients = {
            name: self.create_client(service_type, name)
            for name, service_type in REQUIRED_SERVICES.items()
        }
        self.create_timer(0.2, self.tick)
        self.publish_ready(False)

    def publish_ready(self, ready):
        if ready != self.last_ready:
            self.last_ready = ready
            self.ready_pub.publish(Bool(data=ready))
            self.get_logger().info(f'navigation_runtime_ready={str(ready).lower()}')

    def on_map_state(self, msg):
        self.map_state = msg

    def bootstrap_result(self, future):
        if self.bootstrap_pending is None or self.bootstrap_pending[0] is not future:
            return
        self.bootstrap_pending = None
        try:
            response = future.result()
            self.bootstrap_ok = bool(response.success)
            if not response.success:
                self.get_logger().error(f'Map bootstrap failed: {response.message}')
        except Exception as exc:
            self.get_logger().error(f'Map bootstrap request failed: {exc}')

    def set_enabled(self, request, response):
        self.enabled = request.data
        self.publish_ready(False)
        if not self.enabled and self.frontend_active():
            self.request_manage('frontend', ManageLifecycleNodes.Request.PAUSE)
        response.success = True
        response.message = 'navigation runtime enable requested' if self.enabled else \
            'navigation runtime disable requested'
        return response

    def state_result(self, name, future):
        pending = self.state_pending.get(name)
        if pending is None or pending[0] is not future:
            return
        self.state_pending.pop(name, None)
        try:
            self.states[name] = future.result().current_state.label
        except Exception as exc:
            self.states[name] = 'unavailable'
            self.get_logger().warning(f'Cannot read {name} lifecycle state: {exc}')

    def poll_states(self):
        now = time.monotonic()
        for name, client in self.state_clients.items():
            pending = self.state_pending.get(name)
            if pending is not None and now - pending[1] <= 2.0:
                continue
            if pending is not None:
                self.state_pending.pop(name, None)
                pending[0].cancel()
                self.states[name] = 'unavailable'
                self.get_logger().warning(f'Timed out reading {name} lifecycle state; retrying')
            if not client.service_is_ready():
                self.states[name] = 'unavailable'
                continue
            future = client.call_async(GetState.Request())
            self.state_pending[name] = (future, now)
            future.add_done_callback(
                lambda future, node_name=name: self.state_result(node_name, future))

    def nodes_active(self, names):
        return all(self.states.get(name) == 'active' for name in names)

    def frontend_active(self):
        return self.nodes_active(FRONTEND_NODES)

    def dependencies_ready(self):
        if not self.nodes_active(LOCALIZATION_NODES + CORE_NODES):
            return False
        return (all(client.server_is_ready() for client in self.action_clients.values())
                and all(client.service_is_ready()
                        for client in self.dependency_service_clients.values()))

    def log_frontend_wait(self):
        now = time.monotonic()
        if now - self.last_wait_log < 5.0:
            return
        self.last_wait_log = now
        missing_actions = [name for name, client in self.action_clients.items()
                           if not client.server_is_ready()]
        missing_services = [name for name, client in self.dependency_service_clients.items()
                            if not client.service_is_ready()]
        inactive = [name for name in LOCALIZATION_NODES + CORE_NODES
                    if self.states.get(name) != 'active']
        self.get_logger().info(
            'Waiting to start navigation frontend; inactive=%s missing_actions=%s '
            'missing_services=%s' % (inactive, missing_actions, missing_services))

    def request_manage(self, manager, command):
        client = self.manage_clients[manager]
        if (self.lifecycle_failed or self.manage_pending is not None
                or self.completed_commands.get(manager) == command
                or not client.service_is_ready()):
            return
        request = ManageLifecycleNodes.Request(command=command)
        future = client.call_async(request)
        self.manage_pending = (manager, command, future)
        future.add_done_callback(
            lambda future, owner=manager, requested=command:
            self.manage_result(owner, requested, future))

    def manage_result(self, manager, command, future):
        pending = self.manage_pending
        if pending is None or pending[2] is not future:
            return
        self.manage_pending = None
        try:
            success = future.result().success
        except Exception as exc:
            self.lifecycle_failed = True
            self.get_logger().error(f'{manager} lifecycle request failed: {exc}')
            return
        if not success:
            self.lifecycle_failed = True
            self.get_logger().error(
                f'{manager} lifecycle command {command} was rejected; bringup stopped')
            return
        # A successful manager response is authoritative. GetState replies can still
        # describe an earlier transition; never STARTUP an already-started group again.
        self.completed_commands[manager] = command
        if (manager == 'frontend' and
                command in (ManageLifecycleNodes.Request.STARTUP,
                            ManageLifecycleNodes.Request.RESUME)):
            self.frontend_ever_started = True

    def tick(self):
        # Cancelling a client future does not cancel the manager's lifecycle operation.
        # Keep it in flight until completion instead of queuing duplicate transitions.
        self.poll_states()
        if self.lifecycle_failed:
            self.publish_ready(False)
            return
        if not self.bootstrap_ok:
            if (self.bootstrap_pending is not None
                    and time.monotonic() - self.bootstrap_pending[1] > 5.0):
                self.bootstrap_pending[0].cancel()
                self.bootstrap_pending = None
                self.get_logger().warning('Map bootstrap response timed out; retrying')
            if self.bootstrap_pending is None and self.bootstrap_client.service_is_ready():
                future = self.bootstrap_client.call_async(Trigger.Request())
                self.bootstrap_pending = (future, time.monotonic())
                future.add_done_callback(self.bootstrap_result)
            self.publish_ready(False)
            return
        map_ready = (self.map_state is not None
                     and self.map_state.state == MapState.ACTIVE
                     and self.map_state.mode == MapState.MODE_LOCALIZATION
                     and self.map_state.localization_ready)
        ready = (self.enabled and self.manage_pending is None and self.frontend_active()
                 and map_ready
                 and all(client.server_is_ready()
                         for client in self.frontend_action_clients.values())
                 and self.dependencies_ready())
        self.publish_ready(ready)
        if not self.enabled:
            if self.frontend_active():
                self.request_manage('frontend', ManageLifecycleNodes.Request.PAUSE)
            if (self.map_state is not None and self.map_state.mode == MapState.MODE_MAPPING
                    and not self.nodes_active(CORE_NODES)):
                self.request_manage('core', ManageLifecycleNodes.Request.STARTUP)
            return
        if not self.nodes_active(LOCALIZATION_NODES):
            self.log_frontend_wait()
            self.request_manage('localization', ManageLifecycleNodes.Request.STARTUP)
            return
        if not self.nodes_active(CORE_NODES):
            self.log_frontend_wait()
            self.request_manage('core', ManageLifecycleNodes.Request.STARTUP)
            return
        if not self.frontend_active() and self.dependencies_ready():
            command = (ManageLifecycleNodes.Request.RESUME
                       if self.frontend_ever_started
                       else ManageLifecycleNodes.Request.STARTUP)
            self.request_manage('frontend', command)
        elif not self.frontend_active():
            self.log_frontend_wait()


def main():
    rclpy.init()
    node = NavigationRuntimeCoordinator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
