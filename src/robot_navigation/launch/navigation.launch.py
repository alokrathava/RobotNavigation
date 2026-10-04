import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import ComposableNodeContainer, LifecycleNode, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Navigation and safety stack, with simulation for standalone launches.

    robot_bringup/robot.launch.py supplies its own simulator and sets start_simulator:=false.
    """
    nav_share = get_package_share_directory('robot_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')
    backend = LaunchConfiguration('backend')
    mode = LaunchConfiguration('mode')
    maps_dir = LaunchConfiguration('maps_dir')
    map_file = PathJoinSubstitution([maps_dir, 'main_floor', 'map.yaml'])

    def params(file_arg):
        """Parameter file with every use_sim_time (incl. nested costmap namespaces) overridden."""
        return ParameterFile(
            RewrittenYaml(source_file=file_arg, root_key='',
                          param_rewrites={'use_sim_time': use_sim_time}, convert_types=True),
            allow_substs=True)

    def config(name):
        return os.path.join(nav_share, 'config', name)

    amcl_params = params(LaunchConfiguration('amcl_params_file'))
    nav2_params = params(LaunchConfiguration('nav2_params_file'))
    slam_params = params(LaunchConfiguration('slam_params_file'))
    smoother_params = params(config('velocity_smoother.yaml'))
    collision_params = params(config('collision_monitor.yaml'))

    # --- localization: map_server + amcl (map -> odom) ---
    # In mapping mode the localization manager does not autostart, so AMCL stays unconfigured
    # and SLAM Toolbox is the only map -> odom publisher.
    map_server = Node(
        package='nav2_map_server', executable='map_server',
        name='map_server',
        output='screen',
        parameters=[amcl_params, {'yaml_filename': map_file}],
    )
    amcl = Node(
        package='nav2_amcl', executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params],
    )
    lifecycle_localization = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': False,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    # MapManager lifecycle-manages SLAM; it remains unconfigured in localization mode.
    slam = LifecycleNode(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', namespace='', output='screen',
        parameters=[slam_params, {'use_lifecycle_manager': False,
                                  'use_sim_time': use_sim_time}],
    )
    map_manager = Node(
        package='robot_navigation', executable='map_manager.py',
        name='map_manager', output='screen',
        parameters=[{
            'use_sim_time': False,
            'maps_dir': maps_dir,
            'bundled_map_yaml': os.path.join(nav_share, 'maps', 'main_floor.yaml'),
            'default_map_id': 'main_floor',
        }],
    )

    # --- navigation: controller/planner/behaviors/bt_navigator ---
    # Controller and behaviors publish /cmd_vel_nav only; the safety pipeline
    # below produces the final /cmd_vel consumed by the drivetrain.
    to_nav = [('cmd_vel', 'cmd_vel_nav')]
    controller = ComposableNode(
        package='nav2_controller', plugin='nav2_controller::ControllerServer',
        name='controller_server',
        parameters=[nav2_params], remappings=to_nav,
    )
    planner = ComposableNode(
        package='nav2_planner', plugin='nav2_planner::PlannerServer',
        name='planner_server',
        parameters=[nav2_params],
    )
    behaviors = ComposableNode(
        package='nav2_behaviors', plugin='behavior_server::BehaviorServer',
        name='behavior_server',
        parameters=[nav2_params], remappings=to_nav,
    )
    bt_navigator = ComposableNode(
        package='nav2_bt_navigator', plugin='nav2_bt_navigator::BtNavigator',
        name='bt_navigator',
        parameters=[nav2_params],
    )
    docking_server = ComposableNode(
        package='opennav_docking', plugin='opennav_docking::DockingServer',
        name='docking_server',
        parameters=[nav2_params], remappings=to_nav + [('battery_state', '/robot/battery'),
                             ('detected_dock_pose', '/robot/dock/detected_pose')],
    )
    # Pipeline: twist_mux -> velocity_smoother -> collision_monitor -> safety_supervisor -> /cmd_vel
    twist_mux = Node(
        package='twist_mux', executable='twist_mux',
        name='twist_mux', output='screen',
        parameters=[config('twist_mux.yaml'), {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel_out', 'cmd_vel_mux')],
    )
    smoother = ComposableNode(
        package='nav2_velocity_smoother', plugin='nav2_velocity_smoother::VelocitySmoother',
        name='velocity_smoother',
        parameters=[smoother_params],
        remappings=[('cmd_vel', 'cmd_vel_mux'), ('cmd_vel_smoothed', 'cmd_vel_smoothed')],
    )
    collision_monitor = ComposableNode(
        package='nav2_collision_monitor', plugin='nav2_collision_monitor::CollisionMonitor',
        name='collision_monitor',
        parameters=[collision_params],
    )
    # Share the ROS context so BT clients match their providers locally. Lifecycle
    # activation alone did not make remote action endpoints visible to the BT process.
    # Isolated executors let each server process its own lifecycle callbacks.
    navigation_container = ComposableNodeContainer(
        package='rclcpp_components', executable='component_container',
        name='navigation_container', namespace='', output='screen',
        # Costmaps are child nodes; their namespaced parameters must also be
        # available in the container's global arguments, as in Nav2 bringup.
        parameters=[nav2_params],
        arguments=['--isolated', '--executor-type', 'single-threaded'],
        composable_node_descriptions=[
            controller, planner, behaviors, smoother, collision_monitor,
            bt_navigator, docking_server,
        ],
    )
    safety_supervisor = Node(
        package='robot_navigation', executable='safety_supervisor.py',
        name='safety_supervisor', output='screen',
        # Wall time on purpose: independent of /clock, and sim-time timers busy-spin in rclpy.
        parameters=[{'use_sim_time': False}],
    )
    lifecycle_navigation_core = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation_core',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': False,
            'node_names': [
                'controller_server', 'planner_server', 'behavior_server',
                'velocity_smoother', 'collision_monitor',
            ],
        }],
    )
    lifecycle_navigation_frontend = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation_frontend',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': False,
            'node_names': ['bt_navigator', 'docking_server'],
        }],
    )
    navigation_runtime_coordinator = Node(
        package='robot_navigation', executable='navigation_runtime_coordinator.py',
        name='navigation_runtime_coordinator', output='screen',
        parameters=[{'use_sim_time': False}],
    )
    fastdds_profile = SetEnvironmentVariable(
        name='FASTDDS_DEFAULT_PROFILES_FILE', value=config('fastdds_no_shm.xml'))
    sim = GroupAction(
        scoped=True,
        condition=IfCondition(PythonExpression([
            "'", backend, "' == 'sim' and '",
            LaunchConfiguration('start_simulator'), "' == 'true'",
        ])),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('robot_bringup'), 'launch', 'sim.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time, 'rviz': 'false',
                'headless': LaunchConfiguration('headless'),
            }.items(),
        )],
    )

    def wall_node(script, name, extra=None, condition=None):
        # Wall time on purpose: sim-time timers busy-spin rclpy nodes.
        return Node(
            package='robot_navigation', executable=script, name=name, output='screen',
            parameters=[{'use_sim_time': False}] + (extra or []),
            condition=condition,
        )

    nav_monitor = wall_node('nav_monitor.py', 'nav_monitor')
    virtual_walls = wall_node('virtual_wall_manager.py', 'virtual_wall_manager',
                              [{'maps_dir': maps_dir}])

    # Simulation backend for /robot/battery, /robot/dock/contact, /robot/dock/detected_pose.
    # On hardware (backend:=hardware) the BMS / contact / dock-detector drivers publish them.
    sim_backend_params = config('sim_backend.yaml')
    use_sim_backend = IfCondition(PythonExpression(["'", backend, "' == 'sim'"]))
    sim_backend = []
    for script, name in (('sim_battery.py', 'sim_battery'),
                         ('sim_dock_contact.py', 'sim_dock_contact'),
                         ('sim_dock_detector.py', 'sim_dock_detector')):
        sim_backend.append(wall_node(script, name, [sim_backend_params],
                                     condition=use_sim_backend))
    docking_manager = wall_node('docking_manager.py', 'docking_manager')
    battery_monitor = wall_node('battery_monitor.py', 'battery_monitor')
    state_manager = wall_node('robot_state_manager.py', 'robot_state_manager')
    health_monitor = wall_node('robot_health_monitor.py', 'robot_health_monitor')

    def rviz(config_name, condition):
        return Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(nav_share, 'rviz', config_name)],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=condition,
            output='screen',
        )

    rviz_navigation = rviz('navigation.rviz', IfCondition(PythonExpression(
        ["'", use_rviz, "' == 'true' and '", mode, "' == 'navigation'"])))
    rviz_mapping = rviz('slam.rviz', IfCondition(PythonExpression(
        ["'", use_rviz, "' == 'true' and '", mode, "' == 'mapping'"])))

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the simulation (Gazebo) clock for ROS-time nodes',
        ),
        DeclareLaunchArgument(
            'backend', default_value='sim',
            description="'sim' launches the simulated battery/dock-contact/dock-detector; "
                        "'hardware' expects drivers to publish them",
        ),
        DeclareLaunchArgument(
            'start_simulator', default_value='true', choices=['true', 'false'],
            description='Start Gazebo with the sim backend; false when bringup supplies it',
        ),
        DeclareLaunchArgument(
            'headless', default_value='',
            description="Pass '-s' to run the Gazebo server without its GUI",
        ),
        DeclareLaunchArgument(
            'mode', default_value='navigation',
            description="UI/RViz startup view; runtime mode changes use /robot/maps services",
        ),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Launch RViz with the view matching the mode',
        ),
        DeclareLaunchArgument(
            'maps_dir', default_value=os.path.expanduser('~/.robot/maps'),
            description='Absolute writable runtime map root',
        ),
        DeclareLaunchArgument(
            'amcl_params_file',
            default_value=config('amcl.yaml'),
            description='map_server + AMCL parameters file',
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=config('nav2_params.yaml'),
            description='Nav2 navigation parameters file',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=config('slam_toolbox.yaml'),
            description='slam_toolbox parameters file',
        ),
        fastdds_profile,
        sim,
        map_server,
        amcl,
        lifecycle_localization,
        slam,
        map_manager,
        navigation_container,
        twist_mux,
        safety_supervisor,
        lifecycle_navigation_core,
        lifecycle_navigation_frontend,
        navigation_runtime_coordinator,
        nav_monitor,
        virtual_walls,
        *sim_backend,
        docking_manager,
        battery_monitor,
        state_manager,
        health_monitor,
        rviz_navigation,
        rviz_mapping,
    ])
