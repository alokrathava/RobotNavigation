import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    nav_share = get_package_share_directory('robot_navigation')
    bringup_share = get_package_share_directory('robot_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')
    map_file = LaunchConfiguration('map')
    amcl_params = LaunchConfiguration('amcl_params_file')
    nav2_params = ParameterFile(
        LaunchConfiguration('nav2_params_file'), allow_substs=True
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time, 'rviz': 'false',
            'headless': LaunchConfiguration('headless'),
        }.items(),
    )

    # --- localization: map_server + amcl (map -> odom) ---
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[amcl_params, {'yaml_filename': map_file}],
    )
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params],
    )
    lifecycle_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    # --- navigation: controller/planner/behaviors/bt_navigator ---
    # Controller and behaviors publish /cmd_vel_nav only; the safety pipeline
    # below produces the final /cmd_vel consumed by the Gazebo bridge.
    to_nav = [('cmd_vel', 'cmd_vel_nav')]
    controller = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[nav2_params], remappings=to_nav,
    )
    planner = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen',
        parameters=[nav2_params],
    )
    behaviors = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen',
        parameters=[nav2_params], remappings=to_nav,
    )
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        parameters=[nav2_params],
    )
    docking_server = Node(
        package='opennav_docking', executable='opennav_docking',
        name='docking_server', output='screen',
        parameters=[nav2_params], remappings=to_nav + [('battery_state', '/robot/battery')],
    )
    # Pipeline: twist_mux -> velocity_smoother -> collision_monitor -> safety_supervisor -> /cmd_vel
    twist_mux = Node(
        package='twist_mux', executable='twist_mux',
        name='twist_mux', output='screen',
        parameters=[
            os.path.join(nav_share, 'config', 'twist_mux.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('cmd_vel_out', 'cmd_vel_mux')],
    )
    smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother',
        name='velocity_smoother', output='screen',
        parameters=[os.path.join(nav_share, 'config', 'velocity_smoother.yaml')],
        remappings=[('cmd_vel', 'cmd_vel_mux'), ('cmd_vel_smoothed', 'cmd_vel_smoothed')],
    )
    collision_monitor = Node(
        package='nav2_collision_monitor', executable='collision_monitor',
        name='collision_monitor', output='screen',
        parameters=[os.path.join(nav_share, 'config', 'collision_monitor.yaml')],
    )
    safety_supervisor = Node(
        package='robot_navigation', executable='safety_supervisor.py',
        name='safety_supervisor', output='screen',
        # Wall time on purpose: independent of /clock, and sim-time timers busy-spin in rclpy.
        parameters=[{'use_sim_time': False}],
        respawn=True, respawn_delay=1.0,
    )
    lifecycle_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server', 'planner_server', 'behavior_server',
                'velocity_smoother', 'collision_monitor', 'bt_navigator', 'docking_server',
            ],
        }],
    )

    nav_monitor = Node(
        package='robot_navigation',
        executable='nav_monitor.py',
        name='nav_monitor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    virtual_walls = Node(
        package='robot_navigation',
        executable='virtual_wall_manager.py',
        name='virtual_wall_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_yaml': map_file,
        }],
    )

    def wall_node(script, name, extra=None):
        # Wall time on purpose: sim-time timers busy-spin rclpy nodes.
        return Node(
            package='robot_navigation', executable=script, name=name, output='screen',
            parameters=[{'use_sim_time': False}] + (extra or []),
            respawn=True, respawn_delay=1.0,
        )

    sim_power = wall_node('sim_power_system.py', 'sim_power_system')
    docking_manager = wall_node('docking_manager.py', 'docking_manager')
    battery_monitor = wall_node('battery_monitor.py', 'battery_monitor')
    state_manager = wall_node('robot_state_manager.py', 'robot_state_manager')
    health_monitor = wall_node('robot_health_monitor.py', 'robot_health_monitor')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(nav_share, 'rviz', 'navigation.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true',
        ),
        DeclareLaunchArgument(
            'headless', default_value='',
            description="Pass '-s' to run the Gazebo server without its GUI",
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz with the navigation view',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(nav_share, 'maps', 'main_floor.yaml'),
            description='Saved map yaml',
        ),
        DeclareLaunchArgument(
            'amcl_params_file',
            default_value=os.path.join(nav_share, 'config', 'amcl.yaml'),
            description='map_server + AMCL parameters file',
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(nav_share, 'config', 'nav2_params.yaml'),
            description='Nav2 navigation parameters file',
        ),
        sim,
        map_server,
        amcl,
        lifecycle_localization,
        controller,
        planner,
        behaviors,
        bt_navigator,
        twist_mux,
        smoother,
        collision_monitor,
        safety_supervisor,
        lifecycle_navigation,
        nav_monitor,
        virtual_walls,
        docking_server,
        sim_power,
        docking_manager,
        battery_monitor,
        state_manager,
        health_monitor,
        rviz,
    ])
