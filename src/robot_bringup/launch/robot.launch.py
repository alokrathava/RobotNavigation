import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Single entry point for the whole robot.

    backend:=sim       Gazebo + sensor bridge + simulated battery/dock producers (sim clock)
    backend:=hardware  robot description + runtime only; real drivers publish the sensor,
                       odometry, battery, dock contact and dock detector interfaces (wall clock)
    mode:=navigation   map_server + AMCL (localization on a saved map)
    mode:=mapping      SLAM Toolbox builds a map
    """
    bringup_share = get_package_share_directory('robot_bringup')
    nav_share = get_package_share_directory('robot_navigation')
    desc_share = get_package_share_directory('robot_description')

    backend = LaunchConfiguration('backend')
    is_sim = IfCondition(PythonExpression(["'", backend, "' == 'sim'"]))
    is_hardware = IfCondition(PythonExpression(["'", backend, "' == 'hardware'"]))
    use_sim_time = PythonExpression(["'true' if '", backend, "' == 'sim' else 'false'"])

    # Included launch_arguments leak into the parent context unless the include is scoped;
    # without the GroupAction, sim.launch.py's rviz:=false would override this file's rviz.
    sim = GroupAction(scoped=True, condition=is_sim, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'sim.launch.py')),
        launch_arguments={
            'use_sim_time': 'true', 'rviz': 'false',
            'headless': LaunchConfiguration('headless'),
        }.items(),
    )])
    hardware_description = GroupAction(
        scoped=True, condition=is_hardware, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(desc_share, 'launch', 'rsp.launch.py')),
            launch_arguments={'use_sim_time': 'false'}.items(),
        )])
    runtime = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'backend': backend,
            'start_simulator': 'false',
            'mode': LaunchConfiguration('mode'),
            'maps_dir': LaunchConfiguration('maps_dir'),
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )

    gateway = Node(
        package='robot_gateway', executable='gateway_node', name='robot_gateway',
        output='screen',
        parameters=[{'maps_dir': LaunchConfiguration('maps_dir')}],
        arguments=['--port', LaunchConfiguration('gateway_port'),
                   '--auth-token', LaunchConfiguration('gateway_token')],
        condition=IfCondition(LaunchConfiguration('gateway')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gateway', default_value='true', choices=['true', 'false'],
            description='Start the WebSocket gateway for external clients',
        ),
        DeclareLaunchArgument(
            'gateway_port', default_value='8080',
            description='WebSocket gateway port',
        ),
        DeclareLaunchArgument(
            'gateway_token', default_value='',
            description='WebSocket gateway auth token (empty: none required)',
        ),
        DeclareLaunchArgument(
            'backend', default_value='sim', choices=['sim', 'hardware'],
            description='sim: Gazebo + simulated producers; hardware: real drivers',
        ),
        DeclareLaunchArgument(
            'mode', default_value='navigation', choices=['navigation', 'mapping'],
            description='navigation: localize on a saved map; mapping: SLAM',
        ),
        DeclareLaunchArgument(
            'maps_dir', default_value=os.path.expanduser('~/.robot/maps'),
            description='Absolute writable runtime map root',
        ),
        DeclareLaunchArgument(
            'headless', default_value='',
            description="Pass '-s' to run the Gazebo server without its GUI (sim only)",
        ),
        DeclareLaunchArgument(
            'rviz', default_value='false', choices=['true', 'false'],
            description='Launch RViz with the view matching the mode',
        ),
        sim,
        hardware_description,
        runtime,
        gateway,
    ])
