import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('robot_navigation')
    bringup_share = get_package_share_directory('robot_bringup')
    slam_share = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')
    slam_params_file = LaunchConfiguration('slam_params_file')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'sim.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'rviz': 'false'}.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(nav_share, 'rviz', 'slam.rviz')],
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
            'rviz', default_value='true',
            description='Launch RViz with the SLAM view',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(nav_share, 'config', 'slam_toolbox.yaml'),
            description='slam_toolbox parameters file',
        ),
        sim,
        slam,
        rviz,
    ])
