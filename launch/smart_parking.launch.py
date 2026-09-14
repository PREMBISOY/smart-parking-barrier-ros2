"""One-command launch for the complete autonomous parking barrier demo."""

import os
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory('smart_parking_barrier')
    world = os.path.join(share, 'worlds', 'parking_lot.sdf')
    params = os.path.join(share, 'config', 'barrier_params.yaml')
    bridge = os.path.join(share, 'config', 'bridge.yaml')
    gz_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    gui = LaunchConfiguration('gui')

    gz_world_arg = shlex.quote(world)
    gui_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': f'-r {gz_world_arg}', 'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(gui),
    )
    headless_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': f'-r -s --headless-rendering {gz_world_arg}', 'on_exit_shutdown': 'true'}.items(),
        condition=UnlessCondition(gui),
    )
    common_parameters = [params, {'use_sim_time': True}]
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Launch Gazebo GUI.'),
        DeclareLaunchArgument('rviz', default_value='false', description='Reserved for an optional RViz view.'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gui_sim,
        headless_sim,
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='parking_bridge',
             output='screen', parameters=[{'config_file': bridge}]),
        Node(package='smart_parking_barrier', executable='barrier_controller', output='screen',
             parameters=common_parameters),
        Node(package='smart_parking_barrier', executable='vehicle_controller', output='screen',
             parameters=common_parameters),
        Node(package='smart_parking_barrier', executable='simulation_monitor', output='screen',
             parameters=[{'use_sim_time': True}]),
    ])
