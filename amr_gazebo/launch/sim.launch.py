"""The warehouse world in Gazebo and its clock; robot.launch.py adds the robots."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_share = get_package_share_directory('amr_gazebo')
    world = os.path.join(gazebo_share, 'worlds', 'warehouse.sdf')

    # 0. Tell Gazebo where our own models are (model://apriltag_0 ... in the world)
    models_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(gazebo_share, 'models'))

    # 1. Gazebo with the warehouse; -r = start playing immediately
    gazebo = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'),
        launch_arguments={'gz_args': f'-r {world}'}.items())

    # 2. Simulation time for every node of every robot: /clock is bridged once, here
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])

    return LaunchDescription([models_path, gazebo, clock_bridge])
