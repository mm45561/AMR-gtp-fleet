"""
Put one AMR into the running warehouse world, with all its topics under /<namespace>.

Each robot gets its own robot_state_publisher, Gazebo model and bridge. Its TF tree
goes on /<namespace>/tf instead of /tf, so robots never see each other's frames.
Example: ros2 launch amr_gazebo robot.launch.py namespace:=amr_2 x:=-4.91 y:=-2.05
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory('amr_description'), 'urdf', 'amr.urdf.xacro')
    bridge_config = os.path.join(
        get_package_share_directory('amr_gazebo'), 'config', 'bridge.yaml')
    namespace = LaunchConfiguration('namespace')

    args = [
        DeclareLaunchArgument('namespace', default_value='amr_1', description='Robot name'),
        DeclareLaunchArgument('x', default_value='0.0', description='Start x [m]'),
        DeclareLaunchArgument('y', default_value='0.0', description='Start y [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Start heading [rad]'),
    ]

    # The robot's own TF tree: /tf -> /<namespace>/tf
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # 1. The robot model, with its Gazebo topics under /<namespace>/
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' namespace:=', namespace]), value_type=str)
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace=namespace, remappings=tf_remaps,
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    # 2. Put it into the world, slightly above the floor
    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', ['/', namespace, '/robot_description'], '-name', namespace,
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', '0.05', '-Y', LaunchConfiguration('yaw')])

    # 3. Its topics: the names in bridge.yaml are relative, so expand_gz_topic_names
    #    puts both the ROS and the Gazebo side under /<namespace>/
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', namespace=namespace,
        parameters=[{'config_file': bridge_config, 'expand_gz_topic_names': True,
                     'use_sim_time': True}])

    return LaunchDescription([*args, robot_state_publisher, spawn, bridge])
