"""
Start the fleet side: one task executor per robot, and the fleet manager.

Run it once the robots' Nav2 is up (amr_bringup sim.launch.py, map shown in RViz),
with the same robots:= (default 3): the fleet manager gives a robot its first task as
soon as that robot reports a pose.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start_fleet(context):
    """Start the executors of amr_1 ... amr_N and the fleet manager for all of them."""
    count = int(LaunchConfiguration('robots').perform(context))
    robot_ids = [f'amr_{number}' for number in range(1, count + 1)]
    config_dir = os.path.join(get_package_share_directory('amr_fleet_manager'), 'config')

    # 1. One task executor per robot, in the robot's namespace (/amr_1/...), next to
    #    its Nav2; it reads where its robot is from the robot's own TF tree
    executors = [
        Node(
            package='amr_task_executor',
            executable='executor',
            namespace=robot_id,
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        )
        for robot_id in robot_ids
    ]

    # 2. The fleet manager, with the warehouse layout of site.yaml; it needs a free
    #    queue slot for each robot
    fleet_manager = Node(
        package='amr_fleet_manager',
        executable='fleet_manager',
        namespace='fleet',
        name='fleet_manager',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'fleet_manager.yaml'),
            {   # later entries override the YAML file
                'site_config_file': os.path.join(config_dir, 'site.yaml'),
                'robot_ids': robot_ids,
                'use_sim_time': True,
            },
        ],
    )

    return [*executors, fleet_manager]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robots', default_value='3', description='Number of AMRs'),
        OpaqueFunction(function=start_fleet),
    ])
