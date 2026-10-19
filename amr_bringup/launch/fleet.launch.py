"""
Start the fleet side: one task executor per robot, and the fleet manager.

Run it once Nav2 is up (amr_navigation bringup.launch.py, map shown in RViz): the
fleet manager gives a robot its first task as soon as that robot reports a pose.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# The robots of the fleet; the fleet manager needs a free queue slot for each one.
ROBOTS = ['amr_1']


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('amr_fleet_manager'), 'config')

    # 1. One task executor per robot, in the robot's namespace (/amr_1/...)
    executors = [
        Node(
            package='amr_task_executor',
            executable='executor',
            namespace=robot_id,
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
        for robot_id in ROBOTS
    ]

    # 2. The fleet manager, with the warehouse layout of site.yaml
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
                'robot_ids': ROBOTS,
                'use_sim_time': True,
            },
        ],
    )

    return LaunchDescription([*executors, fleet_manager])
