"""
Start the simulation: Gazebo, every AMR with its Nav2 and tag reader, and RViz.

The warehouse world comes up first, then each robot; RViz shows one of them. Each robot
starts parked in its own queue slot and runs under /amr_N. robots:=1 starts only amr_1,
which is lighter on the PC. Start the fleet side afterwards with fleet.launch.py and the
same robots:=.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Start pose (x, y, yaw) of amr_1, amr_2, amr_3: the queue slots q1, q2, q3 of
# amr_fleet_manager/config/site.yaml, so the fleet manager finds them parked there.
START_POSES = [(-4.88, -0.03, 0.0), (-4.91, -2.05, 0.0), (-4.82, -4.13, 0.0)]


def start_robots(context):
    """Put each robot into the world, then start its Nav2 and tag reader."""
    gazebo_launch = os.path.join(get_package_share_directory('amr_gazebo'), 'launch')
    nav_launch = os.path.join(get_package_share_directory('amr_navigation'), 'launch')
    count = int(LaunchConfiguration('robots').perform(context))
    if not 1 <= count <= len(START_POSES):
        raise ValueError(f'robots:= must be 1 to {len(START_POSES)}, not {count}')

    actions = []
    for index, (x, y, yaw) in enumerate(START_POSES[:count]):
        # floats with a decimal point: AMCL's initial pose parameters are doubles
        robot = {'namespace': f'amr_{index + 1}',
                 'x': f'{x:.2f}', 'y': f'{y:.2f}', 'yaw': f'{yaw:.2f}'}
        actions.append(IncludeLaunchDescription(
            os.path.join(gazebo_launch, 'robot.launch.py'), launch_arguments=robot.items()))
        # Nav2 10 s later, once /clock, odom and scan exist; 5 s between the robots so
        # their Nav2 stacks do not all load at the same moment
        actions.append(TimerAction(period=10.0 + 5.0 * index, actions=[
            IncludeLaunchDescription(
                os.path.join(nav_launch, 'navigation.launch.py'),
                launch_arguments={**robot,
                                  'slam': LaunchConfiguration('slam'),
                                  'map': LaunchConfiguration('map')}.items()),
        ]))
    return actions


def generate_launch_description():
    gazebo_share = get_package_share_directory('amr_gazebo')
    nav_share = get_package_share_directory('amr_navigation')

    args = [
        DeclareLaunchArgument('robots', default_value='3', description='Number of AMRs (1-3)'),
        DeclareLaunchArgument('rviz_robot', default_value='amr_1',
                              description='The robot RViz shows'),
        # slam:=True draws a new map with the LiDAR instead of loading one (use robots:=1)
        DeclareLaunchArgument('slam', default_value='False',
                              description='Build a new map (SLAM) instead of loading one'),
        # the default map is the TurtleBot 4 warehouse map
        DeclareLaunchArgument(
            'map', default_value=os.path.join(gazebo_share, 'maps', 'warehouse.yaml'),
            description='Map to load (not used when slam:=True)'),
    ]

    # 1. Render Gazebo (and RViz) on the NVIDIA card, like the prefix we typed by hand
    nvidia = [
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
    ]

    # 2. The warehouse world and its clock
    world = IncludeLaunchDescription(os.path.join(gazebo_share, 'launch', 'sim.launch.py'))

    # 3. RViz for one robot: its map, scan, plan and front camera (rviz/amr.rviz uses
    #    relative topics, so the namespace picks the robot); 5 s later so it exists
    rviz = TimerAction(period=5.0, actions=[
        Node(package='rviz2', executable='rviz2', namespace=LaunchConfiguration('rviz_robot'),
             output='screen',
             arguments=['-d', os.path.join(nav_share, 'rviz', 'amr.rviz')],
             parameters=[{'use_sim_time': True}],
             remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]),
    ])

    # 4. The robots
    robots = OpaqueFunction(function=start_robots)

    return LaunchDescription([*args, *nvidia, world, rviz, robots])
