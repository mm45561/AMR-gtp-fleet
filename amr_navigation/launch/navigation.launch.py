"""
Nav2 and the floor-tag reader of one AMR, with everything under /<namespace>.

Every robot runs its own copy: AMCL, the planners and the docking server only read
that robot's scan, odom and TF tree (/<namespace>/tf). Put the robot into the world
first (amr_gazebo robot.launch.py) with the same namespace and start pose.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    gazebo_share = get_package_share_directory('amr_gazebo')
    nav_share = get_package_share_directory('amr_navigation')
    nav2_share = get_package_share_directory('nav2_bringup')
    docking_share = get_package_share_directory('amr_docking')
    namespace = LaunchConfiguration('namespace')

    args = [
        DeclareLaunchArgument('namespace', default_value='amr_1', description='Robot name'),
        # where the robot was put into the world; write floats (0.0, not 0): AMCL's
        # initial_pose parameters are doubles
        DeclareLaunchArgument('x', default_value='0.0', description='Start x [m]'),
        DeclareLaunchArgument('y', default_value='0.0', description='Start y [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Start heading [rad]'),
        # slam:=True draws a new map with this robot's LiDAR instead of loading one
        DeclareLaunchArgument('slam', default_value='False',
                              description='Build a new map (SLAM) instead of loading one'),
        # the default map is the TurtleBot 4 warehouse map
        DeclareLaunchArgument(
            'map', default_value=os.path.join(gazebo_share, 'maps', 'warehouse.yaml'),
            description='Map to load (not used when slam:=True)'),
    ]

    # 1. Our Nav2 parameters, with AMCL starting at this robot's start pose
    params = RewrittenYaml(
        source_file=os.path.join(nav_share, 'config', 'nav2_params.yaml'),
        param_rewrites={
            'amcl.ros__parameters.initial_pose.x': LaunchConfiguration('x'),
            'amcl.ros__parameters.initial_pose.y': LaunchConfiguration('y'),
            'amcl.ros__parameters.initial_pose.yaw': LaunchConfiguration('yaw'),
        },
        convert_types=True)

    # 2. Nav2 under /<namespace>; it remaps /tf to the robot's own tf by itself
    nav2 = IncludeLaunchDescription(
        os.path.join(nav2_share, 'launch', 'bringup_launch.py'),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': 'True',
            'slam': LaunchConfiguration('slam'),
            'map': LaunchConfiguration('map'),
            'params_file': params,
        }.items())

    # 3. Floor tags: apriltag_node finds them in this robot's camera image,
    #    tag_dock_pose hands their pose to this robot's docking server
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    tags = [
        Node(package='apriltag_ros', executable='apriltag_node', name='apriltag',
             namespace=namespace,
             parameters=[os.path.join(docking_share, 'config', 'apriltag.yaml'),
                         {'use_sim_time': True}],
             remappings=tf_remaps + [('image_rect', 'camera/image_raw'),
                                     ('camera_info', 'camera/camera_info')]),
        Node(package='amr_docking', executable='tag_dock_pose', namespace=namespace,
             parameters=[{'use_sim_time': True}], remappings=tf_remaps),
    ]

    return LaunchDescription([*args, nav2, *tags])
