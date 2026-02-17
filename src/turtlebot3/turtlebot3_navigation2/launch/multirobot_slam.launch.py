# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Multi-robot SLAM for Blinky + Pinky (domain bridge aggregation)
# Requires: domain bridges running (ROS_DOMAIN_ID=50), robots on domains 30 and 31
#
# Usage: ROS_DOMAIN_ID=50 ros2 launch turtlebot3_navigation2 multirobot_slam.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    workspace_dir = os.path.expanduser('~/turtlebot3_ws')
    param_dir = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'param', 'humble')
    map_merge_params = os.path.join(workspace_dir, 'config', 'map_merge', 'multirobot_params.yaml')
    tf_relay_script = os.path.join(workspace_dir, 'scripts', 'tf_relay_multirobot.py')
    normalizer_script = os.path.join(
        workspace_dir, 'src', 'turtlebot3', 'turtlebot3_navigation2',
        'scripts', 'normalize_laser_scan.py')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Blinky normalizer: /blinky/scan -> /blinky/scan_normalized
    blinky_normalizer = ExecuteProcess(
        cmd=['python3', normalizer_script,
             '--ros-args', '-p', 'input_topic:=/blinky/scan',
             '-p', 'output_topic:=/blinky/scan_normalized'],
        output='screen',
    )

    # Pinky normalizer: /pinky/scan -> /pinky/scan_normalized
    pinky_normalizer = ExecuteProcess(
        cmd=['python3', normalizer_script,
             '--ros-args', '-p', 'input_topic:=/pinky/scan',
             '-p', 'output_topic:=/pinky/scan_normalized'],
        output='screen',
    )

    # TF relay: blinky/tf, pinky/tf -> /tf with frame prefixes
    tf_relay = ExecuteProcess(
        cmd=['python3', tf_relay_script],
        output='screen',
    )

    # Blinky SLAM
    blinky_slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox_blinky',
        output='screen',
        parameters=[
            os.path.join(param_dir, 'mapper_params_blinky.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/map', '/blinky/map'),
            ('/map_metadata', '/blinky/map_metadata'),
        ],
    )

    # Pinky SLAM
    pinky_slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox_pinky',
        output='screen',
        parameters=[
            os.path.join(param_dir, 'mapper_params_pinky.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/map', '/pinky/map'),
            ('/map_metadata', '/pinky/map_metadata'),
        ],
    )

    # Map merge
    map_merge = Node(
        package='multirobot_map_merge',
        executable='map_merge',
        name='map_merge',
        output='screen',
        parameters=[
            map_merge_params,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        blinky_normalizer,
        pinky_normalizer,
        tf_relay,
        blinky_slam,
        pinky_slam,
        map_merge,
    ])
