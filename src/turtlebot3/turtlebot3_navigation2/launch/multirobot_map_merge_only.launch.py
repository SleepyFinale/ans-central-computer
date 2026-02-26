# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Multi-robot map merge + TF relay only (no SLAM, no normalizers on central PC).
# Use when each robot runs its own SLAM and Nav2; domain bridges forward
# /blinky/map, /pinky/map, /blinky/tf, /pinky/tf into domain 50.
#
# Requires: domain bridges running (ROS_DOMAIN_ID=50), robots on 30/31 with local SLAM.
# Usage: ROS_DOMAIN_ID=50 ros2 launch turtlebot3_navigation2 multirobot_map_merge_only.launch.py

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    workspace_dir = os.path.expanduser('~/turtlebot3_ws')
    default_map_merge_params = os.path.join(
        workspace_dir, 'config', 'map_merge', 'multirobot_params.yaml')
    map_merge_params = LaunchConfiguration(
        'map_merge_params_file',
        default=default_map_merge_params)
    tf_relay_script = os.path.join(workspace_dir, 'scripts', 'tf_relay_multirobot.py')
    tf_fallback_script = os.path.join(workspace_dir, 'scripts', 'tf_map_odom_fallback.py')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    tf_relay = ExecuteProcess(
        cmd=['python3', tf_relay_script],
        output='screen',
    )
    tf_map_odom_fallback = ExecuteProcess(
        cmd=['python3', tf_fallback_script],
        output='screen',
    )

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
        DeclareLaunchArgument(
            'map_merge_params_file',
            default_value=default_map_merge_params,
            description='Path to map_merge params YAML'),
        tf_relay,
        tf_map_odom_fallback,
        map_merge,
    ])
