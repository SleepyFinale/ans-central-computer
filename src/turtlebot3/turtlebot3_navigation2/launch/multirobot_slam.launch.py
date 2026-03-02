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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    workspace_dir = os.path.expanduser('~/turtlebot3_ws')
    param_dir = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'param', 'humble')
    default_map_merge_params = os.path.join(
        workspace_dir, 'config', 'map_merge', 'multirobot_params.yaml')
    map_merge_params = LaunchConfiguration(
        'map_merge_params_file',
        default=default_map_merge_params)
    tf_relay_script = os.path.join(workspace_dir, 'scripts', 'tf_relay_multirobot.py')
    tf_fallback_script = os.path.join(workspace_dir, 'scripts', 'tf_map_odom_fallback.py')
    normalizer_script = os.path.join(
        workspace_dir, 'src', 'turtlebot3', 'turtlebot3_navigation2',
        'scripts', 'normalize_laser_scan.py')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_tf_fallback = LaunchConfiguration('use_tf_fallback', default='false')

    # Blinky normalizer: /blinky/scan -> /blinky/scan_normalized, frame_id -> blinky/base_scan
    blinky_normalizer = ExecuteProcess(
        cmd=['python3', normalizer_script,
             '--ros-args', '-p', 'input_topic:=/blinky/scan',
             '-p', 'output_topic:=/blinky/scan_normalized',
             '-p', 'frame_id_prefix:=blinky'],
        output='screen',
    )

    # Pinky normalizer: /pinky/scan -> /pinky/scan_normalized, frame_id -> pinky/base_scan
    pinky_normalizer = ExecuteProcess(
        cmd=['python3', normalizer_script,
             '--ros-args', '-p', 'input_topic:=/pinky/scan',
             '-p', 'output_topic:=/pinky/scan_normalized',
             '-p', 'frame_id_prefix:=pinky'],
        output='screen',
    )

    # TF relay: blinky/tf, pinky/tf -> /tf with frame prefixes
    tf_relay = ExecuteProcess(
        cmd=['python3', tf_relay_script],
        output='screen',
    )

    # Legacy TF fallback: map -> blinky/odom, map -> pinky/odom (identity).
    # Only launched when use_tf_fallback:=true. Normally map_merge publishes
    # the correct TF (map -> <robot>/map) via its publish_tf parameter, so
    # the fallback is no longer needed. Keep it as an escape hatch.
    tf_map_odom_fallback = ExecuteProcess(
        cmd=['python3', tf_fallback_script],
        output='screen',
        condition=IfCondition(use_tf_fallback),
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

    # Map merge (publishes TF by default via publish_tf:=true)
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
            description='Path to map_merge params YAML (e.g. multirobot_params_unknown_poses.yaml for unknown poses)'),
        DeclareLaunchArgument(
            'use_tf_fallback',
            default_value='false',
            description='Launch legacy tf_map_odom_fallback (identity map->odom). '
                        'Not needed when map_merge publish_tf is enabled (default).'),
        blinky_normalizer,
        pinky_normalizer,
        tf_relay,
        tf_map_odom_fallback,
        blinky_slam,
        pinky_slam,
        map_merge,
    ])
