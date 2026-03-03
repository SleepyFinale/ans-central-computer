#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    ThisLaunchFileDir,
)


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration("params_file")

    default_params = PathJoinSubstitution(
        [ThisLaunchFileDir(), "..", "config", "multi_robot_explorer.yaml"]
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description=(
            "YAML file with parameters for the multi_robot_explorer node "
            "(robot_names, map_topic, world_frame, cost weights, etc.)"
        ),
    )

    start_explorer = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution([ThisLaunchFileDir(), "multi_robot_explorer.py"]),
            "--ros-args",
            "--params-file",
            params_file,
        ],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(declare_params_file)
    ld.add_action(start_explorer)
    return ld

