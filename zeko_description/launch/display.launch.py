#!/usr/bin/env python3
"""
launch/display.launch.py

Standalone visualization launch file for ZEKO.

Starts:
  - robot_state_publisher : publishes /tf and /robot_description, loaded
                             by running xacro on urdf/zeko.xacro
  - joint_state_publisher_gui : provides sliders for the two wheel joints
                             (left_wheel_joint / right_wheel_joint) so the
                             model can be inspected without a running
                             controller
  - rviz2                 : optional, launched with a basic config if
                             use_rviz is true (default)

Usage:
  ros2 launch zeko_description display.launch.py
  ros2 launch zeko_description display.launch.py use_rviz:=false
  ros2 launch zeko_description display.launch.py use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("zeko_description")
    default_xacro_path = os.path.join(pkg_share, "urdf", "zeko.xacro")
    default_rviz_config = os.path.join(pkg_share, "launch", "zeko.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    use_joint_state_gui = LaunchConfiguration("use_joint_state_gui")
    model = LaunchConfiguration("model")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Isaac Sim / Gazebo) clock if true",
    )

    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2 alongside robot_state_publisher",
    )

    declare_use_joint_state_gui = DeclareLaunchArgument(
        "use_joint_state_gui",
        default_value="true",
        description="Launch joint_state_publisher_gui for manual joint sliders",
    )

    declare_model = DeclareLaunchArgument(
        "model",
        default_value=default_xacro_path,
        description="Absolute path to the ZEKO Xacro file",
    )

    robot_description = ParameterValue(
        Command(["xacro ", model]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": use_sim_time},
        ],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(use_joint_state_gui),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", default_rviz_config] if os.path.exists(default_rviz_config) else [],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_use_rviz,
            declare_use_joint_state_gui,
            declare_model,
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
