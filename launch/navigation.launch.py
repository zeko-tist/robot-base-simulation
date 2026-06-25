#!/usr/bin/env python3
"""
launch/zeko_navigation_full.launch.py — Complete navigation launch for ZEKO

Starts the full Nav2 + AMCL + robot_state_publisher stack needed for
autonomous navigation in Isaac Sim 6.0.

WHAT THIS LAUNCHES
------------------
  1. robot_state_publisher     — publishes URDF joints to /tf (base_link → lidar_link, etc.)
  2. map_server                — serves the pre-built occupancy map on /map
  3. amcl                      — particle-filter localisation (global, no 2D Pose Estimate)
  4. nav2_bringup (navigation) — full Nav2 stack: planner, controller, BT navigator, etc.
  5. global_localization_trigger — auto-calls reinitialize_global_localization on startup
  6. rviz2                     — visualiser (optional, disable with use_rviz:=false)

PREREQUISITES
-------------
  - Isaac Sim 6.0 running with ZEKO scene loaded
  - isaac_sim_ros2_bridge.py executed inside Isaac Sim (publishes /odom, /scan_base, /tf, /clock)
  - ROS2 Humble sourced
  - robot_base_simulation package built: colcon build --packages-select robot_base_simulation
  - Map file at config/maps/floor1_map.yaml (generated from Isaac Sim)

USAGE
-----
  # Default (with RViz):
  ros2 launch robot_base_simulation zeko_navigation_full.launch.py

  # With custom map:
  ros2 launch robot_base_simulation zeko_navigation_full.launch.py \
    map:=/absolute/path/to/floor1_map.yaml

  # Without RViz:
  ros2 launch robot_base_simulation zeko_navigation_full.launch.py use_rviz:=false

  # To send a navigation goal (once localised):
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "pose: {header: {frame_id: map}, pose: {position: {x: 10.0, y: 5.0, z: 0.0}, \
    orientation: {w: 1.0}}}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Package directories ───────────────────────────────────────────────
    pkg_dir      = get_package_share_directory("robot_base_simulation")
    nav2_bringup = get_package_share_directory("nav2_bringup")

    # ── File paths ────────────────────────────────────────────────────────
    default_map_path    = os.path.join(pkg_dir, "config", "maps", "floor1_map.yaml")
    default_params_path = os.path.join(pkg_dir, "config", "nav2_params_zeko.yaml")
    default_xacro_path  = os.path.join(pkg_dir, "urdf", "zeko_nav.xacro")  # includes lidar.xacro
    default_rviz_config = os.path.join(nav2_bringup, "rviz", "nav2_default_view.rviz")

    # ── Launch arguments ──────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock from Isaac Sim (/clock topic)",
    )
    map_arg = DeclareLaunchArgument(
        "map",
        default_value=default_map_path,
        description="Absolute path to the occupancy map YAML file",
    )
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_path,
        description="Path to the Nav2 + AMCL parameters file",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2 for visualisation",
    )
    use_global_loc_arg = DeclareLaunchArgument(
        "use_global_localization",
        default_value="true",
        description="Auto-trigger global localisation on startup (no 2D Pose Estimate needed)",
    )
    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Auto-start Nav2 lifecycle nodes",
    )

    # ── Resolved configurations ───────────────────────────────────────────
    use_sim_time       = LaunchConfiguration("use_sim_time")
    map_path           = LaunchConfiguration("map")
    params_file        = LaunchConfiguration("params_file")
    use_rviz           = LaunchConfiguration("use_rviz")
    use_global_loc     = LaunchConfiguration("use_global_localization")
    autostart          = LaunchConfiguration("autostart")

    # ── Robot description from xacro ─────────────────────────────────────
    robot_description = ParameterValue(
        Command(["xacro ", default_xacro_path]),
        value_type=str,
    )

    # ── 1. Robot state publisher ──────────────────────────────────────────
    # Reads /robot_description and /joint_states → publishes TF for all
    # fixed/continuous URDF joints (including base_link → lidar_link).
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
            {"publish_frequency": 50.0},    # Hz
            {"ignore_timestamp": False},
        ],
    )

    # ── 2. Map server (standalone, outside nav2_bringup) ─────────────────
    # We launch map_server independently so we can control its lifecycle
    # without relying on the bringup lifecycle manager for it.
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"yaml_filename": map_path},
            {"topic_name": "map"},
            {"frame_id": "map"},
        ],
    )

    # ── 3. Map server lifecycle manager ──────────────────────────────────
    map_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )

    # ── 4. Nav2 navigation stack (includes AMCL + all planners) ──────────
    # nav2_bringup's navigation_launch.py starts:
    #   amcl, controller_server, planner_server, bt_navigator,
    #   behavior_server, waypoint_follower, velocity_smoother,
    #   lifecycle_manager_navigation
    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time":  "true",
            "map":           map_path,
            "params_file":   params_file,
            "autostart":     autostart,
            # Do NOT use_composition here — simpler debugging without composing
        }.items(),
    )

    # ── 5. Global localisation trigger ────────────────────────────────────
    # This node calls /reinitialize_global_localization after AMCL starts,
    # then monitors convergence. Eliminates "2D Pose Estimate" button clicks.
    global_loc_node = Node(
        package="robot_base_simulation",
        executable="global_localization_trigger",
        name="global_localization_trigger",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"covariance_threshold":        0.5},
            {"max_wait_for_amcl":           90.0},   # s (AMCL takes time to become active)
            {"rotation_speed":              0.3},    # rad/s — slow spin for convergence
            {"rotation_duration":           12.0},   # s
            {"check_kidnap":                True},
            {"kidnap_covariance_threshold": 2.0},
            {"kidnap_recheck_interval":     5.0},
        ],
        condition=IfCondition(use_global_loc),
    )

    # ── 6. RViz2 ─────────────────────────────────────────────────────────
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", default_rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    # ── Startup log ───────────────────────────────────────────────────────
    startup_log = LogInfo(msg=(
        "\n"
        "══════════════════════════════════════════════════════════════\n"
        " ZEKO Navigation Launch\n"
        "══════════════════════════════════════════════════════════════\n"
        " BEFORE starting this launch:\n"
        "  1. Isaac Sim 6.0 must be running with ZEKO scene loaded\n"
        "  2. Run isaac_sim_ros2_bridge.py from Isaac Sim Script Editor\n"
        "  3. Confirm topics: ros2 topic list | grep -E 'scan|odom|clock'\n"
        "  4. Confirm TF: ros2 run tf2_tools view_frames\n"
        "\n"
        " Autonomy pipeline:\n"
        "  /scan_base + /odom → AMCL → map→odom TF → Nav2 → /cmd_vel\n"
        "  /cmd_vel → ZEKO extension → wheels → physics\n"
        "\n"
        " To send a goal via CLI:\n"
        "  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose\n"
        "  \"pose: {header: {frame_id: map}, pose: {position: {x: 10.0, y: 5.0}}}\"\n"
        "══════════════════════════════════════════════════════════════\n"
    ))

    # ── Launch description ────────────────────────────────────────────────
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        map_arg,
        params_arg,
        use_rviz_arg,
        use_global_loc_arg,
        autostart_arg,

        # Log
        startup_log,

        # Nodes (order matters: RSP first, then map, then Nav2)
        robot_state_publisher,
        map_server,
        map_lifecycle_manager,
        nav2_stack,
        global_loc_node,
        rviz,
    ])
