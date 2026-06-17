import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir    = get_package_share_directory("zeko_description")
    nav2_bringup = get_package_share_directory("nav2_bringup")

    map_file    = os.path.join(pkg_dir, "config", "maps", "floor1_map.yaml")
    params_file = os.path.join(pkg_dir, "config", "nav2_params.yaml")

    return LaunchDescription([

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, "launch", "navigation_launch.py")
            ),
            launch_arguments={
                "use_sim_time":  "true",
                "map":           map_file,
                "params_file":   params_file,
            }.items(),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d",
                os.path.join(
                    nav2_bringup, "rviz", "nav2_default_view.rviz"
                ),
            ],
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
    ])
