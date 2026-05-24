import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("pathfinder_algo_astar")
    params_file = os.path.join(pkg_share, "config", "params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_radius = LaunchConfiguration("robot_radius")
    max_plan_time_sec = LaunchConfiguration("max_plan_time_sec")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_robot_radius = DeclareLaunchArgument(
        "robot_radius", default_value="0.25",
        description="Inflation radius (m) applied to occupied voxels",
    )
    declare_max_plan_time_sec = DeclareLaunchArgument(
        "max_plan_time_sec", default_value="5.0",
        description="Hard time budget for a single plan (seconds)",
    )

    astar = Node(
        package="pathfinder_algo_astar",
        executable="astar_node",
        name="astar_planner",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "robot_radius": robot_radius,
                "max_plan_time_sec": max_plan_time_sec,
            },
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_robot_radius,
        declare_max_plan_time_sec,
        astar,
    ])
