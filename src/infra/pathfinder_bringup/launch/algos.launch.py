import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    astar_share = get_package_share_directory("pathfinder_algo_astar")
    dijkstra_share = get_package_share_directory("pathfinder_algo_dijkstra")
    dstar_lite_share = get_package_share_directory("pathfinder_algo_dstar_lite")
    rrt_share = get_package_share_directory("pathfinder_algo_rrt")

    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_radius = LaunchConfiguration("robot_radius")
    max_plan_time_sec = LaunchConfiguration("max_plan_time_sec")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_robot_radius = DeclareLaunchArgument(
        "robot_radius", default_value="0.25",
        description="Robot radius (m) used to inflate occupied voxels",
    )
    declare_max_plan_time_sec = DeclareLaunchArgument(
        "max_plan_time_sec", default_value="5.0",
        description="Hard time budget for a single plan (seconds)",
    )

    shared_args = {
        "use_sim_time": use_sim_time,
        "robot_radius": robot_radius,
        "max_plan_time_sec": max_plan_time_sec,
    }.items()

    astar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(astar_share, "launch", "astar.launch.py")
        ),
        launch_arguments=shared_args,
    )

    dijkstra_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dijkstra_share, "launch", "dijkstra.launch.py")
        ),
        launch_arguments=shared_args,
    )

    dstar_lite_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dstar_lite_share, "launch", "dstar_lite.launch.py")
        ),
        launch_arguments=shared_args,
    )

    rrt_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rrt_share, "launch", "rrt.launch.py")
        ),
        launch_arguments=shared_args,
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_robot_radius,
        declare_max_plan_time_sec,
        astar_launch,
        dijkstra_launch,
        dstar_lite_launch,
        rrt_launch,
    ])
