from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_radius = LaunchConfiguration("robot_radius")
    max_plan_time_sec = LaunchConfiguration("max_plan_time_sec")
    feedback_every_nodes = LaunchConfiguration("feedback_every_nodes")
    feedback_every_seconds = LaunchConfiguration("feedback_every_seconds")

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
        description="Hard time budget for a single D* Lite plan (seconds)",
    )
    declare_feedback_every_nodes = DeclareLaunchArgument(
        "feedback_every_nodes", default_value="200",
        description="Publish feedback after this many vertex expansions",
    )
    declare_feedback_every_seconds = DeclareLaunchArgument(
        "feedback_every_seconds", default_value="0.2",
        description="Publish feedback at least this often (seconds)",
    )

    dstar_lite = Node(
        package="pathfinder_algo_dstar_lite",
        executable="dstar_lite_node",
        name="dstar_lite_planner",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_radius": robot_radius,
            "max_plan_time_sec": max_plan_time_sec,
            "feedback_every_nodes": feedback_every_nodes,
            "feedback_every_seconds": feedback_every_seconds,
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_robot_radius,
        declare_max_plan_time_sec,
        declare_feedback_every_nodes,
        declare_feedback_every_seconds,
        dstar_lite,
    ])
