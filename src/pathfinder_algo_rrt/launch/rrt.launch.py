from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_radius = LaunchConfiguration("robot_radius")
    max_plan_time_sec = LaunchConfiguration("max_plan_time_sec")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulated /clock if true",
    )
    declare_robot_radius = DeclareLaunchArgument(
        "robot_radius", default_value="0.25",
        description="Robot radius (m) used to inflate occupied voxels",
    )
    declare_max_plan_time_sec = DeclareLaunchArgument(
        "max_plan_time_sec", default_value="5.0",
        description="Hard time budget for a single plan (seconds)",
    )

    rrt_node = Node(
        package="pathfinder_algo_rrt",
        executable="rrt_node",
        name="rrt_planner",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "step_size": 0.3,
            "goal_bias": 0.05,
            "rewire_radius": 1.0,
            "goal_tolerance": 0.3,
            "max_iterations": 5000,
            "max_plan_time_sec": max_plan_time_sec,
            "robot_radius": robot_radius,
            "min_iterations_after_goal": 1000,
            "random_seed": 0,
            "output_frame": "map",
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_robot_radius,
        declare_max_plan_time_sec,
        rrt_node,
    ])
