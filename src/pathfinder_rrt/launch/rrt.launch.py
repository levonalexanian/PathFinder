from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulated /clock if true",
    )

    rrt_node = Node(
        package="pathfinder_rrt",
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
            "max_plan_time_sec": 5.0,
            "robot_radius": 0.25,
            "min_iterations_after_goal": 1000,
            "random_seed": 0,
            "output_frame": "map",
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        rrt_node,
    ])
