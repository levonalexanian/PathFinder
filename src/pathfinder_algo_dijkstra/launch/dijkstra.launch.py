from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_radius = LaunchConfiguration("robot_radius")
    max_plan_time_sec = LaunchConfiguration("max_plan_time_sec")
    feedback_node_stride = LaunchConfiguration("feedback_node_stride")
    feedback_time_stride_sec = LaunchConfiguration("feedback_time_stride_sec")
    publish_current_best = LaunchConfiguration("publish_current_best")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_robot_radius = DeclareLaunchArgument(
        "robot_radius", default_value="0.25",
        description="Robot radius used to inflate occupied voxels",
    )
    declare_max_plan_time_sec = DeclareLaunchArgument(
        "max_plan_time_sec", default_value="5.0",
        description="Time budget for a single Dijkstra plan",
    )
    declare_feedback_node_stride = DeclareLaunchArgument(
        "feedback_node_stride", default_value="200",
        description="Publish feedback every N expanded nodes",
    )
    declare_feedback_time_stride_sec = DeclareLaunchArgument(
        "feedback_time_stride_sec", default_value="0.2",
        description="Publish feedback at least this often (seconds)",
    )
    declare_publish_current_best = DeclareLaunchArgument(
        "publish_current_best", default_value="true",
        description="Include CURRENT_BEST_PATH approximation in feedback markers",
    )

    dijkstra = Node(
        package="pathfinder_algo_dijkstra",
        executable="dijkstra_node",
        name="dijkstra_planner",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_radius": robot_radius,
            "max_plan_time_sec": max_plan_time_sec,
            "feedback_node_stride": feedback_node_stride,
            "feedback_time_stride_sec": feedback_time_stride_sec,
            "publish_current_best": publish_current_best,
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_robot_radius,
        declare_max_plan_time_sec,
        declare_feedback_node_stride,
        declare_feedback_time_stride_sec,
        declare_publish_current_best,
        dijkstra,
    ])
