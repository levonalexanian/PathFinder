import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    default_map = os.path.join(os.getcwd(), "maps", "demo_world.bt")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map_file")
    default_algorithm = LaunchConfiguration("default_algorithm")
    launch_foxglove = LaunchConfiguration("launch_foxglove")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_map_file = DeclareLaunchArgument(
        "map_file", default_value=default_map,
        description="Path to the .bt octomap file to publish",
    )
    declare_default_algorithm = DeclareLaunchArgument(
        "default_algorithm", default_value="astar",
        description="Default planning algorithm name",
    )
    declare_launch_foxglove = DeclareLaunchArgument(
        "launch_foxglove", default_value="true",
        description="Launch the foxglove_bridge node",
    )

    map_publisher = Node(
        package="pathfinder_core",
        executable="map_publisher_node",
        name="map_publisher",
        output="screen",
        parameters=[{
            "map_file": map_file,
            "map_frame": "map",
            "use_sim_time": use_sim_time,
        }],
    )

    voxel_map_visualizer = Node(
        package="pathfinder_core",
        executable="voxel_map_visualizer_node",
        name="voxel_map_visualizer",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    scheduler = Node(
        package="pathfinder_core",
        executable="scheduler_node",
        name="scheduler",
        output="screen",
        parameters=[{
            "default_algorithm": default_algorithm,
            "use_sim_time": use_sim_time,
        }],
    )

    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[{
            "port": 8765,
            "use_sim_time": use_sim_time,
        }],
        condition=IfCondition(PythonExpression(["'", launch_foxglove, "' == 'true'"])),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map_file,
        declare_default_algorithm,
        declare_launch_foxglove,
        map_publisher,
        voxel_map_visualizer,
        scheduler,
        foxglove,
    ])
