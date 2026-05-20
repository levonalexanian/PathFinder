import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("pathfinder_drone")
    bringup_share = get_package_share_directory("pathfinder_bringup")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    world_path = os.path.join(pkg_share, "gazebo", "worlds", "drone_world.sdf")
    drone_sdf_path = os.path.join(pkg_share, "gazebo", "drone.sdf")
    xacro_path = os.path.join(pkg_share, "urdf", "drone.urdf.xacro")

    headless = LaunchConfiguration("headless")
    default_algorithm = LaunchConfiguration("default_algorithm")
    launch_foxglove = LaunchConfiguration("launch_foxglove")

    declare_headless = DeclareLaunchArgument(
        "headless", default_value="false",
        description="If true, run gz sim server-only with headless rendering",
    )
    declare_default_algorithm = DeclareLaunchArgument(
        "default_algorithm", default_value="astar",
        description="Default planning algorithm name",
    )
    declare_launch_foxglove = DeclareLaunchArgument(
        "launch_foxglove", default_value="true",
        description="Launch the foxglove_bridge node",
    )

    gz_args = PythonExpression([
        "'", world_path, " -r'",
        " + (' -s --headless-rendering' if '", headless, "' == 'true' else '')",
    ])

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "base.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "default_algorithm": default_algorithm,
            "launch_foxglove": launch_foxglove,
        }.items(),
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    robot_description = Command(["xacro ", xacro_path])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }],
    )

    spawn_drone = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_pathfinder_drone",
        output="screen",
        arguments=[
            "-file", drone_sdf_path,
            "-name", "pathfinder_drone",
            "-x", "0", "-y", "0", "-z", "0.5",
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/model/pathfinder_drone/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
        ],
        parameters=[{"use_sim_time": True}],
    )

    pose_relay = Node(
        package="pathfinder_core",
        executable="gz_pose_relay_node",
        name="gz_pose_relay",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "source_topic": "/model/pathfinder_drone/pose",
            "parent_frame": "map",
            "child_frame": "base_link",
        }],
    )

    path_follower = Node(
        package="pathfinder_drone",
        executable="path_follower_node",
        name="path_follower",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    set_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.join(pkg_share, "gazebo"),
    )

    return LaunchDescription([
        declare_headless,
        declare_default_algorithm,
        declare_launch_foxglove,
        set_resource_path,
        base_launch,
        gz_sim,
        robot_state_publisher,
        spawn_drone,
        bridge,
        pose_relay,
        path_follower,
    ])
