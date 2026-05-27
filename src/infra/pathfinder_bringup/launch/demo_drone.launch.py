import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from demo_common import make_demo_launch  # noqa: E402

generate_launch_description = make_demo_launch(
    "pathfinder_robot_drone", "drone.launch.py"
)
