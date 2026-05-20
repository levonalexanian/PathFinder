COMPOSE ?= docker compose
DEV := $(COMPOSE) run --rm -T dev
DEV_TTY := $(COMPOSE) run --rm --service-ports dev

TOOLCHAIN := /workspace/build/conan_toolchain.cmake

.PHONY: help image-build install build test launch launch-drone launch-drone-headless launch-headless launch-car launch-car-headless launch-base launch-astar launch-dijkstra launch-rrt launch-algos launch-demo-drone launch-demo-car generate-map regenerate-drone-description regenerate-car-description sh down clean

help:
	@echo "ros2_pathfinder make targets:"
	@echo "  image-build                  Build the dev container image"
	@echo "  install                      Run conan install inside the container"
	@echo "  build                        colcon build inside the container"
	@echo "  test                         colcon test inside the container"
	@echo "  generate-map                 Generate maps/demo_world.bt via pathfinder_core"
	@echo "  regenerate-drone-description Regenerate drone.urdf from drone.urdf.xacro"
	@echo "  regenerate-car-description   Regenerate car.urdf from car.urdf.xacro"
	@echo "  launch                       Alias for launch-drone (default robot)"
	@echo "  launch-drone                 Launch the full drone bringup (Gazebo + planner)"
	@echo "  launch-drone-headless        Same as launch-drone but server-only with headless rendering"
	@echo "  launch-headless              Alias for launch-drone-headless"
	@echo "  launch-car                   Launch the full car bringup (Gazebo + planner)"
	@echo "  launch-car-headless          Same as launch-car but server-only with headless rendering"
	@echo "  launch-base                  Launch just the base bringup (no Gazebo) for diagnostics"
	@echo "  launch-astar                 Launch the A* action server only"
	@echo "  launch-dijkstra              Launch the Dijkstra action server only"
	@echo "  launch-rrt                   Launch the RRT* action server only"
	@echo "  launch-algos                 Launch all three planner action servers together"
	@echo "  launch-demo-drone            Drone bringup + all three planner servers"
	@echo "  launch-demo-car              Car bringup + all three planner servers"
	@echo "  sh                           Open an interactive shell in the dev container"
	@echo "  down                         Stop and remove containers"
	@echo "  clean                        Stop containers and remove the local image"

image-build:
	$(COMPOSE) build dev

install:
	$(DEV) conan install . --output-folder=build --build=missing -s build_type=Release

build:
	$(DEV) bash -c 'source /opt/ros/jazzy/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=$(TOOLCHAIN)'

test:
	$(DEV) bash -c 'source /opt/ros/jazzy/setup.bash && colcon test && colcon test-result --verbose'

launch: launch-drone

launch-drone:
	$(DEV_TTY) bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch pathfinder_drone drone.launch.py'

launch-drone-headless:
	$(DEV_TTY) bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch pathfinder_drone drone.launch.py headless:=true'

launch-headless: launch-drone-headless

launch-car:
	$(DEV_TTY) bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch pathfinder_car car.launch.py'

launch-car-headless:
	$(DEV_TTY) bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch pathfinder_car car.launch.py headless:=true'

launch-base:
	$(DEV_TTY) bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch pathfinder_bringup base.launch.py'

launch-astar:
	$(DEV_TTY) bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch pathfinder_astar astar.launch.py'

launch-dijkstra:
	$(DEV_TTY) bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch pathfinder_dijkstra dijkstra.launch.py'

launch-rrt:
	$(DEV_TTY) bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch pathfinder_rrt rrt.launch.py'

launch-algos:
	$(DEV_TTY) bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch pathfinder_bringup algos.launch.py'

launch-demo-drone:
	$(DEV_TTY) bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch pathfinder_bringup demo_drone.launch.py'

launch-demo-car:
	$(DEV_TTY) bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch pathfinder_bringup demo_car.launch.py'

generate-map:
	$(DEV) bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && mkdir -p maps && ros2 run pathfinder_core generate_demo_map maps/demo_world.bt'

regenerate-drone-description:
	$(DEV) bash -c 'source /opt/ros/jazzy/setup.bash && xacro src/pathfinder_drone/urdf/drone.urdf.xacro -o src/pathfinder_drone/urdf/drone.urdf && chown -R $$(stat -c %u:%g /workspace) src/pathfinder_drone/urdf'

regenerate-car-description:
	$(DEV) bash -c 'source /opt/ros/jazzy/setup.bash && xacro src/pathfinder_car/urdf/car.urdf.xacro -o src/pathfinder_car/urdf/car.urdf && chown -R $$(stat -c %u:%g /workspace) src/pathfinder_car/urdf'

sh:
	$(DEV_TTY) bash

down:
	$(COMPOSE) down --remove-orphans

clean:
	$(COMPOSE) down --rmi local --remove-orphans
