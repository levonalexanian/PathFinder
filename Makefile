COMPOSE ?= docker compose
DEV := $(COMPOSE) run --rm -T dev
DEV_TTY := $(COMPOSE) run --rm --service-ports dev

TOOLCHAIN := /workspace/build/conan_toolchain.cmake

# Override with HEADLESS=true to run Gazebo server-only (no GUI). Useful when
# DISPLAY isn't set or the host doesn't have X11.
HEADLESS ?= false

.PHONY: help image-build install build test generate-map launch-car launch-drone sh down clean

help:
	@echo "PathFinder make targets:"
	@echo "  image-build    Build the dev container image"
	@echo "  install        Run conan install inside the container"
	@echo "  build          colcon build inside the container"
	@echo "  test           colcon test inside the container"
	@echo "  generate-map   Generate maps/demo_world.bt"
	@echo "  launch-car     Full demo: car + sim + all four planners"
	@echo "  launch-drone   Full demo: drone + sim + all four planners"
	@echo "  sh             Open an interactive shell in the dev container"
	@echo "  down           Stop and remove containers"
	@echo "  clean          Stop containers and remove the local image"

image-build:
	$(COMPOSE) build dev

install:
	$(DEV) conan install . --output-folder=build --build=missing -s build_type=Release

build:
	$(DEV) bash -c 'source /opt/ros/jazzy/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=$(TOOLCHAIN)'

test:
	$(DEV) bash -c 'source /opt/ros/jazzy/setup.bash && colcon test && colcon test-result --verbose'

generate-map:
	$(DEV) bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && mkdir -p maps && ros2 run pathfinder_core generate_demo_map maps/demo_world.bt'

launch-car:
	$(DEV_TTY) bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch pathfinder_bringup demo_car.launch.py headless:=$(HEADLESS)'

launch-drone:
	$(DEV_TTY) bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch pathfinder_bringup demo_drone.launch.py headless:=$(HEADLESS)'

sh:
	$(DEV_TTY) bash

down:
	$(COMPOSE) down --remove-orphans

clean:
	$(COMPOSE) down --rmi local --remove-orphans
