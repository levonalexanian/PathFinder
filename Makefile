COMPOSE ?= docker compose
DEV := $(COMPOSE) run --rm -T dev
DEV_TTY := $(COMPOSE) run --rm --service-ports dev

TOOLCHAIN := build/conan_toolchain.cmake

.PHONY: help image-build install build test launch sh down clean

help:
	@echo "ros2_pathfinder make targets:"
	@echo "  image-build  Build the dev container image"
	@echo "  install      Run conan install inside the container"
	@echo "  build        colcon build inside the container"
	@echo "  test         colcon test inside the container"
	@echo "  launch       Launch the simulation"
	@echo "  sh           Open an interactive shell in the dev container"
	@echo "  down         Stop and remove containers"
	@echo "  clean        Stop containers and remove the local image"

image-build:
	$(COMPOSE) build dev

install:
	$(DEV) conan install . --output-folder=build --build=missing -s build_type=Release

build:
	$(DEV) bash -c 'source /opt/ros/jazzy/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=$(TOOLCHAIN)'

test:
	$(DEV) bash -c 'source /opt/ros/jazzy/setup.bash && colcon test && colcon test-result --verbose'

launch:
	$(DEV_TTY) bash -c 'source /opt/ros/jazzy/setup.bash && ros2 launch pathfinder_bringup simulation.launch.py'

sh:
	$(DEV_TTY) bash

down:
	$(COMPOSE) down --remove-orphans

clean:
	$(COMPOSE) down --rmi local --remove-orphans
