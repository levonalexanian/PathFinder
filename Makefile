COMPOSE ?= docker compose
DEV := $(COMPOSE) run --rm -T dev
DEV_TTY := $(COMPOSE) run --rm --service-ports dev

TOOLCHAIN := /workspace/build/conan_toolchain.cmake

.PHONY: help image-build install build test launch generate-map sh down clean

help:
	@echo "ros2_pathfinder make targets:"
	@echo "  image-build   Build the dev container image"
	@echo "  install       Run conan install inside the container"
	@echo "  build         colcon build inside the container"
	@echo "  test          colcon test inside the container"
	@echo "  generate-map  Generate maps/demo_world.bt via pathfinder_core"
	@echo "  launch        Launch the base bringup"
	@echo "  sh            Open an interactive shell in the dev container"
	@echo "  down          Stop and remove containers"
	@echo "  clean         Stop containers and remove the local image"

image-build:
	$(COMPOSE) build dev

install:
	$(DEV) conan install . --output-folder=build --build=missing -s build_type=Release

build:
	$(DEV) bash -c 'source /opt/ros/jazzy/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=$(TOOLCHAIN)'

test:
	$(DEV) bash -c 'source /opt/ros/jazzy/setup.bash && colcon test && colcon test-result --verbose'

launch:
	$(DEV_TTY) bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch pathfinder_bringup base.launch.py'

generate-map:
	$(DEV) bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && mkdir -p maps && ros2 run pathfinder_core generate_demo_map maps/demo_world.bt'

sh:
	$(DEV_TTY) bash

down:
	$(COMPOSE) down --remove-orphans

clean:
	$(COMPOSE) down --rmi local --remove-orphans
