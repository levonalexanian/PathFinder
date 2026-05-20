# ros2_pathfinder

ROS2 Jazzy C++ showcase of pathfinding algorithms (A\*, Dijkstra, RRT\*, D\* Lite) navigating a 3D voxel map in Gazebo Harmonic, visualized via Foxglove.

## Make targets

| Target | Description |
|--------|-------------|
| `make help` | Print usage |
| `make image-build` | Build the dev container image |
| `make install` | Run `conan install` inside the container |
| `make build` | `colcon build` inside the container |
| `make test` | `colcon test` inside the container |
| `make launch` | Launch the simulation |
| `make sh` | Open an interactive shell in the dev container |
| `make down` | Stop and remove containers |
| `make clean` | Stop containers and remove the local image |
