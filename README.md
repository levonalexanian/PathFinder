# PathFinder

ROS2 Jazzy C++ showcase of four pathfinding algorithms — A\*, Dijkstra, RRT\*, D\* Lite — planning through a 3D voxel map in Gazebo Harmonic, visualized in Foxglove Studio. Two robots: a holonomic drone and a diff-drive car.

## Requirements

- **Docker** with Compose
- **make**
- A modern web browser + a free [Foxglove account](https://app.foxglove.dev/signup)

ROS2 Jazzy, Gazebo Harmonic, Conan 2, and the C++ toolchain all run in the dev container. Foxglove Studio runs in the browser via `app.foxglove.dev` — no host install needed.

## First-time setup

```bash
git clone https://github.com/levonalexanian/PathFinder.git
cd PathFinder
make image-build      # ~5-10 min, one-time
make install
make build
make generate-map
```

## Run the demo

```bash
make launch-car     # diff-drive car
make launch-drone   # holonomic drone
```

Then open **https://app.foxglove.dev**:

1. **Open connection** → **Foxglove WebSocket** → `ws://localhost:8765`
2. **File → Import Layout** → `foxglove/layouts/car.json` or `drone.json`

The URDF is fetched from GitHub raw, so the branch must be pushed for the robot mesh to render.

### Send a goal

In another terminal:

```bash
make sh
source install/setup.bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 9.0, y: 9.0, z: 1.5}}}'
```

### Switch algorithms

```bash
ros2 topic pub --once /algorithm_selection pathfinder_msgs/AlgorithmSelection \
  '{algorithm_name: dijkstra}'    # astar, dijkstra, rrt, or dstar_lite
```

Dijkstra explores ~35× more nodes than A\* on the same path; D\* Lite returns near-instantly on its second call (incremental replan).

## Make targets

| Target | What |
|---|---|
| `image-build`, `install`, `build`, `test` | Container, deps, colcon |
| `generate-map` | Regenerate `maps/demo_world.bt` |
| `launch-{car,drone}` | Full demo (robot + sim + all four planners) |
| `sh` | Shell in the dev container (for `ros2 launch …` of individual planners, etc.) |
| `down`, `clean` | Stop containers / remove image |

## Layout

```
src/
├── algorithms/   pathfinder_algo_{astar,dijkstra,dstar_lite,rrt}
├── robots/       pathfinder_robot_{car,drone}
└── infra/        pathfinder_{msgs,core,bringup}
```

Each algorithm follows a Node/Core split: a pure C++ library (`*_core`) holds the algorithm and is unit-testable without ROS; a thin ROS Node (`*_node`) wraps it. Parameters live in `config/params.yaml` per package.

CI builds and tests on push and PR.
