# PathFinder

ROS2 Jazzy C++ showcase of four pathfinding algorithms (A\*, Dijkstra, RRT\*, D\* Lite) planning through a 3D voxel map in Gazebo Harmonic, visualized in Foxglove Studio. Two robots: a holonomic drone and a diff-drive car.

## Requirements

Everything runs inside Docker — you only need these on the host:

- **Docker** with Compose (tested on `docker 29.x`, `compose v5.x`)
- **make**
- A modern web browser to open Foxglove Studio at `http://localhost:8080`

That's it. ROS2 Jazzy, Gazebo Harmonic, Conan 2, the C++ toolchain, and Foxglove Studio all live in containers — no host install of Foxglove is needed.

## First-time setup

```bash
git clone <this-repo>
cd PathFinder
make image-build      # builds the dev container (~5-10 min, one-time)
make install          # runs conan install inside the container
make build            # colcon build of all 9 packages (~1 min)
make generate-map     # generates maps/demo_world.bt
```

## Run the demo

Pick one (both bring up the simulation, all four planners, and the Foxglove bridge):

```bash
make launch-demo-drone   # drone in 3D world with overhangs, vertical posts, low tunnel
make launch-demo-car     # diff-drive car at floor level
```

Then open **http://localhost:8080** in your browser (the Foxglove container is started automatically by the launch targets; `foxglove/layouts/default.json` is bind-mounted as the default layout):

1. Click **Open connection** → select **Foxglove WebSocket** → connect to `ws://localhost:8765`
2. You should see the voxel map, the robot's TF tree, and panels for `planner_status` + `algorithm_selection`

### Send a goal

In another terminal:

```bash
make sh                  # interactive shell in the dev container
# inside the container:
source install/setup.bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 9.0, y: 9.0, z: 1.5}}}'
```

The active planner (default `astar`) will plan a path; the robot follows it in Gazebo and Foxglove visualizes everything.

### Switch algorithms on the fly

```bash
ros2 topic pub --once /algorithm_selection pathfinder_msgs/AlgorithmSelection \
  '{algorithm_name: dijkstra}'    # or astar, rrt, dstar_lite
```

Then publish a new goal. Watch the comparison: Dijkstra explores ~35× more nodes than A* for the same path; D* Lite returns near-instantly on its second call (incremental).

## Make targets

| Target | Description |
|---|---|
| `make image-build` | Build the dev container image |
| `make install` | `conan install` inside the container |
| `make build` | `colcon build` all packages |
| `make test` | `colcon test` |
| `make generate-map` | Regenerate `maps/demo_world.bt` |
| `make launch-demo-drone` | Full demo: drone + sim + all planners + Foxglove bridge |
| `make launch-demo-car` | Same with the car |
| `make launch-drone` / `launch-car` | Just the robot + bridge (no planners) |
| `make launch-algos` | Just the four planner action servers |
| `make launch-{astar,dijkstra,rrt}` | Individual planner action server |
| `make foxglove-up` / `foxglove-down` | Start/stop the Foxglove Studio web container (`http://localhost:8080`) |
| `make sh` | Interactive shell in the dev container |
| `make down` / `make clean` | Stop containers / remove image |

## What's inside

Nine ROS2 packages under `src/`:

- `pathfinder_msgs` — custom messages + `RequestPath.action`
- `pathfinder_core` — abstract planner base class, scheduler, map publisher, shared voxel-grid utilities
- `pathfinder_bringup` — launch composition
- `pathfinder_robot_drone`, `pathfinder_robot_car` — robot models + path followers
- `pathfinder_algo_astar`, `pathfinder_algo_dijkstra`, `pathfinder_algo_rrt`, `pathfinder_algo_dstar_lite` — the four planning algorithms

CI (`.github/workflows/ci.yml`) builds + tests on push and PR.
