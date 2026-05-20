#include "pathfinder_dijkstra/dijkstra_grid.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <vector>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

namespace pathfinder_dijkstra
{

namespace
{

constexpr std::array<std::array<int, 3>, 26> kNeighborOffsets = []() {
  std::array<std::array<int, 3>, 26> out{};
  int i = 0;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }
        out[i++] = {dx, dy, dz};
      }
    }
  }
  return out;
}();

WorldPoint to_world_point(const std::array<double, 3> & a)
{
  return {a[0], a[1], a[2]};
}

std::vector<WorldPoint> sample_world_flag(
  const pathfinder_core::InflatedVoxelGrid & grid,
  const std::vector<std::uint8_t> & flags,
  std::size_t max_count)
{
  std::vector<WorldPoint> out;
  std::size_t count = 0;
  for (auto f : flags) {
    if (f) {
      ++count;
    }
  }
  if (count == 0) {
    return out;
  }
  const std::size_t stride = std::max<std::size_t>(1, count / max_count);
  out.reserve(std::min(count, max_count));
  std::size_t seen = 0;
  for (std::size_t i = 0; i < flags.size(); ++i) {
    if (!flags[i]) {
      continue;
    }
    if ((seen++ % stride) != 0) {
      continue;
    }
    out.push_back(to_world_point(grid.voxel_to_world(grid.from_linear_index(i))));
    if (out.size() >= max_count) {
      break;
    }
  }
  return out;
}

std::vector<WorldPoint> reconstruct_world_path(
  const pathfinder_core::InflatedVoxelGrid & grid,
  const std::vector<std::int32_t> & parent,
  std::int32_t end_lin)
{
  std::vector<std::int32_t> chain;
  chain.push_back(end_lin);
  while (parent[chain.back()] >= 0) {
    chain.push_back(parent[chain.back()]);
    if (chain.size() > 100000) {
      break;
    }
  }
  std::reverse(chain.begin(), chain.end());
  std::vector<WorldPoint> path;
  path.reserve(chain.size());
  for (auto lin : chain) {
    path.push_back(to_world_point(
      grid.voxel_to_world(grid.from_linear_index(static_cast<std::size_t>(lin)))));
  }
  return path;
}

}  // namespace

DijkstraGrid::DijkstraGrid(const octomap::OcTree & tree, double robot_radius)
: grid_(tree, robot_radius)
{
}

PlanOutcome DijkstraGrid::plan(
  const PlanRequest & req,
  std::function<void(const PlanProgress &)> on_progress,
  int feedback_node_stride,
  double feedback_time_stride_sec) const
{
  PlanOutcome out{};
  out.success = false;
  out.nodes_explored = 0;
  out.best_cost = std::numeric_limits<double>::infinity();

  const auto start_idx = grid_.world_to_voxel(req.start.x, req.start.y, req.start.z);
  const auto goal_idx = grid_.world_to_voxel(req.goal.x, req.goal.y, req.goal.z);

  if (!grid_.in_bounds(start_idx)) {
    out.message = "start is outside map bounds";
    return out;
  }
  if (!grid_.in_bounds(goal_idx)) {
    out.message = "goal is outside map bounds";
    return out;
  }
  if (grid_.is_occupied(start_idx)) {
    out.message = "start voxel is occupied (after inflation)";
    return out;
  }
  if (grid_.is_occupied(goal_idx)) {
    out.message = "goal voxel is occupied (after inflation)";
    return out;
  }

  const double res = grid_.resolution();
  std::array<double, 26> step_costs{};
  for (std::size_t i = 0; i < kNeighborOffsets.size(); ++i) {
    const auto & d = kNeighborOffsets[i];
    step_costs[i] = res * std::sqrt(
      static_cast<double>(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]));
  }

  const std::size_t n_cells = grid_.cell_count();
  std::vector<double> g(n_cells, std::numeric_limits<double>::infinity());
  std::vector<std::int32_t> parent(n_cells, -1);
  std::vector<std::uint8_t> closed(n_cells, 0);
  std::vector<std::uint8_t> in_open(n_cells, 0);

  struct PqEntry
  {
    double cost;
    std::int32_t idx;
  };
  struct PqCmp
  {
    bool operator()(const PqEntry & a, const PqEntry & b) const
    {
      return a.cost > b.cost;
    }
  };
  std::priority_queue<PqEntry, std::vector<PqEntry>, PqCmp> open;

  const auto dims = grid_.dims();
  const int dim_x = dims[0];
  const int dim_y = dims[1];
  const int dim_z = dims[2];

  const std::int32_t start_lin = static_cast<std::int32_t>(grid_.linear_index(start_idx));
  const std::int32_t goal_lin = static_cast<std::int32_t>(grid_.linear_index(goal_idx));
  g[start_lin] = 0.0;
  open.push({0.0, start_lin});
  in_open[start_lin] = 1;

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(req.max_plan_time_sec);
  auto last_feedback = std::chrono::steady_clock::now();
  std::int32_t expanded = 0;

  while (!open.empty()) {
    PqEntry top = open.top();
    open.pop();
    if (closed[top.idx]) {
      continue;
    }
    if (top.cost > g[top.idx] + 1e-9) {
      continue;
    }
    closed[top.idx] = 1;
    in_open[top.idx] = 0;
    ++expanded;

    if (top.idx == goal_lin) {
      out.success = true;
      out.message = "ok";
      out.best_cost = top.cost;
      out.nodes_explored = expanded;
      out.path = reconstruct_world_path(grid_, parent, top.idx);
      return out;
    }

    if ((expanded & 1023) == 0 && std::chrono::steady_clock::now() > deadline) {
      out.message = "time budget exceeded";
      out.nodes_explored = expanded;
      return out;
    }

    const pathfinder_core::VoxelIndex cur = grid_.from_linear_index(
      static_cast<std::size_t>(top.idx));
    for (std::size_t k = 0; k < kNeighborOffsets.size(); ++k) {
      const auto & d = kNeighborOffsets[k];
      const int nx = cur.x + d[0];
      const int ny = cur.y + d[1];
      const int nz = cur.z + d[2];
      if (nx < 0 || ny < 0 || nz < 0 || nx >= dim_x || ny >= dim_y || nz >= dim_z) {
        continue;
      }
      const std::size_t nb_lin = grid_.linear_index({nx, ny, nz});
      if (grid_.is_occupied({nx, ny, nz}) || closed[nb_lin]) {
        continue;
      }
      const double tentative = top.cost + step_costs[k];
      if (tentative < g[nb_lin] - 1e-9) {
        g[nb_lin] = tentative;
        parent[nb_lin] = top.idx;
        open.push({tentative, static_cast<std::int32_t>(nb_lin)});
        in_open[nb_lin] = 1;
      }
    }

    const bool by_count = feedback_node_stride > 0 &&
      (expanded % feedback_node_stride) == 0;
    bool by_time = false;
    if (feedback_time_stride_sec > 0.0 && (expanded & 511) == 0) {
      const auto now = std::chrono::steady_clock::now();
      by_time = std::chrono::duration<double>(now - last_feedback).count() >=
        feedback_time_stride_sec;
    }
    if (on_progress && (by_count || by_time)) {
      last_feedback = std::chrono::steady_clock::now();
      PlanProgress prog{};
      prog.nodes_explored = expanded;
      prog.best_cost_so_far = top.cost;
      prog.resolution = res;
      prog.start = req.start;
      prog.current_best = reconstruct_world_path(grid_, parent, top.idx);
      prog.explored_set = sample_world_flag(grid_, closed, 500);
      prog.frontier_set = sample_world_flag(grid_, in_open, 500);
      on_progress(prog);
    }
  }

  out.message = "no path found";
  out.nodes_explored = expanded;
  return out;
}

}  // namespace pathfinder_dijkstra
