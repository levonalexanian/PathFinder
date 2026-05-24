#include "pathfinder_algo_dijkstra/dijkstra_core.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <thread>
#include <vector>

#include <rclcpp/logging.hpp>

namespace pathfinder_algo_dijkstra
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

std::vector<std::size_t> sample_flat(
  const std::vector<std::uint8_t> & flags,
  std::size_t max_count)
{
  std::vector<std::size_t> out;
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
    out.push_back(i);
    if (out.size() >= max_count) {
      break;
    }
  }
  return out;
}

std::vector<pathfinder_core::VoxelIndex> reconstruct_path(
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
  std::vector<pathfinder_core::VoxelIndex> path;
  path.reserve(chain.size());
  for (auto lin : chain) {
    path.push_back(grid.from_linear_index(static_cast<std::size_t>(lin)));
  }
  return path;
}

}  // namespace

DijkstraCore::DijkstraCore(const rclcpp::Logger & logger)
: logger_(logger)
{
}

DijkstraResult DijkstraCore::plan(
  const pathfinder_core::InflatedVoxelGrid & grid,
  const pathfinder_core::VoxelIndex & start,
  const pathfinder_core::VoxelIndex & goal,
  const DijkstraParams & params,
  FeedbackCallback feedback)
{
  const auto clock_start = std::chrono::steady_clock::now();

  DijkstraResult out{};
  out.success = false;
  out.nodes_expanded = 0;

  auto finalize_duration = [&]() {
    const auto clock_end = std::chrono::steady_clock::now();
    out.plan_duration_ms = std::chrono::duration<double, std::milli>(
      clock_end - clock_start).count();
  };

  if (!grid.in_bounds(start)) {
    out.message = "start is outside map bounds";
    finalize_duration();
    return out;
  }
  if (!grid.in_bounds(goal)) {
    out.message = "goal is outside map bounds";
    finalize_duration();
    return out;
  }
  if (grid.is_occupied(start)) {
    out.message = "start voxel is occupied (after inflation)";
    finalize_duration();
    return out;
  }
  if (grid.is_occupied(goal)) {
    out.message = "goal voxel is occupied (after inflation)";
    finalize_duration();
    return out;
  }

  const double res = grid.resolution();
  std::array<double, 26> step_costs{};
  for (std::size_t i = 0; i < kNeighborOffsets.size(); ++i) {
    const auto & d = kNeighborOffsets[i];
    step_costs[i] = res * std::sqrt(
      static_cast<double>(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]));
  }

  const std::size_t n_cells = grid.cell_count();
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

  const auto dims = grid.dims();
  const int dim_x = dims[0];
  const int dim_y = dims[1];
  const int dim_z = dims[2];

  const std::int32_t start_lin = static_cast<std::int32_t>(grid.linear_index(start));
  const std::int32_t goal_lin = static_cast<std::int32_t>(grid.linear_index(goal));
  g[start_lin] = 0.0;
  open.push({0.0, start_lin});
  in_open[start_lin] = 1;

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(params.max_plan_time_sec);
  auto last_feedback = std::chrono::steady_clock::now();
  std::int32_t expanded = 0;
  double last_cost = 0.0;

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
    last_cost = top.cost;

    if (top.idx == goal_lin) {
      out.success = true;
      out.message = "ok";
      out.nodes_expanded = expanded;
      out.path = reconstruct_path(grid, parent, top.idx);
      finalize_duration();
      return out;
    }

    if ((expanded & 1023) == 0 && std::chrono::steady_clock::now() > deadline) {
      out.message = "time budget exceeded";
      out.nodes_expanded = expanded;
      finalize_duration();
      return out;
    }

    const pathfinder_core::VoxelIndex cur = grid.from_linear_index(
      static_cast<std::size_t>(top.idx));
    for (std::size_t k = 0; k < kNeighborOffsets.size(); ++k) {
      const auto & d = kNeighborOffsets[k];
      const int nx = cur.x + d[0];
      const int ny = cur.y + d[1];
      const int nz = cur.z + d[2];
      if (nx < 0 || ny < 0 || nz < 0 || nx >= dim_x || ny >= dim_y || nz >= dim_z) {
        continue;
      }
      const std::size_t nb_lin = grid.linear_index({nx, ny, nz});
      if (grid.is_occupied({nx, ny, nz}) || closed[nb_lin]) {
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

    const bool by_count = params.feedback_node_stride > 0 &&
      (expanded % params.feedback_node_stride) == 0;
    bool by_time = false;
    if (params.feedback_time_stride_sec > 0.0 && (expanded & 511) == 0) {
      const auto now = std::chrono::steady_clock::now();
      by_time = std::chrono::duration<double>(now - last_feedback).count() >=
        params.feedback_time_stride_sec;
    }
    if (feedback && (by_count || by_time)) {
      last_feedback = std::chrono::steady_clock::now();
      DijkstraFeedback fb{};
      fb.nodes_explored = expanded;
      fb.best_cost_so_far = top.cost;
      fb.best_partial_path = reconstruct_path(grid, parent, top.idx);
      fb.sampled_closed_flat = sample_flat(closed, 500);
      fb.sampled_open_flat = sample_flat(in_open, 500);
      feedback(fb);
      if (params.viz_delay_ms > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(params.viz_delay_ms));
      }
    }
  }

  out.message = "no path found";
  out.nodes_expanded = expanded;
  (void)last_cost;
  finalize_duration();
  RCLCPP_DEBUG(
    logger_, "dijkstra: exhausted open set after %d expansions", expanded);
  return out;
}

}  // namespace pathfinder_algo_dijkstra
