#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <string>
#include <vector>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

namespace pathfinder_dijkstra
{

struct VoxelIndex
{
  int x;
  int y;
  int z;

  bool operator==(const VoxelIndex & other) const noexcept
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct WorldPoint
{
  double x;
  double y;
  double z;
};

struct PlanRequest
{
  WorldPoint start;
  WorldPoint goal;
  double robot_radius;
  double max_plan_time_sec;
};

struct PlanProgress
{
  std::int32_t nodes_explored;
  double best_cost_so_far;
  std::vector<WorldPoint> current_best;
  std::vector<WorldPoint> explored_set;
  std::vector<WorldPoint> frontier_set;
  WorldPoint start;
  double resolution;
};

struct PlanOutcome
{
  bool success;
  std::string message;
  std::vector<WorldPoint> path;
  std::int32_t nodes_explored;
  double best_cost;
};

class DijkstraGrid
{
public:
  explicit DijkstraGrid(const octomap::OcTree & tree)
  : resolution_(tree.getResolution())
  {
    double min_x;
    double min_y;
    double min_z;
    double max_x;
    double max_y;
    double max_z;
    tree.getMetricMin(min_x, min_y, min_z);
    tree.getMetricMax(max_x, max_y, max_z);
    origin_ = {min_x, min_y, min_z};

    dim_x_ = std::max(1, static_cast<int>(std::ceil((max_x - min_x) / resolution_)) + 1);
    dim_y_ = std::max(1, static_cast<int>(std::ceil((max_y - min_y) / resolution_)) + 1);
    dim_z_ = std::max(1, static_cast<int>(std::ceil((max_z - min_z) / resolution_)) + 1);
    occupied_.assign(static_cast<std::size_t>(dim_x_) * dim_y_ * dim_z_, 0);

    for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
      if (!tree.isNodeOccupied(*it)) {
        continue;
      }
      const double size = it.getSize();
      const int span = std::max(1, static_cast<int>(std::round(size / resolution_)));
      const auto base = world_to_index({it.getX() - size / 2.0 + resolution_ / 2.0,
                                        it.getY() - size / 2.0 + resolution_ / 2.0,
                                        it.getZ() - size / 2.0 + resolution_ / 2.0});
      for (int dx = 0; dx < span; ++dx) {
        for (int dy = 0; dy < span; ++dy) {
          for (int dz = 0; dz < span; ++dz) {
            VoxelIndex v{base.x + dx, base.y + dy, base.z + dz};
            if (in_bounds(v)) {
              occupied_[linear(v)] = 1;
            }
          }
        }
      }
    }
  }

  void inflate(double robot_radius)
  {
    if (robot_radius <= 0.0) {
      return;
    }
    const int r = static_cast<int>(std::ceil(robot_radius / resolution_));
    if (r <= 0) {
      return;
    }
    std::vector<std::uint8_t> source = occupied_;
    for (int x = 0; x < dim_x_; ++x) {
      for (int y = 0; y < dim_y_; ++y) {
        for (int z = 0; z < dim_z_; ++z) {
          if (!source[linear({x, y, z})]) {
            continue;
          }
          for (int dx = -r; dx <= r; ++dx) {
            const int nx = x + dx;
            if (nx < 0 || nx >= dim_x_) {
              continue;
            }
            for (int dy = -r; dy <= r; ++dy) {
              const int ny = y + dy;
              if (ny < 0 || ny >= dim_y_) {
                continue;
              }
              for (int dz = -r; dz <= r; ++dz) {
                const int nz = z + dz;
                if (nz < 0 || nz >= dim_z_) {
                  continue;
                }
                occupied_[linear({nx, ny, nz})] = 1;
              }
            }
          }
        }
      }
    }
  }

  VoxelIndex world_to_index(const WorldPoint & p) const
  {
    return {
      static_cast<int>(std::floor((p.x - origin_.x) / resolution_)),
      static_cast<int>(std::floor((p.y - origin_.y) / resolution_)),
      static_cast<int>(std::floor((p.z - origin_.z) / resolution_)),
    };
  }

  WorldPoint index_to_world(const VoxelIndex & v) const
  {
    return {
      origin_.x + (static_cast<double>(v.x) + 0.5) * resolution_,
      origin_.y + (static_cast<double>(v.y) + 0.5) * resolution_,
      origin_.z + (static_cast<double>(v.z) + 0.5) * resolution_,
    };
  }

  bool in_bounds(const VoxelIndex & v) const
  {
    return v.x >= 0 && v.y >= 0 && v.z >= 0 &&
           v.x < dim_x_ && v.y < dim_y_ && v.z < dim_z_;
  }

  bool is_occupied(const VoxelIndex & v) const
  {
    return in_bounds(v) && occupied_[linear(v)] != 0;
  }

  double resolution() const { return resolution_; }

  PlanOutcome plan(
    const PlanRequest & req,
    std::function<void(const PlanProgress &)> on_progress,
    int feedback_node_stride,
    double feedback_time_stride_sec) const
  {
    PlanOutcome out{};
    out.success = false;
    out.nodes_explored = 0;
    out.best_cost = std::numeric_limits<double>::infinity();

    const auto start_idx = world_to_index(req.start);
    const auto goal_idx = world_to_index(req.goal);

    if (!in_bounds(start_idx)) {
      out.message = "start is outside map bounds";
      return out;
    }
    if (!in_bounds(goal_idx)) {
      out.message = "goal is outside map bounds";
      return out;
    }
    if (is_occupied(start_idx)) {
      out.message = "start voxel is occupied (after inflation)";
      return out;
    }
    if (is_occupied(goal_idx)) {
      out.message = "goal voxel is occupied (after inflation)";
      return out;
    }

    static const std::array<std::array<int, 3>, 26> neighbors = make_neighbors();
    std::array<double, 26> step_costs{};
    for (std::size_t i = 0; i < neighbors.size(); ++i) {
      const auto & d = neighbors[i];
      step_costs[i] = resolution_ *
        std::sqrt(static_cast<double>(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]));
    }

    const std::size_t n_cells = static_cast<std::size_t>(dim_x_) * dim_y_ * dim_z_;
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

    const std::int32_t start_lin = linear(start_idx);
    const std::int32_t goal_lin = linear(goal_idx);
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
        out.path = reconstruct_world_path(parent, top.idx);
        return out;
      }

      if ((expanded & 1023) == 0 && std::chrono::steady_clock::now() > deadline) {
        out.message = "time budget exceeded";
        out.nodes_explored = expanded;
        return out;
      }

      const VoxelIndex cur = from_linear(top.idx);
      for (std::size_t k = 0; k < neighbors.size(); ++k) {
        const auto & d = neighbors[k];
        VoxelIndex nb{cur.x + d[0], cur.y + d[1], cur.z + d[2]};
        if (!in_bounds(nb)) {
          continue;
        }
        const std::int32_t nb_lin = linear(nb);
        if (occupied_[nb_lin] || closed[nb_lin]) {
          continue;
        }
        const double tentative = top.cost + step_costs[k];
        if (tentative < g[nb_lin] - 1e-9) {
          g[nb_lin] = tentative;
          parent[nb_lin] = top.idx;
          open.push({tentative, nb_lin});
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
        prog.resolution = resolution_;
        prog.start = req.start;
        prog.current_best = reconstruct_world_path(parent, top.idx);
        prog.explored_set = sample_world_flag(closed, 500);
        prog.frontier_set = sample_world_flag(in_open, 500);
        on_progress(prog);
      }
    }

    out.message = "no path found";
    out.nodes_explored = expanded;
    return out;
  }

private:
  static constexpr std::array<std::array<int, 3>, 26> make_neighbors()
  {
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
  }

  std::int32_t linear(const VoxelIndex & v) const
  {
    return static_cast<std::int32_t>(
      (static_cast<std::size_t>(v.x) * dim_y_ + v.y) * dim_z_ + v.z);
  }

  VoxelIndex from_linear(std::int32_t lin) const
  {
    const int z = lin % dim_z_;
    const int rem = lin / dim_z_;
    const int y = rem % dim_y_;
    const int x = rem / dim_y_;
    return {x, y, z};
  }

  std::vector<WorldPoint> reconstruct_world_path(
    const std::vector<std::int32_t> & parent,
    std::int32_t end_lin) const
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
      path.push_back(index_to_world(from_linear(lin)));
    }
    return path;
  }

  std::vector<WorldPoint> sample_world_flag(
    const std::vector<std::uint8_t> & flags,
    std::size_t max_count) const
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
      out.push_back(index_to_world(from_linear(static_cast<std::int32_t>(i))));
      if (out.size() >= max_count) {
        break;
      }
    }
    return out;
  }

  double resolution_;
  WorldPoint origin_;
  int dim_x_ = 0;
  int dim_y_ = 0;
  int dim_z_ = 0;
  std::vector<std::uint8_t> occupied_;
};

}  // namespace pathfinder_dijkstra
