#include "pathfinder_dstar_lite/dstar_lite_grid.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

namespace pathfinder_dstar_lite
{

namespace
{

double voxel_distance_meters(
  const pathfinder_core::VoxelIndex & a,
  const pathfinder_core::VoxelIndex & b,
  double resolution)
{
  const double dx = static_cast<double>(a.x - b.x);
  const double dy = static_cast<double>(a.y - b.y);
  const double dz = static_cast<double>(a.z - b.z);
  return std::sqrt(dx * dx + dy * dy + dz * dz) * resolution;
}

}  // namespace

DstarLiteGrid::DstarLiteGrid(
  std::unique_ptr<octomap::OcTree> tree,
  double robot_radius,
  std::uint64_t map_signature)
: tree_(std::move(tree)),
  grid_(*tree_, robot_radius),
  map_signature_(map_signature),
  resolution_(grid_.resolution())
{
  const std::size_t n = grid_.cell_count();
  g_.assign(n, kInf);
  rhs_.assign(n, kInf);
  in_queue_.assign(n, 0);
  queue_key_.assign(n, Key{kInf, kInf});
}

double DstarLiteGrid::heuristic(std::size_t a, std::size_t b) const
{
  const auto va = grid_.from_linear_index(a);
  const auto vb = grid_.from_linear_index(b);
  return voxel_distance_meters(va, vb, resolution_);
}

double DstarLiteGrid::edge_cost(std::size_t a, std::size_t b) const
{
  const auto va = grid_.from_linear_index(a);
  const auto vb = grid_.from_linear_index(b);
  if (grid_.is_occupied(va) || grid_.is_occupied(vb)) {
    return kInf;
  }
  return voxel_distance_meters(va, vb, resolution_);
}

Key DstarLiteGrid::calculate_key(std::size_t flat) const
{
  const double min_g_rhs = std::min(g_[flat], rhs_[flat]);
  if (min_g_rhs == kInf) {
    return Key{kInf, kInf};
  }
  return Key{min_g_rhs + heuristic(s_start_flat_, flat) + k_m_, min_g_rhs};
}

std::vector<std::size_t> DstarLiteGrid::neighbors(std::size_t flat) const
{
  const auto v = grid_.from_linear_index(flat);
  const auto nb_idx = grid_.neighbors_26(v);
  std::vector<std::size_t> out;
  out.reserve(nb_idx.size());
  for (const auto & nb : nb_idx) {
    out.push_back(grid_.linear_index(nb));
  }
  return out;
}

void DstarLiteGrid::initialize(std::size_t start_flat, std::size_t goal_flat)
{
  const std::size_t n = grid_.cell_count();
  g_.assign(n, kInf);
  rhs_.assign(n, kInf);
  in_queue_.assign(n, 0);
  queue_key_.assign(n, Key{kInf, kInf});
  open_ = OpenQueue();
  k_m_ = 0.0;
  s_start_flat_ = start_flat;
  s_goal_flat_ = goal_flat;
  rhs_[goal_flat] = 0.0;
  const Key key{heuristic(start_flat, goal_flat), 0.0};
  queue_key_[goal_flat] = key;
  in_queue_[goal_flat] = 1;
  open_.push({key, goal_flat});
}

void DstarLiteGrid::shift_start(std::size_t new_start_flat)
{
  k_m_ += heuristic(s_start_flat_, new_start_flat);
  s_start_flat_ = new_start_flat;
}

void DstarLiteGrid::update_vertex(std::size_t flat)
{
  if (g_[flat] != rhs_[flat]) {
    const Key key = calculate_key(flat);
    queue_key_[flat] = key;
    in_queue_[flat] = 1;
    open_.push({key, flat});
  } else if (in_queue_[flat]) {
    in_queue_[flat] = 0;
    queue_key_[flat] = Key{kInf, kInf};
  }
}

ComputeStats DstarLiteGrid::compute_shortest_path(
  double max_time_sec,
  std::int32_t feedback_every_nodes,
  double feedback_every_seconds,
  const FeedbackHook & on_feedback)
{
  ComputeStats stats;
  const auto start_wall = std::chrono::steady_clock::now();
  auto last_feedback = start_wall;
  std::int32_t since_feedback = 0;

  while (!open_.empty()) {
    const auto now_wall = std::chrono::steady_clock::now();
    const double elapsed_sec =
      std::chrono::duration<double>(now_wall - start_wall).count();
    if (elapsed_sec > max_time_sec) {
      stats.timed_out = true;
      return stats;
    }

    QueueEntry top = open_.top();
    const Key start_key = calculate_key(s_start_flat_);
    const bool start_inconsistent = rhs_[s_start_flat_] != g_[s_start_flat_];
    if (!(top.key < start_key) && !start_inconsistent) {
      stats.reached_start = true;
      return stats;
    }

    open_.pop();

    if (!in_queue_[top.flat]) {
      continue;
    }
    if (queue_key_[top.flat] != top.key) {
      continue;
    }

    const Key k_new = calculate_key(top.flat);
    if (top.key < k_new) {
      queue_key_[top.flat] = k_new;
      open_.push({k_new, top.flat});
      continue;
    }

    in_queue_[top.flat] = 0;
    queue_key_[top.flat] = Key{kInf, kInf};

    if (g_[top.flat] > rhs_[top.flat]) {
      g_[top.flat] = rhs_[top.flat];
      for (std::size_t pred : neighbors(top.flat)) {
        if (pred != s_goal_flat_) {
          const double c = edge_cost(pred, top.flat);
          if (c != kInf) {
            rhs_[pred] = std::min(rhs_[pred], c + g_[top.flat]);
          }
        }
        update_vertex(pred);
      }
    } else {
      const double g_old = g_[top.flat];
      g_[top.flat] = kInf;
      auto preds_and_self = neighbors(top.flat);
      preds_and_self.push_back(top.flat);
      for (std::size_t s : preds_and_self) {
        if (s != s_goal_flat_) {
          const double c = edge_cost(s, top.flat);
          if (rhs_[s] == c + g_old && c != kInf) {
            double best = kInf;
            for (std::size_t succ : neighbors(s)) {
              const double cs = edge_cost(s, succ);
              if (cs == kInf) {
                continue;
              }
              best = std::min(best, cs + g_[succ]);
            }
            rhs_[s] = best;
          }
        }
        update_vertex(s);
      }
    }

    ++stats.expansions;
    ++since_feedback;

    const double since_fb_sec =
      std::chrono::duration<double>(now_wall - last_feedback).count();
    if (on_feedback &&
        (since_feedback >= feedback_every_nodes ||
         since_fb_sec >= feedback_every_seconds))
    {
      on_feedback(stats.expansions);
      since_feedback = 0;
      last_feedback = now_wall;
    }
  }

  stats.reached_start = (g_[s_start_flat_] != kInf);
  return stats;
}

std::vector<pathfinder_core::VoxelIndex> DstarLiteGrid::extract_path(
  std::size_t max_steps) const
{
  std::vector<pathfinder_core::VoxelIndex> out;
  if (g_[s_start_flat_] == kInf) {
    return out;
  }
  std::size_t cur = s_start_flat_;
  out.push_back(grid_.from_linear_index(cur));
  for (std::size_t step = 0; step < max_steps; ++step) {
    if (cur == s_goal_flat_) {
      return out;
    }
    double best_cost = kInf;
    std::size_t best_next = cur;
    for (std::size_t succ : neighbors(cur)) {
      if (g_[succ] == kInf) {
        continue;
      }
      const double c = edge_cost(cur, succ);
      if (c == kInf) {
        continue;
      }
      const double total = c + g_[succ];
      if (total < best_cost) {
        best_cost = total;
        best_next = succ;
      }
    }
    if (best_next == cur) {
      out.clear();
      return out;
    }
    cur = best_next;
    out.push_back(grid_.from_linear_index(cur));
  }
  out.clear();
  return out;
}

std::vector<std::size_t> DstarLiteGrid::sample_open_flats(std::size_t max_count) const
{
  std::vector<std::size_t> out;
  if (max_count == 0) {
    return out;
  }
  OpenQueue copy = open_;
  while (!copy.empty() && out.size() < max_count) {
    const auto top = copy.top();
    copy.pop();
    if (!in_queue_[top.flat] || queue_key_[top.flat] != top.key) {
      continue;
    }
    out.push_back(top.flat);
  }
  return out;
}

std::vector<std::size_t> DstarLiteGrid::sample_locked_flats(std::size_t max_count) const
{
  std::vector<std::size_t> out;
  if (max_count == 0) {
    return out;
  }
  std::size_t total = 0;
  for (std::size_t i = 0; i < g_.size(); ++i) {
    if (g_[i] != kInf && g_[i] == rhs_[i]) {
      ++total;
    }
  }
  if (total == 0) {
    return out;
  }
  const std::size_t stride = std::max<std::size_t>(1, total / max_count);
  std::size_t seen = 0;
  for (std::size_t i = 0; i < g_.size() && out.size() < max_count; ++i) {
    if (g_[i] != kInf && g_[i] == rhs_[i]) {
      if ((seen % stride) == 0) {
        out.push_back(i);
      }
      ++seen;
    }
  }
  return out;
}

}  // namespace pathfinder_dstar_lite
