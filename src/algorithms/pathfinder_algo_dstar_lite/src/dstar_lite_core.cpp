#include "pathfinder_algo_dstar_lite/dstar_lite_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/logging.hpp>

namespace pathfinder_algo_dstar_lite
{

namespace
{

constexpr std::size_t kMaxSampledOpen = 500;
constexpr std::size_t kMaxSampledClosed = 500;

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

DstarLiteCore::DstarLiteCore(const rclcpp::Logger & logger)
: logger_(logger)
{
}

void DstarLiteCore::reset()
{
  g_.clear();
  rhs_.clear();
  in_queue_.clear();
  queue_key_.clear();
  open_ = OpenQueue();
  k_m_ = 0.0;
  s_start_flat_ = 0;
  s_goal_flat_ = 0;
  resolution_ = 0.0;
  last_grid_ = nullptr;
  last_grid_cells_ = 0;
  last_start_flat_.reset();
  last_goal_flat_.reset();
  initialized_ = false;
}

double DstarLiteCore::heuristic(
  const pathfinder_core::InflatedVoxelGrid & grid,
  std::size_t a, std::size_t b) const
{
  const auto va = grid.from_linear_index(a);
  const auto vb = grid.from_linear_index(b);
  return voxel_distance_meters(va, vb, resolution_);
}

double DstarLiteCore::edge_cost(
  const pathfinder_core::InflatedVoxelGrid & grid,
  std::size_t a, std::size_t b) const
{
  const auto va = grid.from_linear_index(a);
  const auto vb = grid.from_linear_index(b);
  if (grid.is_occupied(va) || grid.is_occupied(vb)) {
    return kInf;
  }
  return voxel_distance_meters(va, vb, resolution_);
}

DstarLiteCore::Key DstarLiteCore::calculate_key(
  const pathfinder_core::InflatedVoxelGrid & grid, std::size_t flat) const
{
  const double min_g_rhs = std::min(g_[flat], rhs_[flat]);
  if (min_g_rhs == kInf) {
    return Key{kInf, kInf};
  }
  return Key{min_g_rhs + heuristic(grid, s_start_flat_, flat) + k_m_, min_g_rhs};
}

std::vector<std::size_t> DstarLiteCore::neighbors(
  const pathfinder_core::InflatedVoxelGrid & grid, std::size_t flat) const
{
  const auto v = grid.from_linear_index(flat);
  const auto nb_idx = grid.neighbors_26(v);
  std::vector<std::size_t> out;
  out.reserve(nb_idx.size());
  for (const auto & nb : nb_idx) {
    out.push_back(grid.linear_index(nb));
  }
  return out;
}

void DstarLiteCore::initialize_search(
  const pathfinder_core::InflatedVoxelGrid & grid,
  std::size_t start_flat,
  std::size_t goal_flat)
{
  const std::size_t n = grid.cell_count();
  g_.assign(n, kInf);
  rhs_.assign(n, kInf);
  in_queue_.assign(n, 0);
  queue_key_.assign(n, Key{kInf, kInf});
  open_ = OpenQueue();
  k_m_ = 0.0;
  s_start_flat_ = start_flat;
  s_goal_flat_ = goal_flat;
  resolution_ = grid.resolution();
  rhs_[goal_flat] = 0.0;
  const Key key{heuristic(grid, start_flat, goal_flat), 0.0};
  queue_key_[goal_flat] = key;
  in_queue_[goal_flat] = 1;
  open_.push({key, goal_flat});
  initialized_ = true;
}

void DstarLiteCore::shift_start(
  const pathfinder_core::InflatedVoxelGrid & grid,
  std::size_t new_start_flat)
{
  k_m_ += heuristic(grid, s_start_flat_, new_start_flat);
  s_start_flat_ = new_start_flat;
}

void DstarLiteCore::update_vertex(
  const pathfinder_core::InflatedVoxelGrid & grid, std::size_t flat)
{
  if (g_[flat] != rhs_[flat]) {
    const Key key = calculate_key(grid, flat);
    queue_key_[flat] = key;
    in_queue_[flat] = 1;
    open_.push({key, flat});
  } else if (in_queue_[flat]) {
    in_queue_[flat] = 0;
    queue_key_[flat] = Key{kInf, kInf};
  }
}

DstarLiteCore::ComputeStats DstarLiteCore::compute_shortest_path(
  const pathfinder_core::InflatedVoxelGrid & grid,
  const DstarLiteParams & params,
  const FeedbackCallback & feedback)
{
  ComputeStats stats;
  const auto start_wall = std::chrono::steady_clock::now();
  auto last_feedback = start_wall;
  std::int32_t since_feedback = 0;

  const std::int32_t feedback_every_nodes =
    std::max<std::int32_t>(1, static_cast<std::int32_t>(params.feedback_every_nodes));
  const double feedback_every_seconds = params.feedback_every_seconds;

  auto emit_feedback = [&](std::int32_t expansions) {
    if (!feedback) {
      return;
    }
    DstarLiteFeedback fb;
    fb.nodes_explored = expansions;
    fb.sampled_open_flat = sample_open_flats(kMaxSampledOpen);
    fb.sampled_closed_flat = sample_locked_flats(kMaxSampledClosed);
    fb.best_partial_path = extract_path(grid, static_cast<std::size_t>(grid.cell_count()));
    fb.best_cost_so_far = g_[s_start_flat_];
    feedback(fb);
  };

  while (!open_.empty()) {
    const auto now_wall = std::chrono::steady_clock::now();
    const double elapsed_sec =
      std::chrono::duration<double>(now_wall - start_wall).count();
    if (elapsed_sec > params.max_plan_time_sec) {
      stats.timed_out = true;
      return stats;
    }

    QueueEntry top = open_.top();
    const Key start_key = calculate_key(grid, s_start_flat_);
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

    const Key k_new = calculate_key(grid, top.flat);
    if (top.key < k_new) {
      queue_key_[top.flat] = k_new;
      open_.push({k_new, top.flat});
      continue;
    }

    in_queue_[top.flat] = 0;
    queue_key_[top.flat] = Key{kInf, kInf};

    if (g_[top.flat] > rhs_[top.flat]) {
      g_[top.flat] = rhs_[top.flat];
      for (std::size_t pred : neighbors(grid, top.flat)) {
        if (pred != s_goal_flat_) {
          const double c = edge_cost(grid, pred, top.flat);
          if (c != kInf) {
            rhs_[pred] = std::min(rhs_[pred], c + g_[top.flat]);
          }
        }
        update_vertex(grid, pred);
      }
    } else {
      const double g_old = g_[top.flat];
      g_[top.flat] = kInf;
      auto preds_and_self = neighbors(grid, top.flat);
      preds_and_self.push_back(top.flat);
      for (std::size_t s : preds_and_self) {
        if (s != s_goal_flat_) {
          const double c = edge_cost(grid, s, top.flat);
          if (rhs_[s] == c + g_old && c != kInf) {
            double best = kInf;
            for (std::size_t succ : neighbors(grid, s)) {
              const double cs = edge_cost(grid, s, succ);
              if (cs == kInf) {
                continue;
              }
              best = std::min(best, cs + g_[succ]);
            }
            rhs_[s] = best;
          }
        }
        update_vertex(grid, s);
      }
    }

    ++stats.expansions;
    ++since_feedback;

    const double since_fb_sec =
      std::chrono::duration<double>(now_wall - last_feedback).count();
    if (feedback &&
        (since_feedback >= feedback_every_nodes ||
         since_fb_sec >= feedback_every_seconds))
    {
      emit_feedback(stats.expansions);
      if (params.viz_delay_ms > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(params.viz_delay_ms));
      }
      since_feedback = 0;
      last_feedback = now_wall;
    }
  }

  stats.reached_start = (g_[s_start_flat_] != kInf);
  return stats;
}

std::vector<pathfinder_core::VoxelIndex> DstarLiteCore::extract_path(
  const pathfinder_core::InflatedVoxelGrid & grid,
  std::size_t max_steps) const
{
  std::vector<pathfinder_core::VoxelIndex> out;
  if (g_.empty() || g_[s_start_flat_] == kInf) {
    return out;
  }
  std::size_t cur = s_start_flat_;
  out.push_back(grid.from_linear_index(cur));
  for (std::size_t step = 0; step < max_steps; ++step) {
    if (cur == s_goal_flat_) {
      return out;
    }
    double best_cost = kInf;
    std::size_t best_next = cur;
    for (std::size_t succ : neighbors(grid, cur)) {
      if (g_[succ] == kInf) {
        continue;
      }
      const double c = edge_cost(grid, cur, succ);
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
    out.push_back(grid.from_linear_index(cur));
  }
  out.clear();
  return out;
}

std::vector<std::size_t> DstarLiteCore::sample_open_flats(std::size_t max_count) const
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

std::vector<std::size_t> DstarLiteCore::sample_locked_flats(std::size_t max_count) const
{
  std::vector<std::size_t> out;
  if (max_count == 0 || g_.empty()) {
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

DstarLiteResult DstarLiteCore::plan(
  const pathfinder_core::InflatedVoxelGrid & grid,
  const pathfinder_core::VoxelIndex & start,
  const pathfinder_core::VoxelIndex & goal,
  const DstarLiteParams & params,
  FeedbackCallback feedback)
{
  DstarLiteResult result;
  const auto start_wall = std::chrono::steady_clock::now();

  if (grid.cell_count() == 0) {
    result.success = false;
    result.message = "voxel grid is empty";
    return result;
  }
  if (!grid.in_bounds(start) || grid.is_occupied(start)) {
    result.success = false;
    result.message = "start voxel is occupied or out of bounds";
    return result;
  }
  if (!grid.in_bounds(goal) || grid.is_occupied(goal)) {
    result.success = false;
    result.message = "goal voxel is occupied or out of bounds";
    return result;
  }

  const std::size_t start_flat = grid.linear_index(start);
  const std::size_t goal_flat = grid.linear_index(goal);

  const bool grid_changed =
    !initialized_ || &grid != last_grid_ || grid.cell_count() != last_grid_cells_;
  const bool goal_changed =
    !last_goal_flat_.has_value() || *last_goal_flat_ != goal_flat;
  const bool start_changed =
    !last_start_flat_.has_value() || *last_start_flat_ != start_flat;

  if (grid_changed || goal_changed) {
    initialize_search(grid, start_flat, goal_flat);
    RCLCPP_INFO(
      logger_,
      "D* Lite: %s (start=%zu goal=%zu cells=%zu)",
      grid_changed ? "full rebuild (grid changed or first call)"
                   : "goal changed; reinitializing",
      start_flat, goal_flat, grid.cell_count());
  } else if (start_changed) {
    shift_start(grid, start_flat);
    RCLCPP_INFO(
      logger_,
      "D* Lite: incremental replan, start moved (k_m=%.3f)",
      k_m_);
  } else {
    RCLCPP_INFO(logger_, "D* Lite: nothing changed, replaying solution");
  }

  last_grid_ = &grid;
  last_grid_cells_ = grid.cell_count();
  last_start_flat_ = start_flat;
  last_goal_flat_ = goal_flat;

  const auto stats = compute_shortest_path(grid, params, feedback);

  const auto end_wall = std::chrono::steady_clock::now();
  result.plan_duration_ms =
    std::chrono::duration<double, std::milli>(end_wall - start_wall).count();
  result.nodes_expanded = stats.expansions;

  if (stats.timed_out) {
    result.success = false;
    result.message = "time budget exceeded";
    RCLCPP_WARN(
      logger_,
      "D* Lite aborted after %.2f ms; %d expansions",
      result.plan_duration_ms, stats.expansions);
    return result;
  }
  if (!stats.reached_start) {
    result.success = false;
    result.message = "no path found";
    RCLCPP_WARN(
      logger_,
      "D* Lite failed: open exhausted (%d expansions, %.2f ms)",
      stats.expansions, result.plan_duration_ms);
    return result;
  }

  const auto path_voxels =
    extract_path(grid, static_cast<std::size_t>(grid.cell_count()));
  if (path_voxels.empty()) {
    result.success = false;
    result.message = "no path found";
    return result;
  }

  result.path = path_voxels;
  result.success = true;
  result.message = "ok";
  RCLCPP_INFO(
    logger_,
    "D* Lite ok: %zu voxels, %d expansions, %.2f ms",
    path_voxels.size(), stats.expansions, result.plan_duration_ms);
  return result;
}

}  // namespace pathfinder_algo_dstar_lite
