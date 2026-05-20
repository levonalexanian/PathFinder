#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <random>
#include <vector>

#include <pathfinder_core/voxel_grid.hpp>

namespace pathfinder_rrt
{

struct Point3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

inline double distance(const Point3 & a, const Point3 & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

struct TreeNode
{
  Point3 position;
  int parent_idx{-1};
  double cost{0.0};
};

struct RRTStarParams
{
  double step_size{0.3};
  double goal_bias{0.05};
  double rewire_radius{1.0};
  double goal_tolerance{0.3};
  int max_iterations{5000};
  double max_plan_time_sec{5.0};
  double robot_radius{0.25};
  int min_iterations_after_goal{1000};
  unsigned int random_seed{0};
};

class RRTStar
{
public:
  RRTStar(const pathfinder_core::InflatedVoxelGrid & grid, const RRTStarParams & params)
  : grid_(grid), params_(params), rng_(params.random_seed)
  {
    const auto mn = grid_.min_bound();
    const auto mx = grid_.max_bound();
    min_bound_ = {mn[0], mn[1], mn[2]};
    max_bound_ = {mx[0], mx[1], mx[2]};
  }

  void add_node(const Point3 & p, int parent_idx, double cost)
  {
    tree_nodes_.push_back({p, parent_idx, cost});
  }

  const std::vector<TreeNode> & nodes() const { return tree_nodes_; }
  std::size_t size() const { return tree_nodes_.size(); }

  Point3 sample(const Point3 & goal)
  {
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    if (unit(rng_) < params_.goal_bias) {
      return goal;
    }
    std::uniform_real_distribution<double> dx(min_bound_.x, max_bound_.x);
    std::uniform_real_distribution<double> dy(min_bound_.y, max_bound_.y);
    std::uniform_real_distribution<double> dz(min_bound_.z, max_bound_.z);
    return {dx(rng_), dy(rng_), dz(rng_)};
  }

  int nearest(const Point3 & q) const
  {
    int best = -1;
    double best_d = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < tree_nodes_.size(); ++i) {
      const double d = distance(tree_nodes_[i].position, q);
      if (d < best_d) {
        best_d = d;
        best = static_cast<int>(i);
      }
    }
    return best;
  }

  Point3 steer(const Point3 & from, const Point3 & to) const
  {
    const double d = distance(from, to);
    if (d <= params_.step_size || d < 1e-9) {
      return to;
    }
    const double s = params_.step_size / d;
    return {from.x + (to.x - from.x) * s,
            from.y + (to.y - from.y) * s,
            from.z + (to.z - from.z) * s};
  }

  std::vector<int> neighbors_within(const Point3 & q, double radius) const
  {
    std::vector<int> out;
    out.reserve(32);
    const double r2 = radius * radius;
    for (std::size_t i = 0; i < tree_nodes_.size(); ++i) {
      const double dx = tree_nodes_[i].position.x - q.x;
      const double dy = tree_nodes_[i].position.y - q.y;
      const double dz = tree_nodes_[i].position.z - q.z;
      if (dx * dx + dy * dy + dz * dz <= r2) {
        out.push_back(static_cast<int>(i));
      }
    }
    return out;
  }

  bool point_in_bounds(const Point3 & p) const
  {
    return p.x >= min_bound_.x && p.x <= max_bound_.x &&
           p.y >= min_bound_.y && p.y <= max_bound_.y &&
           p.z >= min_bound_.z && p.z <= max_bound_.z;
  }

  bool point_collision(const Point3 & p) const
  {
    return grid_.is_occupied_at_point(p.x, p.y, p.z);
  }

  bool segment_collision_free(const Point3 & a, const Point3 & b) const
  {
    if (!point_in_bounds(a) || !point_in_bounds(b)) {
      return false;
    }
    return grid_.is_segment_free(a.x, a.y, a.z, b.x, b.y, b.z);
  }

  int choose_parent(const std::vector<int> & near, const Point3 & x_new, int fallback_parent) const
  {
    int best = fallback_parent;
    double best_cost = (fallback_parent >= 0)
      ? tree_nodes_[fallback_parent].cost + distance(tree_nodes_[fallback_parent].position, x_new)
      : std::numeric_limits<double>::infinity();
    for (int idx : near) {
      if (idx == fallback_parent) {
        continue;
      }
      const double c = tree_nodes_[idx].cost + distance(tree_nodes_[idx].position, x_new);
      if (c < best_cost && segment_collision_free(tree_nodes_[idx].position, x_new)) {
        best_cost = c;
        best = idx;
      }
    }
    return best;
  }

  void rewire(const std::vector<int> & near, int new_idx)
  {
    const auto & x_new = tree_nodes_[new_idx];
    for (int idx : near) {
      if (idx == x_new.parent_idx) {
        continue;
      }
      auto & cand = tree_nodes_[idx];
      const double new_cost = x_new.cost + distance(x_new.position, cand.position);
      if (new_cost + 1e-9 < cand.cost &&
          segment_collision_free(x_new.position, cand.position))
      {
        cand.parent_idx = new_idx;
        const double delta = new_cost - cand.cost;
        cand.cost = new_cost;
        propagate_cost_decrease(idx, delta);
      }
    }
  }

  std::vector<Point3> reconstruct_from(int idx) const
  {
    std::vector<Point3> out;
    while (idx >= 0) {
      out.push_back(tree_nodes_[idx].position);
      idx = tree_nodes_[idx].parent_idx;
    }
    std::reverse(out.begin(), out.end());
    return out;
  }

  const Point3 & min_bound() const { return min_bound_; }
  const Point3 & max_bound() const { return max_bound_; }
  double resolution() const { return grid_.resolution(); }

private:
  void propagate_cost_decrease(int root_idx, double delta)
  {
    for (std::size_t i = 0; i < tree_nodes_.size(); ++i) {
      if (static_cast<int>(i) == root_idx) {
        continue;
      }
      int p = tree_nodes_[i].parent_idx;
      while (p >= 0) {
        if (p == root_idx) {
          tree_nodes_[i].cost -= delta;
          break;
        }
        p = tree_nodes_[p].parent_idx;
      }
    }
  }

  const pathfinder_core::InflatedVoxelGrid & grid_;
  RRTStarParams params_;
  std::mt19937 rng_;

  Point3 min_bound_;
  Point3 max_bound_;

  std::vector<TreeNode> tree_nodes_;
};

}  // namespace pathfinder_rrt
