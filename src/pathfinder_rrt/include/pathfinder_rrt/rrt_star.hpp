#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <random>
#include <vector>

#include <octomap/octomap.h>

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
  RRTStar(const octomap::OcTree & tree, const RRTStarParams & params)
  : tree_(tree), params_(params), rng_(params.random_seed)
  {
    double mx, my, mz, Mx, My, Mz;
    tree_.getMetricMin(mx, my, mz);
    tree_.getMetricMax(Mx, My, Mz);
    min_bound_ = {mx, my, mz};
    max_bound_ = {Mx, My, Mz};
    resolution_ = tree_.getResolution();
    collision_step_ = std::max(resolution_ * 0.5, 1e-3);

    inflation_voxels_ = static_cast<int>(std::ceil(params_.robot_radius / resolution_));
    if (inflation_voxels_ < 0) {
      inflation_voxels_ = 0;
    }
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
    if (inflation_voxels_ == 0) {
      auto * node = tree_.search(p.x, p.y, p.z);
      if (node && tree_.isNodeOccupied(node)) {
        return true;
      }
      return false;
    }
    const double r = params_.robot_radius;
    const double r2 = r * r;
    for (int ix = -inflation_voxels_; ix <= inflation_voxels_; ++ix) {
      for (int iy = -inflation_voxels_; iy <= inflation_voxels_; ++iy) {
        for (int iz = -inflation_voxels_; iz <= inflation_voxels_; ++iz) {
          const double ox = ix * resolution_;
          const double oy = iy * resolution_;
          const double oz = iz * resolution_;
          if (ox * ox + oy * oy + oz * oz > r2) {
            continue;
          }
          auto * node = tree_.search(p.x + ox, p.y + oy, p.z + oz);
          if (node && tree_.isNodeOccupied(node)) {
            return true;
          }
        }
      }
    }
    return false;
  }

  bool segment_collision_free(const Point3 & a, const Point3 & b) const
  {
    const double d = distance(a, b);
    if (d < 1e-9) {
      return !point_collision(a);
    }
    const int steps = std::max(1, static_cast<int>(std::ceil(d / collision_step_)));
    for (int i = 0; i <= steps; ++i) {
      const double t = static_cast<double>(i) / steps;
      Point3 p{
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t};
      if (!point_in_bounds(p)) {
        return false;
      }
      if (point_collision(p)) {
        return false;
      }
    }
    return true;
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
  double resolution() const { return resolution_; }

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

  const octomap::OcTree & tree_;
  RRTStarParams params_;
  std::mt19937 rng_;

  Point3 min_bound_;
  Point3 max_bound_;
  double resolution_{0.1};
  double collision_step_{0.05};
  int inflation_voxels_{0};

  std::vector<TreeNode> tree_nodes_;
};

}  // namespace pathfinder_rrt
