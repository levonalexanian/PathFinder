#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <pathfinder_core/voxel_grid.hpp>

namespace octomap
{
class OcTree;
}

namespace pathfinder_algo_dijkstra
{

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
  DijkstraGrid(const octomap::OcTree & tree, double robot_radius);

  double resolution() const { return grid_.resolution(); }
  const pathfinder_core::InflatedVoxelGrid & grid() const { return grid_; }

  PlanOutcome plan(
    const PlanRequest & req,
    std::function<void(const PlanProgress &)> on_progress,
    int feedback_node_stride,
    double feedback_time_stride_sec) const;

private:
  pathfinder_core::InflatedVoxelGrid grid_;
};

}  // namespace pathfinder_algo_dijkstra
