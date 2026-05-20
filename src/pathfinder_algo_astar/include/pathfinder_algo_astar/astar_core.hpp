#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

#include <rclcpp/logger.hpp>

#include <pathfinder_core/voxel_grid.hpp>

namespace pathfinder_algo_astar
{

struct AstarParams
{
  double robot_radius{0.25};
  double max_plan_time_sec{5.0};
  int feedback_every_nodes{200};
  double feedback_every_seconds{0.2};
};

struct AstarFeedback
{
  int nodes_explored{0};
  std::vector<pathfinder_core::VoxelIndex> best_partial_path;
  double best_cost_so_far{0.0};
  std::vector<std::size_t> sampled_open_flat;
  std::vector<std::size_t> sampled_closed_flat;
};

struct AstarResult
{
  bool success{false};
  std::string message;
  std::vector<pathfinder_core::VoxelIndex> path;
  int nodes_expanded{0};
  double plan_duration_ms{0.0};
};

class AstarCore
{
public:
  using FeedbackCallback = std::function<void(const AstarFeedback &)>;

  explicit AstarCore(const rclcpp::Logger & logger);

  AstarResult plan(
    const pathfinder_core::InflatedVoxelGrid & grid,
    const pathfinder_core::VoxelIndex & start,
    const pathfinder_core::VoxelIndex & goal,
    const AstarParams & params,
    FeedbackCallback feedback);

private:
  rclcpp::Logger logger_;
};

}  // namespace pathfinder_algo_astar
