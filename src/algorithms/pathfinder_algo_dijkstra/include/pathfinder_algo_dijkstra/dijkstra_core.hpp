#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

#include <rclcpp/logger.hpp>

#include <pathfinder_core/voxel_grid.hpp>

namespace pathfinder_algo_dijkstra
{

struct DijkstraParams
{
  double robot_radius{0.25};
  double max_plan_time_sec{5.0};
  int feedback_node_stride{200};
  double feedback_time_stride_sec{0.2};
  int viz_delay_ms{0};
  bool publish_current_best{true};
};

struct DijkstraFeedback
{
  int nodes_explored{0};
  std::vector<pathfinder_core::VoxelIndex> best_partial_path;
  double best_cost_so_far{0.0};
  std::vector<std::size_t> sampled_open_flat;
  std::vector<std::size_t> sampled_closed_flat;
};

struct DijkstraResult
{
  bool success{false};
  std::string message;
  std::vector<pathfinder_core::VoxelIndex> path;
  int nodes_expanded{0};
  double plan_duration_ms{0.0};
};

class DijkstraCore
{
public:
  using FeedbackCallback = std::function<void(const DijkstraFeedback &)>;

  explicit DijkstraCore(const rclcpp::Logger & logger);

  DijkstraResult plan(
    const pathfinder_core::InflatedVoxelGrid & grid,
    const pathfinder_core::VoxelIndex & start,
    const pathfinder_core::VoxelIndex & goal,
    const DijkstraParams & params,
    FeedbackCallback feedback);

private:
  rclcpp::Logger logger_;
};

}  // namespace pathfinder_algo_dijkstra
