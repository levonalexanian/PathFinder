#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <rclcpp/logger.hpp>

#include <pathfinder_core/voxel_grid.hpp>

namespace pathfinder_algo_rrt
{

struct RRTParams
{
  double robot_radius{0.25};
  double max_plan_time_sec{5.0};
  double step_size{0.3};
  double goal_bias{0.05};
  double rewire_radius{1.0};
  double goal_tolerance{0.3};
  int max_iterations{5000};
  int min_iterations_after_goal{1000};
  int viz_delay_ms{0};
  uint32_t random_seed{0};  // 0 means use a time-based seed
  std::string output_frame{"map"};
};

// Primitive world-frame point used for viz hints. No ROS types here.
struct WorldPoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct RRTFeedback
{
  int nodes_explored{0};
  std::vector<pathfinder_core::VoxelIndex> best_partial_path;
  double best_cost_so_far{0.0};

  // RRT-specific viz hints as PRIMITIVE data. The Node turns these into
  // visualization_msgs::Marker(Array). The continuous best path is exposed
  // separately so the Node can publish a high-fidelity nav_msgs::Path
  // without round-tripping through the voxel grid.
  std::vector<WorldPoint> best_partial_path_world;
  // Newly-discovered tree edges since the last feedback (deltas).
  std::vector<std::pair<WorldPoint, WorldPoint>> tree_edges;
  // Stable per-edge identity (child node index in the RRT tree); same length
  // as tree_edges. Used by the Node to give each emitted edge a unique
  // marker ID so each can fade independently.
  std::vector<std::size_t> tree_edge_ids;
};

struct RRTResult
{
  bool success{false};
  std::string message;
  std::vector<pathfinder_core::VoxelIndex> path;  // empty if !success
  std::vector<WorldPoint> path_world;             // continuous-space path, empty if !success
  int nodes_expanded{0};
  double plan_duration_ms{0.0};
};

class RRTCore
{
public:
  using FeedbackCallback = std::function<void(const RRTFeedback &)>;

  explicit RRTCore(const rclcpp::Logger & logger);

  RRTResult plan(
    const pathfinder_core::InflatedVoxelGrid & grid,
    const pathfinder_core::VoxelIndex & start,
    const pathfinder_core::VoxelIndex & goal,
    const RRTParams & params,
    FeedbackCallback feedback);

private:
  rclcpp::Logger logger_;
  // For viz: tree-node indices already pushed to the Node as edges. Reset at
  // the start of every plan() so each plan emits a fresh wavefront.
  std::unordered_set<std::size_t> emitted_edges_;
  // RRT has no closed-set analog, so this exists only for API parity with
  // grid planners. Unused today.
  std::unordered_set<std::size_t> emitted_open_;
  std::unordered_set<std::size_t> emitted_closed_;
};

}  // namespace pathfinder_algo_rrt
