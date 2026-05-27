#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <octomap_msgs/msg/octomap.hpp>

#include <pathfinder_core/octomap_utils.hpp>
#include <pathfinder_core/planner_action_server_base.hpp>
#include <pathfinder_core/planner_params.hpp>
#include <pathfinder_core/planner_utils.hpp>
#include <pathfinder_core/search_viz.hpp>
#include <pathfinder_core/voxel_grid.hpp>

#include "pathfinder_algo_dstar_lite/dstar_lite_core.hpp"

namespace pathfinder_algo_dstar_lite
{

namespace
{

constexpr char kActionName[] = "/dstar_lite/request_path";
constexpr char kMapFrame[] = "map";

using pathfinder_core::InflatedVoxelGrid;
using pathfinder_core::VoxelIndex;

// Cheap map signature: combines octomap size with a sampled-bit hash. Two
// snapshots of the same OcTree produce the same signature; a genuine update
// flips bits and changes the hash.
std::uint64_t compute_map_signature(const octomap::OcTree & tree)
{
  std::uint64_t h = tree.size();
  std::uint64_t mix = 0xcbf29ce484222325ULL;
  std::size_t sampled = 0;
  for (auto it = tree.begin_leafs(), end = tree.end_leafs();
       it != end && sampled < 4096; ++it, ++sampled)
  {
    const auto c = it.getCoordinate();
    const std::uint64_t bx = static_cast<std::uint64_t>(
      static_cast<std::int64_t>(std::floor(c.x() * 100.0)));
    const std::uint64_t by = static_cast<std::uint64_t>(
      static_cast<std::int64_t>(std::floor(c.y() * 100.0)));
    const std::uint64_t bz = static_cast<std::uint64_t>(
      static_cast<std::int64_t>(std::floor(c.z() * 100.0)));
    const std::uint64_t occ = tree.isNodeOccupied(*it) ? 1ULL : 0ULL;
    mix ^= bx + 0x9e3779b97f4a7c15ULL + (mix << 6) + (mix >> 2);
    mix ^= by + 0x9e3779b97f4a7c15ULL + (mix << 6) + (mix >> 2);
    mix ^= bz + 0x9e3779b97f4a7c15ULL + (mix << 6) + (mix >> 2);
    mix ^= occ + 0x9e3779b97f4a7c15ULL + (mix << 6) + (mix >> 2);
  }
  h ^= mix;
  return h;
}

}  // namespace

class DstarLiteServer : public pathfinder_core::PlannerActionServerBase
{
public:
  explicit DstarLiteServer(rclcpp::Node * node)
  : PlannerActionServerBase(node, kActionName)
  {
    pathfinder_core::declare_base_planner_params(node);
    node->declare_parameter<int>("feedback_every_nodes", 200);
    node->declare_parameter<double>("feedback_every_seconds", 0.2);
  }

protected:
  void compute_path(
    const Goal & goal,
    const octomap_msgs::msg::Octomap & latest_map,
    std::function<void(const Feedback &)> publish_feedback,
    Result & result) override
  {
    std::lock_guard<std::mutex> lk(state_mutex_);

    pathfinder_core::BasePlannerParams base_params;
    pathfinder_core::load_base_planner_params(node(), base_params);

    DstarLiteParams params;
    params.robot_radius = base_params.robot_radius;
    params.max_plan_time_sec = base_params.max_plan_time_sec;
    params.viz_delay_ms = base_params.viz_delay_ms;
    params.feedback_every_nodes = static_cast<int>(std::max<int64_t>(
      1, node()->get_parameter("feedback_every_nodes").as_int()));
    params.feedback_every_seconds =
      node()->get_parameter("feedback_every_seconds").as_double();

    auto tree = pathfinder_core::octree_from_msg(latest_map);
    if (!tree) {
      result.success = false;
      result.message = "failed to deserialize octomap";
      return;
    }
    const std::uint64_t signature = compute_map_signature(*tree);

    // Only rebuild the inflated grid when the octomap actually changed.
    // Keeping the same grid pointer between calls is what lets DstarLiteCore
    // recognize the grid as unchanged and perform an incremental replan.
    if (!grid_ || signature != map_signature_ ||
        params.robot_radius != last_robot_radius_)
    {
      grid_ = std::make_unique<InflatedVoxelGrid>(*tree, params.robot_radius);
      map_signature_ = signature;
      last_robot_radius_ = params.robot_radius;
      core_.reset();
    }

    if (grid_->cell_count() == 0) {
      result.success = false;
      result.message = "voxel grid is empty";
      return;
    }

    const auto [start_v_raw, goal_v_raw] =
      pathfinder_core::voxels_from_goal(*grid_, goal);
    const auto nudged = pathfinder_core::nudge_start_and_goal(*grid_, start_v_raw, goal_v_raw);
    if (!nudged) {
      result.success = false;
      result.message = "start or goal voxel is occupied and no nearby free voxel found";
      return;
    }
    const auto [start_v, goal_v] = *nudged;

    const InflatedVoxelGrid & grid_ref = *grid_;
    const double res = grid_ref.resolution();

    auto feedback_cb = [&](const DstarLiteFeedback & fb) {
      Feedback ros_fb;
      ros_fb.nodes_expanded = fb.nodes_expanded;

      const auto stamp = node()->now();
      const double cube_size = res * pathfinder_core::viz::kVoxelCubeFraction;

      auto open_marker = pathfinder_core::viz::make_cube_list_marker(
        stamp, "OPEN_SET", 0, cube_size,
        pathfinder_core::viz::kOpenSetR, pathfinder_core::viz::kOpenSetG,
        pathfinder_core::viz::kOpenSetB, pathfinder_core::viz::kOpenSetA);
      open_marker.points.reserve(fb.sampled_open_flat.size());
      for (auto f : fb.sampled_open_flat) {
        open_marker.points.push_back(pathfinder_core::viz::flat_to_point(grid_ref, f));
      }

      auto locked_marker = pathfinder_core::viz::make_cube_list_marker(
        stamp, "LOCKED_DOWN", 1, cube_size,
        pathfinder_core::viz::kClosedSetR, pathfinder_core::viz::kClosedSetG,
        pathfinder_core::viz::kClosedSetB, pathfinder_core::viz::kClosedSetA);
      locked_marker.points.reserve(fb.sampled_closed_flat.size());
      for (auto f : fb.sampled_closed_flat) {
        locked_marker.points.push_back(pathfinder_core::viz::flat_to_point(grid_ref, f));
      }

      auto path_marker = pathfinder_core::viz::make_line_strip_marker(
        stamp, "CURRENT_BEST_PATH", 2, res * pathfinder_core::viz::kVoxelLineFraction,
        pathfinder_core::viz::kBestPathR, pathfinder_core::viz::kBestPathG,
        pathfinder_core::viz::kBestPathB, pathfinder_core::viz::kBestPathA);
      path_marker.points.reserve(fb.best_partial_path.size());
      for (const auto & v : fb.best_partial_path) {
        path_marker.points.push_back(pathfinder_core::viz::voxel_to_point(grid_ref, v));
      }

      std_msgs::msg::Header hdr;
      hdr.frame_id = kMapFrame;
      hdr.stamp = stamp;
      ros_fb.current_best = pathfinder_core::viz::voxels_to_path(
        grid_ref, fb.best_partial_path, hdr);
      ros_fb.best_cost_so_far = fb.best_cost_so_far;

      ros_fb.search_state.markers.push_back(open_marker);
      ros_fb.search_state.markers.push_back(locked_marker);
      ros_fb.search_state.markers.push_back(path_marker);

      publish_feedback(ros_fb);
    };

    const auto core_result = core_.plan(grid_ref, start_v, goal_v, params, feedback_cb);

    result.plan_duration_ms = core_result.plan_duration_ms;
    if (!core_result.success) {
      result.success = false;
      result.message = core_result.message;
      return;
    }

    nav_msgs::msg::Path path;
    path.header.frame_id = kMapFrame;
    path.header.stamp = node()->now();
    path.poses.reserve(core_result.path.size() + 2);

    {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose = goal.start.pose;
      path.poses.push_back(ps);
    }
    for (const auto & v : core_result.path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      const auto w = grid_ref.voxel_to_world(v);
      ps.pose.position.x = w[0];
      ps.pose.position.y = w[1];
      ps.pose.position.z = w[2];
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose = goal.goal.pose;
      path.poses.push_back(ps);
    }

    result.path = path;
    result.success = true;
    result.message = core_result.message;
  }

private:
  std::mutex state_mutex_;
  DstarLiteCore core_;
  std::unique_ptr<InflatedVoxelGrid> grid_;
  std::uint64_t map_signature_{0};
  double last_robot_radius_{-1.0};
};

}  // namespace pathfinder_algo_dstar_lite

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("dstar_lite_planner");
  pathfinder_algo_dstar_lite::DstarLiteServer server(node.get());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
