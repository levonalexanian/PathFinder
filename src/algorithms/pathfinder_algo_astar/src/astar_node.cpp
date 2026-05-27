#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <octomap_msgs/msg/octomap.hpp>

#include <pathfinder_core/octomap_utils.hpp>
#include <pathfinder_core/planner_action_server_base.hpp>
#include <pathfinder_core/planner_params.hpp>
#include <pathfinder_core/planner_utils.hpp>
#include <pathfinder_core/search_viz.hpp>
#include <pathfinder_core/voxel_grid.hpp>
#include "pathfinder_algo_astar/astar_core.hpp"

namespace pathfinder_algo_astar
{

namespace
{

constexpr char kActionName[] = "/astar/request_path";
constexpr char kMapFrame[] = "map";

using pathfinder_core::InflatedVoxelGrid;
using pathfinder_core::VoxelIndex;
namespace viz = pathfinder_core::viz;

}  // namespace

class AstarServer : public pathfinder_core::PlannerActionServerBase
{
public:
  explicit AstarServer(rclcpp::Node * node)
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
    pathfinder_core::BasePlannerParams base_params;
    pathfinder_core::load_base_planner_params(node(), base_params);

    AstarParams params;
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

    InflatedVoxelGrid grid(*tree, params.robot_radius);
    if (grid.cell_count() == 0) {
      result.success = false;
      result.message = "voxel grid is empty";
      return;
    }

    const double res = grid.resolution();
    const auto dims = grid.dims();
    RCLCPP_INFO(
      node()->get_logger(),
      "A* search on %dx%dx%d grid (res=%.3f m, radius=%.2f m)",
      dims[0], dims[1], dims[2], res, params.robot_radius);

    const auto [start_v, goal_v] = pathfinder_core::voxels_from_goal(grid, goal);
    const auto nudged = pathfinder_core::nudge_start_and_goal(grid, start_v, goal_v);
    if (!nudged) {
      result.success = false;
      result.message = "start or goal voxel is occupied and no nearby free voxel found";
      return;
    }
    const auto [start_idx, goal_idx] = *nudged;

    auto feedback_cb = [&](const AstarFeedback & afb) {
      Feedback fb;
      fb.nodes_expanded = afb.nodes_expanded;

      const auto stamp = node()->now();
      const double cube_size = res * viz::kVoxelCubeFraction;

      auto open_marker = viz::make_cube_list_marker(
        stamp, "OPEN_SET", 0, cube_size,
        viz::kOpenSetR, viz::kOpenSetG, viz::kOpenSetB, viz::kOpenSetA);
      open_marker.points.reserve(afb.sampled_open_flat.size());
      for (auto f : afb.sampled_open_flat) {
        open_marker.points.push_back(viz::flat_to_point(grid, f));
      }

      auto closed_marker = viz::make_cube_list_marker(
        stamp, "CLOSED_SET", 1, cube_size,
        viz::kClosedSetR, viz::kClosedSetG, viz::kClosedSetB, viz::kClosedSetA);
      closed_marker.points.reserve(afb.sampled_closed_flat.size());
      for (auto f : afb.sampled_closed_flat) {
        closed_marker.points.push_back(viz::flat_to_point(grid, f));
      }

      auto path_marker = viz::make_line_strip_marker(
        stamp, "CURRENT_BEST_PATH", 2, res * viz::kVoxelLineFraction,
        viz::kBestPathR, viz::kBestPathG, viz::kBestPathB, viz::kBestPathA);
      if (!afb.best_partial_path.empty()) {
        path_marker.points.reserve(afb.best_partial_path.size());
        for (const auto & v : afb.best_partial_path) {
          path_marker.points.push_back(viz::voxel_to_point(grid, v));
        }

        std_msgs::msg::Header hdr;
        hdr.frame_id = kMapFrame;
        hdr.stamp = stamp;
        fb.current_best = viz::voxels_to_path(grid, afb.best_partial_path, hdr);
        fb.best_cost_so_far = afb.best_cost_so_far;
      }

      fb.search_state.markers.push_back(open_marker);
      fb.search_state.markers.push_back(closed_marker);
      fb.search_state.markers.push_back(path_marker);

      publish_feedback(fb);
    };

    const AstarResult ares = core_.plan(grid, start_idx, goal_idx, params, feedback_cb);

    result.plan_duration_ms = ares.plan_duration_ms;

    if (!ares.success) {
      result.success = false;
      result.message = ares.message;
      return;
    }

    std_msgs::msg::Header hdr;
    hdr.frame_id = kMapFrame;
    hdr.stamp = node()->now();
    nav_msgs::msg::Path path = viz::voxels_to_path(grid, ares.path, hdr);

    path.poses.insert(path.poses.begin(),
      viz::make_pose_stamped(
        goal.start.pose.position.x,
        goal.start.pose.position.y,
        goal.start.pose.position.z, hdr));
    path.poses.push_back(
      viz::make_pose_stamped(
        goal.goal.pose.position.x,
        goal.goal.pose.position.y,
        goal.goal.pose.position.z, hdr));

    result.path = path;
    result.success = true;
    result.message = "ok";
    RCLCPP_INFO(
      node()->get_logger(),
      "A* succeeded: %zu poses, %d nodes expanded, %.2f ms",
      path.poses.size(), ares.nodes_expanded, ares.plan_duration_ms);
  }

private:
  AstarCore core_;
};

}  // namespace pathfinder_algo_astar

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("astar_planner");
  pathfinder_algo_astar::AstarServer server(node.get());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
