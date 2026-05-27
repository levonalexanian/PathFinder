#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pathfinder_core/octomap_utils.hpp>
#include <pathfinder_core/planner_action_server_base.hpp>
#include <pathfinder_core/planner_params.hpp>
#include <pathfinder_core/planner_utils.hpp>
#include <pathfinder_core/search_viz.hpp>
#include <pathfinder_core/voxel_grid.hpp>

#include "pathfinder_algo_dijkstra/dijkstra_core.hpp"

namespace pathfinder_algo_dijkstra
{

namespace
{

constexpr char kActionName[] = "/dijkstra/request_path";
constexpr char kMapFrame[] = "map";

// Dijkstra-specific: fade closed-set cubes by distance from the search origin.
// Each point's grey shade tracks how far it is from start (darker = farther).
visualization_msgs::msg::Marker make_faded_cube_marker(
  const pathfinder_core::InflatedVoxelGrid & grid,
  const std::vector<std::size_t> & flat_indices,
  const rclcpp::Time & stamp,
  const std::string & ns,
  int id,
  double scale,
  float base_a,
  const pathfinder_core::VoxelIndex & origin)
{
  auto m = pathfinder_core::viz::make_cube_list_marker(
    stamp, ns, id, scale,
    pathfinder_core::viz::kClosedSetR,
    pathfinder_core::viz::kClosedSetG,
    pathfinder_core::viz::kClosedSetB,
    base_a,
    kMapFrame);

  m.points.reserve(flat_indices.size());
  m.colors.reserve(flat_indices.size());

  double max_dist = 1.0;
  for (auto lin : flat_indices) {
    const auto v = grid.from_linear_index(lin);
    const double d = pathfinder_core::voxel_distance(v, origin, grid.resolution());
    if (d > max_dist) {
      max_dist = d;
    }
  }

  for (auto lin : flat_indices) {
    const auto v = grid.from_linear_index(lin);
    m.points.push_back(pathfinder_core::viz::flat_to_point(grid, lin));
    const double d = pathfinder_core::voxel_distance(v, origin, grid.resolution());
    const double t = std::min(1.0, d / max_dist);
    const float shade = static_cast<float>(0.75 - 0.55 * t);
    std_msgs::msg::ColorRGBA c;
    c.r = shade;
    c.g = shade;
    c.b = shade;
    c.a = base_a;
    m.colors.push_back(c);
  }
  return m;
}

}  // namespace

class DijkstraServer : public pathfinder_core::PlannerActionServerBase
{
public:
  explicit DijkstraServer(rclcpp::Node * node)
  : pathfinder_core::PlannerActionServerBase(node, kActionName)
  {
    pathfinder_core::declare_base_planner_params(node);
    node->declare_parameter<int>("feedback_every_nodes", 200);
    node->declare_parameter<double>("feedback_every_seconds", 0.2);
    node->declare_parameter<bool>("publish_current_best", true);
  }

protected:
  void compute_path(
    const Goal & goal,
    const octomap_msgs::msg::Octomap & latest_map,
    std::function<void(const Feedback &)> publish_feedback,
    Result & result) override
  {
    pathfinder_core::BasePlannerParams base;
    pathfinder_core::load_base_planner_params(node(), base);

    DijkstraParams params;
    params.robot_radius = base.robot_radius;
    params.max_plan_time_sec = base.max_plan_time_sec;
    params.viz_delay_ms = base.viz_delay_ms;
    params.feedback_every_nodes =
      static_cast<int>(node()->get_parameter("feedback_every_nodes").as_int());
    params.feedback_every_seconds =
      node()->get_parameter("feedback_every_seconds").as_double();
    params.publish_current_best =
      node()->get_parameter("publish_current_best").as_bool();

    auto tree = pathfinder_core::octree_from_msg(latest_map);
    if (!tree) {
      result.success = false;
      result.message = "failed to deserialize octomap";
      return;
    }

    pathfinder_core::InflatedVoxelGrid grid(*tree, params.robot_radius);

    const auto [start_v, goal_v] = pathfinder_core::voxels_from_goal(grid, goal);

    const pathfinder_core::VoxelIndex search_origin = start_v;

    auto on_feedback = [&](const DijkstraFeedback & cb) {
      Feedback fb;
      fb.nodes_expanded = cb.nodes_expanded;
      fb.best_cost_so_far = cb.best_cost_so_far;

      const auto stamp = node()->now();
      const double cube_scale = grid.resolution() * 0.9;

      auto closed_marker = make_faded_cube_marker(
        grid, cb.sampled_closed_flat, stamp,
        "EXPLORED_SET", 0, cube_scale,
        pathfinder_core::viz::kClosedSetA,
        search_origin);

      auto open_marker = pathfinder_core::viz::make_cube_list_marker(
        stamp, "CURRENT_FRONTIER", 1, cube_scale,
        pathfinder_core::viz::kOpenSetR,
        pathfinder_core::viz::kOpenSetG,
        pathfinder_core::viz::kOpenSetB,
        pathfinder_core::viz::kOpenSetA,
        kMapFrame);
      open_marker.points.reserve(cb.sampled_open_flat.size());
      for (auto lin : cb.sampled_open_flat) {
        open_marker.points.push_back(pathfinder_core::viz::flat_to_point(grid, lin));
      }

      visualization_msgs::msg::MarkerArray markers;
      markers.markers.push_back(closed_marker);
      markers.markers.push_back(open_marker);

      if (params.publish_current_best && !cb.best_partial_path.empty()) {
        std_msgs::msg::Header hdr;
        hdr.frame_id = kMapFrame;
        hdr.stamp = stamp;
        fb.current_best = pathfinder_core::viz::voxels_to_path(
          grid, cb.best_partial_path, hdr);

        if (cb.best_partial_path.size() >= 2) {
          auto line = pathfinder_core::viz::make_line_strip_marker(
            stamp, "CURRENT_BEST_PATH", 2,
            grid.resolution() * pathfinder_core::viz::kVoxelLineFraction,
            pathfinder_core::viz::kBestPathR,
            pathfinder_core::viz::kBestPathG,
            pathfinder_core::viz::kBestPathB,
            pathfinder_core::viz::kBestPathA,
            kMapFrame);
          line.points.reserve(cb.best_partial_path.size());
          for (const auto & v : cb.best_partial_path) {
            line.points.push_back(pathfinder_core::viz::voxel_to_point(grid, v));
          }
          markers.markers.push_back(line);
        }
      } else {
        fb.current_best.header.frame_id = kMapFrame;
        fb.current_best.header.stamp = stamp;
      }

      fb.search_state = markers;
      publish_feedback(fb);
    };

    const auto outcome = core_.plan(grid, start_v, goal_v, params, on_feedback);

    std_msgs::msg::Header hdr;
    hdr.frame_id = kMapFrame;
    hdr.stamp = node()->now();
    result.path = pathfinder_core::viz::voxels_to_path(grid, outcome.path, hdr);
    result.success = outcome.success;
    result.message = outcome.message;
    result.plan_duration_ms = outcome.plan_duration_ms;

    RCLCPP_INFO(
      node()->get_logger(),
      "dijkstra plan: success=%d nodes=%d duration_ms=%.2f msg=%s",
      static_cast<int>(outcome.success),
      outcome.nodes_expanded,
      outcome.plan_duration_ms,
      outcome.message.c_str());
  }

private:
  DijkstraCore core_;
};

}  // namespace pathfinder_algo_dijkstra

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("dijkstra_planner");
  pathfinder_algo_dijkstra::DijkstraServer server(node.get());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
