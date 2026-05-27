#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pathfinder_core/octomap_utils.hpp>
#include <pathfinder_core/planner_action_server_base.hpp>
#include <pathfinder_core/planner_params.hpp>
#include <pathfinder_core/planner_utils.hpp>
#include <pathfinder_core/search_viz.hpp>
#include <pathfinder_core/voxel_grid.hpp>
#include <pathfinder_algo_rrt/rrt_core.hpp>

namespace pathfinder_algo_rrt
{

namespace
{

nav_msgs::msg::Path world_points_to_path(
  const std::vector<WorldPoint> & pts,
  const std_msgs::msg::Header & header)
{
  nav_msgs::msg::Path path;
  path.header = header;
  path.poses.reserve(pts.size());
  for (const auto & p : pts) {
    path.poses.push_back(
      pathfinder_core::viz::make_pose_stamped(p.x, p.y, p.z, header));
  }
  return path;
}

visualization_msgs::msg::MarkerArray build_markers(
  const RRTFeedback & fb,
  const std::string & frame,
  const rclcpp::Time & stamp)
{
  visualization_msgs::msg::MarkerArray arr;

  visualization_msgs::msg::Marker edges;
  pathfinder_core::viz::fill_marker_header(edges, stamp, 0, "rrt_tree_edges", frame);
  edges.type = visualization_msgs::msg::Marker::LINE_LIST;
  edges.scale.x = 0.02;
  edges.color.r = 0.0f;
  edges.color.g = 1.0f;
  edges.color.b = 1.0f;
  edges.color.a = 0.3f;
  edges.lifetime = pathfinder_core::viz::kSearchMarkerLifetime;
  edges.points.reserve(fb.tree_edges.size() * 2);
  for (const auto & e : fb.tree_edges) {
    geometry_msgs::msg::Point a, b;
    a.x = e.first.x;
    a.y = e.first.y;
    a.z = e.first.z;
    b.x = e.second.x;
    b.y = e.second.y;
    b.z = e.second.z;
    edges.points.push_back(a);
    edges.points.push_back(b);
  }
  arr.markers.push_back(edges);

  visualization_msgs::msg::Marker best_nodes;
  pathfinder_core::viz::fill_marker_header(best_nodes, stamp, 1, "rrt_best_path_nodes", frame);
  best_nodes.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  best_nodes.scale.x = 0.08;
  best_nodes.scale.y = 0.08;
  best_nodes.scale.z = 0.08;
  best_nodes.color.r = pathfinder_core::viz::kBestPathR;
  best_nodes.color.g = pathfinder_core::viz::kBestPathG;
  best_nodes.color.b = pathfinder_core::viz::kBestPathB;
  best_nodes.color.a = pathfinder_core::viz::kBestPathA;
  best_nodes.lifetime = pathfinder_core::viz::kSearchMarkerLifetime;
  best_nodes.points.reserve(fb.best_partial_path_world.size());
  for (const auto & p : fb.best_partial_path_world) {
    geometry_msgs::msg::Point gp;
    gp.x = p.x;
    gp.y = p.y;
    gp.z = p.z;
    best_nodes.points.push_back(gp);
  }
  arr.markers.push_back(best_nodes);

  return arr;
}

}  // namespace

class RRTServer : public pathfinder_core::PlannerActionServerBase
{
public:
  explicit RRTServer(rclcpp::Node * node)
  : pathfinder_core::PlannerActionServerBase(node, "/rrt/request_path")
  {
    pathfinder_core::declare_base_planner_params(node);
    node->declare_parameter<double>("step_size", 0.3);
    node->declare_parameter<double>("goal_bias", 0.05);
    node->declare_parameter<double>("rewire_radius", 1.0);
    node->declare_parameter<double>("goal_tolerance", 0.3);
    node->declare_parameter<int>("max_iterations", 5000);
    node->declare_parameter<int>("min_iterations_after_goal", 1000);
    node->declare_parameter<int>("random_seed", 0);
    node->declare_parameter<std::string>("output_frame", "map");
  }

protected:
  void compute_path(
    const Goal & goal,
    const octomap_msgs::msg::Octomap & latest_map,
    std::function<void(const Feedback &)> publish_feedback,
    Result & result) override
  {
    auto octree = pathfinder_core::octree_from_msg(latest_map);
    if (!octree) {
      result.success = false;
      result.message = "failed to decode octomap";
      return;
    }

    pathfinder_core::BasePlannerParams base_params;
    pathfinder_core::load_base_planner_params(node(), base_params);

    RRTParams params;
    params.robot_radius = base_params.robot_radius;
    params.max_plan_time_sec = base_params.max_plan_time_sec;
    params.viz_delay_ms = base_params.viz_delay_ms;
    params.step_size = node()->get_parameter("step_size").as_double();
    params.goal_bias = node()->get_parameter("goal_bias").as_double();
    params.rewire_radius = node()->get_parameter("rewire_radius").as_double();
    params.goal_tolerance = node()->get_parameter("goal_tolerance").as_double();
    params.max_iterations = node()->get_parameter("max_iterations").as_int();
    params.min_iterations_after_goal =
      node()->get_parameter("min_iterations_after_goal").as_int();
    const int seed_param = node()->get_parameter("random_seed").as_int();
    params.random_seed = (seed_param > 0) ? static_cast<uint32_t>(seed_param) : 0u;
    const std::string frame = node()->get_parameter("output_frame").as_string();
    params.output_frame = frame;

    pathfinder_core::InflatedVoxelGrid grid(*octree, params.robot_radius);

    const auto [start_v, goal_v] = pathfinder_core::voxels_from_goal(grid, goal);

    auto feedback_cb = [this, &publish_feedback, &frame](const RRTFeedback & fb) {
      Feedback ros_fb;
      ros_fb.nodes_expanded = static_cast<int32_t>(fb.nodes_expanded);
      ros_fb.best_cost_so_far = fb.best_cost_so_far;
      const auto stamp = node()->now();
      std_msgs::msg::Header header;
      header.frame_id = frame;
      header.stamp = stamp;
      ros_fb.current_best = world_points_to_path(fb.best_partial_path_world, header);
      ros_fb.search_state = build_markers(fb, frame, stamp);
      publish_feedback(ros_fb);
    };

    const RRTResult core_result = core_.plan(grid, start_v, goal_v, params, feedback_cb);

    result.plan_duration_ms = core_result.plan_duration_ms;
    result.success = core_result.success;
    result.message = core_result.message;
    if (core_result.success) {
      std_msgs::msg::Header header;
      header.frame_id = frame;
      header.stamp = node()->now();
      result.path = world_points_to_path(core_result.path_world, header);
    } else {
      result.path = nav_msgs::msg::Path();
      result.path.header.frame_id = frame;
      result.path.header.stamp = node()->now();
    }
  }

private:
  RRTCore core_;
};

}  // namespace pathfinder_algo_rrt

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rrt_planner");
  auto server = std::make_shared<pathfinder_algo_rrt::RRTServer>(node.get());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
