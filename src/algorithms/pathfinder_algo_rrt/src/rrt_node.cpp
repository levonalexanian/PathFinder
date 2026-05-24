#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pathfinder_core/planner_action_server_base.hpp>
#include <pathfinder_core/voxel_grid.hpp>
#include <pathfinder_algo_rrt/rrt_core.hpp>

namespace pathfinder_algo_rrt
{

namespace
{

geometry_msgs::msg::PoseStamped make_pose(
  const WorldPoint & p,
  const std::string & frame,
  const rclcpp::Time & stamp)
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id = frame;
  ps.header.stamp = stamp;
  ps.pose.position.x = p.x;
  ps.pose.position.y = p.y;
  ps.pose.position.z = p.z;
  ps.pose.orientation.w = 1.0;
  return ps;
}

nav_msgs::msg::Path make_path(
  const std::vector<WorldPoint> & pts,
  const std::string & frame,
  const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  path.header.stamp = stamp;
  path.poses.reserve(pts.size());
  for (const auto & p : pts) {
    path.poses.push_back(make_pose(p, frame, stamp));
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
  edges.header.frame_id = frame;
  edges.header.stamp = stamp;
  edges.ns = "rrt_tree_edges";
  edges.id = 0;
  edges.type = visualization_msgs::msg::Marker::LINE_LIST;
  edges.action = visualization_msgs::msg::Marker::ADD;
  edges.scale.x = 0.02;
  edges.color.r = 0.0f;
  edges.color.g = 1.0f;
  edges.color.b = 1.0f;
  edges.color.a = 0.3f;
  edges.pose.orientation.w = 1.0;
  edges.lifetime = rclcpp::Duration::from_seconds(2.0);
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
  best_nodes.header.frame_id = frame;
  best_nodes.header.stamp = stamp;
  best_nodes.ns = "rrt_best_path_nodes";
  best_nodes.id = 1;
  best_nodes.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  best_nodes.action = visualization_msgs::msg::Marker::ADD;
  best_nodes.scale.x = 0.08;
  best_nodes.scale.y = 0.08;
  best_nodes.scale.z = 0.08;
  best_nodes.color.r = 0.0f;
  best_nodes.color.g = 1.0f;
  best_nodes.color.b = 0.0f;
  best_nodes.color.a = 1.0f;
  best_nodes.pose.orientation.w = 1.0;
  best_nodes.lifetime = rclcpp::Duration::from_seconds(2.0);
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
  : pathfinder_core::PlannerActionServerBase(node, "/rrt/request_path"),
    core_(node->get_logger())
  {
    node->declare_parameter<double>("step_size", 0.3);
    node->declare_parameter<double>("goal_bias", 0.05);
    node->declare_parameter<double>("rewire_radius", 1.0);
    node->declare_parameter<double>("goal_tolerance", 0.3);
    node->declare_parameter<int>("max_iterations", 5000);
    node->declare_parameter<double>("max_plan_time_sec", 5.0);
    node->declare_parameter<double>("robot_radius", 0.25);
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
    auto * abstract_tree = octomap_msgs::msgToMap(latest_map);
    auto * octree = dynamic_cast<octomap::OcTree *>(abstract_tree);
    std::unique_ptr<octomap::AbstractOcTree> tree_guard(abstract_tree);
    if (octree == nullptr) {
      result.success = false;
      result.message = "failed to decode octomap";
      return;
    }

    RRTParams params;
    params.step_size = node()->get_parameter("step_size").as_double();
    params.goal_bias = node()->get_parameter("goal_bias").as_double();
    params.rewire_radius = node()->get_parameter("rewire_radius").as_double();
    params.goal_tolerance = node()->get_parameter("goal_tolerance").as_double();
    params.max_iterations = node()->get_parameter("max_iterations").as_int();
    params.max_plan_time_sec = node()->get_parameter("max_plan_time_sec").as_double();
    params.robot_radius = node()->get_parameter("robot_radius").as_double();
    params.min_iterations_after_goal =
      node()->get_parameter("min_iterations_after_goal").as_int();
    const int seed_param = node()->get_parameter("random_seed").as_int();
    params.random_seed = (seed_param > 0) ? static_cast<uint32_t>(seed_param) : 0u;
    const std::string frame = node()->get_parameter("output_frame").as_string();
    params.output_frame = frame;

    pathfinder_core::InflatedVoxelGrid grid(*octree, params.robot_radius);

    const auto start_v = grid.world_to_voxel(
      goal.start.pose.position.x,
      goal.start.pose.position.y,
      goal.start.pose.position.z);
    const auto goal_v = grid.world_to_voxel(
      goal.goal.pose.position.x,
      goal.goal.pose.position.y,
      goal.goal.pose.position.z);

    auto feedback_cb = [this, &publish_feedback, &frame](const RRTFeedback & fb) {
      Feedback ros_fb;
      ros_fb.nodes_explored = static_cast<int32_t>(fb.nodes_explored);
      ros_fb.best_cost_so_far = fb.best_cost_so_far;
      const auto stamp = node()->now();
      ros_fb.current_best = make_path(fb.best_partial_path_world, frame, stamp);
      ros_fb.search_state = build_markers(fb, frame, stamp);
      publish_feedback(ros_fb);
    };

    const RRTResult core_result = core_.plan(grid, start_v, goal_v, params, feedback_cb);

    result.plan_duration_ms = core_result.plan_duration_ms;
    result.success = core_result.success;
    result.message = core_result.message;
    if (core_result.success) {
      result.path = make_path(core_result.path_world, frame, node()->now());
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
