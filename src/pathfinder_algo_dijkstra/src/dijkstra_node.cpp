#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pathfinder_core/planner_action_server_base.hpp>

#include "pathfinder_algo_dijkstra/dijkstra_grid.hpp"

namespace pathfinder_algo_dijkstra
{

namespace
{

geometry_msgs::msg::PoseStamped make_pose(
  const WorldPoint & p,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  geometry_msgs::msg::PoseStamped out;
  out.header.frame_id = frame_id;
  out.header.stamp = stamp;
  out.pose.position.x = p.x;
  out.pose.position.y = p.y;
  out.pose.position.z = p.z;
  out.pose.orientation.w = 1.0;
  return out;
}

nav_msgs::msg::Path make_path(
  const std::vector<WorldPoint> & pts,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = stamp;
  path.poses.reserve(pts.size());
  for (const auto & p : pts) {
    path.poses.push_back(make_pose(p, frame_id, stamp));
  }
  return path;
}

visualization_msgs::msg::Marker make_cube_marker(
  const std::vector<WorldPoint> & pts,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const std::string & ns,
  int id,
  double scale,
  float r, float g, float b, float a,
  const WorldPoint * fade_origin = nullptr)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::msg::Marker::CUBE_LIST;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = scale;
  m.scale.y = scale;
  m.scale.z = scale;
  m.pose.orientation.w = 1.0;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
  m.points.reserve(pts.size());
  if (fade_origin) {
    m.colors.reserve(pts.size());
  }
  double max_dist = 1.0;
  if (fade_origin) {
    for (const auto & p : pts) {
      const double dx = p.x - fade_origin->x;
      const double dy = p.y - fade_origin->y;
      const double dz = p.z - fade_origin->z;
      const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (d > max_dist) {
        max_dist = d;
      }
    }
  }
  for (const auto & p : pts) {
    geometry_msgs::msg::Point pt;
    pt.x = p.x;
    pt.y = p.y;
    pt.z = p.z;
    m.points.push_back(pt);
    if (fade_origin) {
      const double dx = p.x - fade_origin->x;
      const double dy = p.y - fade_origin->y;
      const double dz = p.z - fade_origin->z;
      const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
      const double t = std::min(1.0, d / max_dist);
      std_msgs::msg::ColorRGBA c;
      const float shade = static_cast<float>(0.75 - 0.55 * t);
      c.r = shade;
      c.g = shade;
      c.b = shade;
      c.a = a;
      m.colors.push_back(c);
    }
  }
  return m;
}

visualization_msgs::msg::Marker make_line_marker(
  const std::vector<WorldPoint> & pts,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const std::string & ns,
  int id,
  double scale,
  float r, float g, float b, float a)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::msg::Marker::LINE_STRIP;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = scale;
  m.pose.orientation.w = 1.0;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
  m.points.reserve(pts.size());
  for (const auto & p : pts) {
    geometry_msgs::msg::Point pt;
    pt.x = p.x;
    pt.y = p.y;
    pt.z = p.z;
    m.points.push_back(pt);
  }
  return m;
}

}  // namespace

class DijkstraServer : public pathfinder_core::PlannerActionServerBase
{
public:
  DijkstraServer(rclcpp::Node * node, const std::string & action_name)
  : pathfinder_core::PlannerActionServerBase(node, action_name)
  {
  }

protected:
  void compute_path(
    const Goal & goal,
    const octomap_msgs::msg::Octomap & latest_map,
    std::function<void(const Feedback &)> publish_feedback,
    Result & result) override
  {
    const auto clock_start = std::chrono::steady_clock::now();
    auto * logger_node = node();
    const std::string output_frame = "map";

    const double robot_radius =
      logger_node->get_parameter("robot_radius").as_double();
    const double max_plan_time_sec =
      logger_node->get_parameter("max_plan_time_sec").as_double();
    const int feedback_node_stride =
      static_cast<int>(logger_node->get_parameter("feedback_node_stride").as_int());
    const double feedback_time_stride_sec =
      logger_node->get_parameter("feedback_time_stride_sec").as_double();
    const bool publish_current_best =
      logger_node->get_parameter("publish_current_best").as_bool();

    octomap::AbstractOcTree * abstract = octomap_msgs::msgToMap(latest_map);
    if (!abstract) {
      result.success = false;
      result.message = "failed to deserialize octomap";
      return;
    }
    auto * tree = dynamic_cast<octomap::OcTree *>(abstract);
    if (!tree) {
      delete abstract;
      result.success = false;
      result.message = "octomap is not an OcTree";
      return;
    }

    DijkstraGrid grid(*tree, robot_radius);
    delete tree;

    PlanRequest req{};
    req.start = {
      goal.start.pose.position.x,
      goal.start.pose.position.y,
      goal.start.pose.position.z};
    req.goal = {
      goal.goal.pose.position.x,
      goal.goal.pose.position.y,
      goal.goal.pose.position.z};
    req.robot_radius = robot_radius;
    req.max_plan_time_sec = max_plan_time_sec;

    auto on_progress = [&](const PlanProgress & prog) {
      Feedback fb;
      fb.nodes_explored = prog.nodes_explored;
      fb.best_cost_so_far = prog.best_cost_so_far;
      const auto stamp = logger_node->now();
      if (publish_current_best) {
        fb.current_best = make_path(prog.current_best, output_frame, stamp);
      } else {
        fb.current_best.header.frame_id = output_frame;
        fb.current_best.header.stamp = stamp;
      }
      visualization_msgs::msg::MarkerArray markers;
      const double cube_scale = prog.resolution * 0.9;
      markers.markers.push_back(make_cube_marker(
        prog.explored_set, output_frame, stamp, "EXPLORED_SET", 0,
        cube_scale, 0.5f, 0.5f, 0.5f, 0.4f, &prog.start));
      markers.markers.push_back(make_cube_marker(
        prog.frontier_set, output_frame, stamp, "CURRENT_FRONTIER", 1,
        cube_scale, 1.0f, 1.0f, 0.0f, 0.7f, nullptr));
      if (publish_current_best && prog.current_best.size() >= 2) {
        markers.markers.push_back(make_line_marker(
          prog.current_best, output_frame, stamp, "CURRENT_BEST_PATH", 2,
          0.05, 0.0f, 1.0f, 0.0f, 0.9f));
      }
      fb.search_state = markers;
      publish_feedback(fb);
    };

    const auto outcome = grid.plan(
      req, on_progress, feedback_node_stride, feedback_time_stride_sec);

    const auto stamp = logger_node->now();
    result.path = make_path(outcome.path, output_frame, stamp);
    result.success = outcome.success;
    result.message = outcome.message;
    const auto clock_end = std::chrono::steady_clock::now();
    result.plan_duration_ms = std::chrono::duration<double, std::milli>(
      clock_end - clock_start).count();

    RCLCPP_INFO(
      logger_node->get_logger(),
      "dijkstra plan: success=%d nodes=%d cost=%.3f duration_ms=%.2f msg=%s",
      static_cast<int>(outcome.success),
      outcome.nodes_explored,
      outcome.best_cost,
      result.plan_duration_ms,
      outcome.message.c_str());
  }
};

class DijkstraPlannerNode : public rclcpp::Node
{
public:
  DijkstraPlannerNode()
  : rclcpp::Node("dijkstra_planner")
  {
    declare_parameter<double>("robot_radius", 0.25);
    declare_parameter<double>("max_plan_time_sec", 5.0);
    declare_parameter<int>("feedback_node_stride", 200);
    declare_parameter<double>("feedback_time_stride_sec", 0.2);
    declare_parameter<bool>("publish_current_best", true);
  }

  void start_server()
  {
    server_ = std::make_unique<DijkstraServer>(this, "/dijkstra/request_path");
  }

private:
  std::unique_ptr<DijkstraServer> server_;
};

}  // namespace pathfinder_algo_dijkstra

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pathfinder_algo_dijkstra::DijkstraPlannerNode>();
  node->start_server();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
