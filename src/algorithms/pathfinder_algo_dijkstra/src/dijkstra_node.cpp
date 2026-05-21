#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

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
#include <pathfinder_core/voxel_grid.hpp>

#include "pathfinder_algo_dijkstra/dijkstra_core.hpp"

namespace pathfinder_algo_dijkstra
{

namespace
{

struct WorldPoint
{
  double x;
  double y;
  double z;
};

WorldPoint voxel_to_world_point(
  const pathfinder_core::InflatedVoxelGrid & grid,
  const pathfinder_core::VoxelIndex & v)
{
  const auto a = grid.voxel_to_world(v);
  return {a[0], a[1], a[2]};
}

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

std::vector<WorldPoint> voxels_to_world(
  const pathfinder_core::InflatedVoxelGrid & grid,
  const std::vector<pathfinder_core::VoxelIndex> & voxels)
{
  std::vector<WorldPoint> out;
  out.reserve(voxels.size());
  for (const auto & v : voxels) {
    out.push_back(voxel_to_world_point(grid, v));
  }
  return out;
}

std::vector<WorldPoint> flat_indices_to_world(
  const pathfinder_core::InflatedVoxelGrid & grid,
  const std::vector<std::size_t> & flat)
{
  std::vector<WorldPoint> out;
  out.reserve(flat.size());
  for (auto lin : flat) {
    out.push_back(voxel_to_world_point(grid, grid.from_linear_index(lin)));
  }
  return out;
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
  : pathfinder_core::PlannerActionServerBase(node, action_name),
    core_(node->get_logger())
  {
  }

protected:
  void compute_path(
    const Goal & goal,
    const octomap_msgs::msg::Octomap & latest_map,
    std::function<void(const Feedback &)> publish_feedback,
    Result & result) override
  {
    auto * logger_node = node();
    const std::string output_frame = "map";

    DijkstraParams params;
    params.robot_radius =
      logger_node->get_parameter("robot_radius").as_double();
    params.max_plan_time_sec =
      logger_node->get_parameter("max_plan_time_sec").as_double();
    params.feedback_node_stride =
      static_cast<int>(logger_node->get_parameter("feedback_node_stride").as_int());
    params.feedback_time_stride_sec =
      logger_node->get_parameter("feedback_time_stride_sec").as_double();
    params.publish_current_best =
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

    pathfinder_core::InflatedVoxelGrid grid(*tree, params.robot_radius);
    delete tree;

    const auto start_v = grid.world_to_voxel(
      goal.start.pose.position.x,
      goal.start.pose.position.y,
      goal.start.pose.position.z);
    const auto goal_v = grid.world_to_voxel(
      goal.goal.pose.position.x,
      goal.goal.pose.position.y,
      goal.goal.pose.position.z);

    const WorldPoint start_world{
      goal.start.pose.position.x,
      goal.start.pose.position.y,
      goal.start.pose.position.z};

    auto on_feedback = [&](const DijkstraFeedback & cb) {
      Feedback fb;
      fb.nodes_explored = cb.nodes_explored;
      fb.best_cost_so_far = cb.best_cost_so_far;
      const auto stamp = logger_node->now();
      const auto best_world = voxels_to_world(grid, cb.best_partial_path);
      if (params.publish_current_best) {
        fb.current_best = make_path(best_world, output_frame, stamp);
      } else {
        fb.current_best.header.frame_id = output_frame;
        fb.current_best.header.stamp = stamp;
      }
      const auto explored_world = flat_indices_to_world(grid, cb.sampled_closed_flat);
      const auto frontier_world = flat_indices_to_world(grid, cb.sampled_open_flat);
      visualization_msgs::msg::MarkerArray markers;
      const double cube_scale = grid.resolution() * 0.9;
      markers.markers.push_back(make_cube_marker(
        explored_world, output_frame, stamp, "EXPLORED_SET", 0,
        cube_scale, 0.5f, 0.5f, 0.5f, 0.4f, &start_world));
      markers.markers.push_back(make_cube_marker(
        frontier_world, output_frame, stamp, "CURRENT_FRONTIER", 1,
        cube_scale, 1.0f, 1.0f, 0.0f, 0.7f, nullptr));
      if (params.publish_current_best && best_world.size() >= 2) {
        markers.markers.push_back(make_line_marker(
          best_world, output_frame, stamp, "CURRENT_BEST_PATH", 2,
          0.05, 0.0f, 1.0f, 0.0f, 0.9f));
      }
      fb.search_state = markers;
      publish_feedback(fb);
    };

    const auto outcome = core_.plan(grid, start_v, goal_v, params, on_feedback);

    const auto stamp = logger_node->now();
    const auto path_world = voxels_to_world(grid, outcome.path);
    result.path = make_path(path_world, output_frame, stamp);
    result.success = outcome.success;
    result.message = outcome.message;
    result.plan_duration_ms = outcome.plan_duration_ms;

    RCLCPP_INFO(
      logger_node->get_logger(),
      "dijkstra plan: success=%d nodes=%d duration_ms=%.2f msg=%s",
      static_cast<int>(outcome.success),
      outcome.nodes_expanded,
      outcome.plan_duration_ms,
      outcome.message.c_str());
  }

private:
  DijkstraCore core_;
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
