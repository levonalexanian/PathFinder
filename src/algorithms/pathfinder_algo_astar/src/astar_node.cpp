#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <pathfinder_core/planner_action_server_base.hpp>
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

void fill_marker_header(
  visualization_msgs::msg::Marker & m,
  const rclcpp::Time & stamp,
  int id,
  const std::string & ns)
{
  m.header.frame_id = kMapFrame;
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = id;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.w = 1.0;
}

geometry_msgs::msg::Point voxel_point(const InflatedVoxelGrid & grid, std::size_t flat)
{
  const auto v = grid.from_linear_index(flat);
  const auto w = grid.voxel_to_world(v);
  geometry_msgs::msg::Point p;
  p.x = w[0];
  p.y = w[1];
  p.z = w[2];
  return p;
}

visualization_msgs::msg::Marker make_voxel_marker(
  const rclcpp::Time & stamp,
  const std::string & ns,
  int id,
  double cube_size,
  float r, float g, float b, float a)
{
  visualization_msgs::msg::Marker m;
  fill_marker_header(m, stamp, id, ns);
  m.type = visualization_msgs::msg::Marker::CUBE_LIST;
  m.scale.x = cube_size;
  m.scale.y = cube_size;
  m.scale.z = cube_size;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
  m.lifetime = rclcpp::Duration::from_seconds(2.0);
  return m;
}

visualization_msgs::msg::Marker make_line_marker(
  const rclcpp::Time & stamp,
  const std::string & ns,
  int id,
  double width,
  float r, float g, float b, float a)
{
  visualization_msgs::msg::Marker m;
  fill_marker_header(m, stamp, id, ns);
  m.type = visualization_msgs::msg::Marker::LINE_STRIP;
  m.scale.x = width;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
  m.lifetime = rclcpp::Duration::from_seconds(2.0);
  return m;
}

std::unique_ptr<octomap::OcTree> octree_from_msg(const octomap_msgs::msg::Octomap & msg)
{
  std::unique_ptr<octomap::AbstractOcTree> abstract_tree{octomap_msgs::msgToMap(msg)};
  if (!abstract_tree) {
    return nullptr;
  }
  auto * raw = dynamic_cast<octomap::OcTree *>(abstract_tree.get());
  if (raw == nullptr) {
    return nullptr;
  }
  abstract_tree.release();
  return std::unique_ptr<octomap::OcTree>(raw);
}

}  // namespace

class AstarServer : public pathfinder_core::PlannerActionServerBase
{
public:
  explicit AstarServer(rclcpp::Node * node)
  : PlannerActionServerBase(node, kActionName),
    core_(node->get_logger())
  {
    node->declare_parameter<double>("robot_radius", 0.25);
    node->declare_parameter<double>("max_plan_time_sec", 5.0);
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
    AstarParams params;
    params.robot_radius = node()->get_parameter("robot_radius").as_double();
    params.max_plan_time_sec = node()->get_parameter("max_plan_time_sec").as_double();
    params.feedback_every_nodes = static_cast<int>(std::max<int64_t>(
      1, node()->get_parameter("feedback_every_nodes").as_int()));
    params.feedback_every_seconds =
      node()->get_parameter("feedback_every_seconds").as_double();

    auto tree = octree_from_msg(latest_map);
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

    const VoxelIndex start_v = grid.world_to_voxel(
      goal.start.pose.position.x,
      goal.start.pose.position.y,
      goal.start.pose.position.z);
    const VoxelIndex goal_v = grid.world_to_voxel(
      goal.goal.pose.position.x,
      goal.goal.pose.position.y,
      goal.goal.pose.position.z);

    const int nudge_radius = std::max(
      4, static_cast<int>(std::ceil(0.5 / std::max(res, 1e-3))));
    const auto start_opt = grid.nudge_to_free(start_v, nudge_radius);
    const auto goal_opt = grid.nudge_to_free(goal_v, nudge_radius);
    if (!start_opt) {
      result.success = false;
      result.message = "start voxel is occupied and no nearby free voxel found";
      return;
    }
    if (!goal_opt) {
      result.success = false;
      result.message = "goal voxel is occupied and no nearby free voxel found";
      return;
    }
    const VoxelIndex start_idx = *start_opt;
    const VoxelIndex goal_idx = *goal_opt;

    auto feedback_cb = [&](const AstarFeedback & afb) {
      Feedback fb;
      fb.nodes_explored = afb.nodes_explored;

      const auto stamp = node()->now();
      const double cube_size = res * 0.6;

      auto open_marker = make_voxel_marker(
        stamp, "OPEN_SET", 0, cube_size, 1.0f, 1.0f, 0.0f, 0.35f);
      open_marker.points.reserve(afb.sampled_open_flat.size());
      for (auto f : afb.sampled_open_flat) {
        open_marker.points.push_back(voxel_point(grid, f));
      }

      auto closed_marker = make_voxel_marker(
        stamp, "CLOSED_SET", 1, cube_size, 0.5f, 0.5f, 0.5f, 0.2f);
      closed_marker.points.reserve(afb.sampled_closed_flat.size());
      for (auto f : afb.sampled_closed_flat) {
        closed_marker.points.push_back(voxel_point(grid, f));
      }

      auto path_marker = make_line_marker(
        stamp, "CURRENT_BEST_PATH", 2, res * 0.3, 0.0f, 1.0f, 0.0f, 0.9f);
      if (!afb.best_partial_path.empty()) {
        path_marker.points.reserve(afb.best_partial_path.size());
        for (const auto & v : afb.best_partial_path) {
          const auto w = grid.voxel_to_world(v);
          geometry_msgs::msg::Point p;
          p.x = w[0];
          p.y = w[1];
          p.z = w[2];
          path_marker.points.push_back(p);
        }
        nav_msgs::msg::Path cb;
        cb.header.frame_id = kMapFrame;
        cb.header.stamp = stamp;
        cb.poses.reserve(afb.best_partial_path.size());
        for (const auto & v : afb.best_partial_path) {
          geometry_msgs::msg::PoseStamped ps;
          ps.header = cb.header;
          const auto w = grid.voxel_to_world(v);
          ps.pose.position.x = w[0];
          ps.pose.position.y = w[1];
          ps.pose.position.z = w[2];
          ps.pose.orientation.w = 1.0;
          cb.poses.push_back(ps);
        }
        fb.current_best = cb;
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

    nav_msgs::msg::Path path;
    path.header.frame_id = kMapFrame;
    path.header.stamp = node()->now();
    path.poses.reserve(ares.path.size() + 2);

    {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose = goal.start.pose;
      path.poses.push_back(ps);
    }
    for (const auto & v : ares.path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      const auto w = grid.voxel_to_world(v);
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
