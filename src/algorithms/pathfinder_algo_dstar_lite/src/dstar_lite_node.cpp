#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>

#include <pathfinder_core/planner_action_server_base.hpp>
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

visualization_msgs::msg::Marker make_cube_marker(
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
  m.lifetime = rclcpp::Duration::from_seconds(3.0);
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
  m.lifetime = rclcpp::Duration::from_seconds(3.0);
  return m;
}

geometry_msgs::msg::Point voxel_world_point(
  const InflatedVoxelGrid & grid, std::size_t flat)
{
  const auto v = grid.from_linear_index(flat);
  const auto w = grid.voxel_to_world(v);
  geometry_msgs::msg::Point p;
  p.x = w[0];
  p.y = w[1];
  p.z = w[2];
  return p;
}

geometry_msgs::msg::Point voxel_world_point(
  const InflatedVoxelGrid & grid, const VoxelIndex & v)
{
  const auto w = grid.voxel_to_world(v);
  geometry_msgs::msg::Point p;
  p.x = w[0];
  p.y = w[1];
  p.z = w[2];
  return p;
}

}  // namespace

class DstarLiteServer : public pathfinder_core::PlannerActionServerBase
{
public:
  explicit DstarLiteServer(rclcpp::Node * node)
  : PlannerActionServerBase(node, kActionName),
    core_(node->get_logger())
  {
    node->declare_parameter<double>("robot_radius", 0.25);
    node->declare_parameter<double>("max_plan_time_sec", 5.0);
    node->declare_parameter<int>("feedback_every_nodes", 200);
    node->declare_parameter<double>("feedback_every_seconds", 0.2);
    node->declare_parameter<int>("viz_delay_ms", 0);
  }

protected:
  void compute_path(
    const Goal & goal,
    const octomap_msgs::msg::Octomap & latest_map,
    std::function<void(const Feedback &)> publish_feedback,
    Result & result) override
  {
    std::lock_guard<std::mutex> lk(state_mutex_);

    DstarLiteParams params;
    params.robot_radius = node()->get_parameter("robot_radius").as_double();
    params.max_plan_time_sec =
      node()->get_parameter("max_plan_time_sec").as_double();
    params.feedback_every_nodes = static_cast<int>(std::max<int64_t>(
      1, node()->get_parameter("feedback_every_nodes").as_int()));
    params.feedback_every_seconds =
      node()->get_parameter("feedback_every_seconds").as_double();
    params.viz_delay_ms =
      static_cast<int>(node()->get_parameter("viz_delay_ms").as_int());

    auto tree = octree_from_msg(latest_map);
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

    const VoxelIndex start_v_raw = grid_->world_to_voxel(
      goal.start.pose.position.x,
      goal.start.pose.position.y,
      goal.start.pose.position.z);
    const VoxelIndex goal_v_raw = grid_->world_to_voxel(
      goal.goal.pose.position.x,
      goal.goal.pose.position.y,
      goal.goal.pose.position.z);

    const double res = grid_->resolution();
    const int nudge_radius = std::max(
      4, static_cast<int>(std::ceil(0.5 / std::max(res, 1e-3))));
    const auto start_opt = grid_->nudge_to_free(start_v_raw, nudge_radius);
    const auto goal_opt = grid_->nudge_to_free(goal_v_raw, nudge_radius);
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

    const InflatedVoxelGrid & grid_ref = *grid_;
    auto feedback_cb = [&](const DstarLiteFeedback & fb) {
      Feedback ros_fb;
      ros_fb.nodes_explored = fb.nodes_explored;

      const auto stamp = node()->now();
      const double cube_size = res * 0.6;

      auto open_marker = make_cube_marker(
        stamp, "OPEN_SET", 0, cube_size, 1.0f, 1.0f, 0.0f, 0.35f);
      open_marker.points.reserve(fb.sampled_open_flat.size());
      for (auto f : fb.sampled_open_flat) {
        open_marker.points.push_back(voxel_world_point(grid_ref, f));
      }

      auto locked_marker = make_cube_marker(
        stamp, "LOCKED_DOWN", 1, cube_size, 0.5f, 0.5f, 0.5f, 0.2f);
      locked_marker.points.reserve(fb.sampled_closed_flat.size());
      for (auto f : fb.sampled_closed_flat) {
        locked_marker.points.push_back(voxel_world_point(grid_ref, f));
      }

      auto path_marker = make_line_marker(
        stamp, "CURRENT_BEST_PATH", 2, res * 0.3, 0.0f, 1.0f, 0.0f, 0.9f);
      path_marker.points.reserve(fb.best_partial_path.size());
      for (const auto & v : fb.best_partial_path) {
        path_marker.points.push_back(voxel_world_point(grid_ref, v));
      }

      nav_msgs::msg::Path cb;
      cb.header.frame_id = kMapFrame;
      cb.header.stamp = stamp;
      cb.poses.reserve(fb.best_partial_path.size());
      for (const auto & v : fb.best_partial_path) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = cb.header;
        const auto w = grid_ref.voxel_to_world(v);
        ps.pose.position.x = w[0];
        ps.pose.position.y = w[1];
        ps.pose.position.z = w[2];
        ps.pose.orientation.w = 1.0;
        cb.poses.push_back(ps);
      }
      ros_fb.current_best = cb;
      ros_fb.best_cost_so_far = fb.best_cost_so_far;

      ros_fb.search_state.markers.push_back(open_marker);
      ros_fb.search_state.markers.push_back(locked_marker);
      ros_fb.search_state.markers.push_back(path_marker);

      publish_feedback(ros_fb);
    };

    const auto core_result = core_.plan(grid_ref, *start_opt, *goal_opt, params, feedback_cb);

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
