#include <algorithm>
#include <chrono>
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

#include "pathfinder_algo_dstar_lite/dstar_lite_grid.hpp"

namespace pathfinder_algo_dstar_lite
{

namespace
{

constexpr char kActionName[] = "/dstar_lite/request_path";
constexpr char kMapFrame[] = "map";
constexpr std::size_t kMaxMarkerVoxels = 500;

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
// snapshots of the same OcTree (same memory contents) will produce the same
// signature; a re-published identical map will match. A genuine update flips
// at least one occupancy bit and almost certainly the size, changing the hash.
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

}  // namespace

class DstarLiteServer : public pathfinder_core::PlannerActionServerBase
{
public:
  explicit DstarLiteServer(rclcpp::Node * node)
  : PlannerActionServerBase(node, kActionName)
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
    std::lock_guard<std::mutex> lk(state_mutex_);

    const auto start_wall = std::chrono::steady_clock::now();

    const double robot_radius = node()->get_parameter("robot_radius").as_double();
    const double max_plan_time_sec =
      node()->get_parameter("max_plan_time_sec").as_double();
    const std::int32_t feedback_every_nodes = static_cast<std::int32_t>(std::max<int64_t>(
      1, node()->get_parameter("feedback_every_nodes").as_int()));
    const double feedback_every_seconds =
      node()->get_parameter("feedback_every_seconds").as_double();

    auto tree = octree_from_msg(latest_map);
    if (!tree) {
      result.success = false;
      result.message = "failed to deserialize octomap";
      return;
    }
    const std::uint64_t signature = compute_map_signature(*tree);

    const bool need_full_rebuild =
      !state_ || state_->map_signature() != signature ||
      state_->grid().cell_count() == 0;

    if (need_full_rebuild) {
      state_ = std::make_unique<DstarLiteGrid>(
        std::move(tree), robot_radius, signature);
      last_start_flat_.reset();
      last_goal_flat_.reset();
    }
    const InflatedVoxelGrid * grid_ptr = &state_->grid();

    if (grid_ptr->cell_count() == 0) {
      result.success = false;
      result.message = "voxel grid is empty";
      return;
    }

    const VoxelIndex start_v_raw = grid_ptr->world_to_voxel(
      goal.start.pose.position.x,
      goal.start.pose.position.y,
      goal.start.pose.position.z);
    const VoxelIndex goal_v_raw = grid_ptr->world_to_voxel(
      goal.goal.pose.position.x,
      goal.goal.pose.position.y,
      goal.goal.pose.position.z);

    const double res = grid_ptr->resolution();
    const int nudge_radius = std::max(
      4, static_cast<int>(std::ceil(0.5 / std::max(res, 1e-3))));
    const auto start_opt = grid_ptr->nudge_to_free(start_v_raw, nudge_radius);
    const auto goal_opt = grid_ptr->nudge_to_free(goal_v_raw, nudge_radius);
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
    const std::size_t start_flat = grid_ptr->linear_index(*start_opt);
    const std::size_t goal_flat = grid_ptr->linear_index(*goal_opt);

    const bool goal_changed =
      !last_goal_flat_.has_value() || *last_goal_flat_ != goal_flat;
    const bool start_changed =
      !last_start_flat_.has_value() || *last_start_flat_ != start_flat;

    if (need_full_rebuild || goal_changed) {
      state_->initialize(start_flat, goal_flat);
      RCLCPP_INFO(
        node()->get_logger(),
        "D* Lite: %s (start=%zu goal=%zu sig=%lu)",
        need_full_rebuild ? "full rebuild (map signature or first call)"
                          : "goal changed; reinitializing",
        start_flat, goal_flat,
        static_cast<unsigned long>(signature));
    } else if (start_changed) {
      state_->shift_start(start_flat);
      RCLCPP_INFO(
        node()->get_logger(),
        "D* Lite: incremental replan, start moved (k_m=%.3f)",
        state_->k_m());
    } else {
      RCLCPP_INFO(node()->get_logger(), "D* Lite: nothing changed, replaying solution");
    }

    last_start_flat_ = start_flat;
    last_goal_flat_ = goal_flat;

    auto publish_state_feedback = [&](std::int32_t expansions) {
      Feedback fb;
      fb.nodes_explored = expansions;

      const auto stamp = node()->now();
      const double cube_size = res * 0.6;

      auto open_marker = make_cube_marker(
        stamp, "OPEN_SET", 0, cube_size, 1.0f, 1.0f, 0.0f, 0.7f);
      const auto open_flats = state_->sample_open_flats(kMaxMarkerVoxels);
      open_marker.points.reserve(open_flats.size());
      for (auto f : open_flats) {
        open_marker.points.push_back(voxel_world_point(*grid_ptr, f));
      }

      auto locked_marker = make_cube_marker(
        stamp, "LOCKED_DOWN", 1, cube_size, 0.5f, 0.5f, 0.5f, 0.4f);
      const auto locked_flats = state_->sample_locked_flats(kMaxMarkerVoxels);
      locked_marker.points.reserve(locked_flats.size());
      for (auto f : locked_flats) {
        locked_marker.points.push_back(voxel_world_point(*grid_ptr, f));
      }

      auto path_marker = make_line_marker(
        stamp, "CURRENT_BEST_PATH", 2, res * 0.3, 0.0f, 1.0f, 0.0f, 0.9f);
      const auto best_voxels =
        state_->extract_path(static_cast<std::size_t>(grid_ptr->cell_count()));
      path_marker.points.reserve(best_voxels.size());
      for (const auto & v : best_voxels) {
        const auto w = grid_ptr->voxel_to_world(v);
        geometry_msgs::msg::Point p;
        p.x = w[0];
        p.y = w[1];
        p.z = w[2];
        path_marker.points.push_back(p);
      }
      nav_msgs::msg::Path cb;
      cb.header.frame_id = kMapFrame;
      cb.header.stamp = stamp;
      cb.poses.reserve(best_voxels.size());
      for (const auto & v : best_voxels) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = cb.header;
        const auto w = grid_ptr->voxel_to_world(v);
        ps.pose.position.x = w[0];
        ps.pose.position.y = w[1];
        ps.pose.position.z = w[2];
        ps.pose.orientation.w = 1.0;
        cb.poses.push_back(ps);
      }
      fb.current_best = cb;
      fb.best_cost_so_far = state_->g(state_->s_start_flat());

      fb.search_state.markers.push_back(open_marker);
      fb.search_state.markers.push_back(locked_marker);
      fb.search_state.markers.push_back(path_marker);

      publish_feedback(fb);
    };

    const auto stats = state_->compute_shortest_path(
      max_plan_time_sec,
      feedback_every_nodes,
      feedback_every_seconds,
      publish_state_feedback);

    const auto end_wall = std::chrono::steady_clock::now();
    result.plan_duration_ms =
      std::chrono::duration<double, std::milli>(end_wall - start_wall).count();

    if (stats.timed_out) {
      result.success = false;
      result.message = "time budget exceeded";
      RCLCPP_WARN(
        node()->get_logger(),
        "D* Lite aborted after %.2f ms; %d expansions",
        result.plan_duration_ms, stats.expansions);
      return;
    }
    if (!stats.reached_start) {
      result.success = false;
      result.message = "no path found";
      RCLCPP_WARN(
        node()->get_logger(),
        "D* Lite failed: open exhausted (%d expansions, %.2f ms)",
        stats.expansions, result.plan_duration_ms);
      return;
    }

    const auto path_voxels =
      state_->extract_path(static_cast<std::size_t>(grid_ptr->cell_count()));
    if (path_voxels.empty()) {
      result.success = false;
      result.message = "no path found";
      return;
    }

    nav_msgs::msg::Path path;
    path.header.frame_id = kMapFrame;
    path.header.stamp = node()->now();
    path.poses.reserve(path_voxels.size() + 2);

    {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose = goal.start.pose;
      path.poses.push_back(ps);
    }
    for (const auto & v : path_voxels) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      const auto w = grid_ptr->voxel_to_world(v);
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
      "D* Lite ok: %zu poses, %d expansions, %.2f ms",
      path.poses.size(), stats.expansions, result.plan_duration_ms);
  }

private:
  std::mutex state_mutex_;
  std::unique_ptr<DstarLiteGrid> state_;
  std::optional<std::size_t> last_start_flat_;
  std::optional<std::size_t> last_goal_flat_;
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
