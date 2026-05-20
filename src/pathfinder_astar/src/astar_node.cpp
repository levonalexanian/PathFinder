#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
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
#include "pathfinder_astar/astar_grid.hpp"

namespace pathfinder_astar
{

namespace
{

constexpr char kActionName[] = "/astar/request_path";
constexpr char kMapFrame[] = "map";
constexpr std::size_t kMaxMarkerVoxels = 500;

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

double voxel_distance(const VoxelIndex & a, const VoxelIndex & b, double resolution)
{
  const double dx = static_cast<double>(a.x - b.x);
  const double dy = static_cast<double>(a.y - b.y);
  const double dz = static_cast<double>(a.z - b.z);
  return std::sqrt(dx * dx + dy * dy + dz * dz) * resolution;
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
    const auto start_wall = std::chrono::steady_clock::now();

    const double robot_radius = node()->get_parameter("robot_radius").as_double();
    const double max_plan_time_sec =
      node()->get_parameter("max_plan_time_sec").as_double();
    const int feedback_every_nodes = static_cast<int>(std::max<int64_t>(
      1, node()->get_parameter("feedback_every_nodes").as_int()));
    const double feedback_every_seconds =
      node()->get_parameter("feedback_every_seconds").as_double();

    auto tree = octree_from_msg(latest_map);
    if (!tree) {
      result.success = false;
      result.message = "failed to deserialize octomap";
      return;
    }

    InflatedVoxelGrid grid(*tree, robot_radius);
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
      dims[0], dims[1], dims[2], res, robot_radius);

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
    const std::size_t start_flat = grid.linear_index(start_idx);
    const std::size_t goal_flat = grid.linear_index(goal_idx);

    SearchBuffers buf;
    buf.reset(grid.cell_count());
    buf.g_score[start_flat] = 0.0;

    OpenQueue open;
    const double h_start = voxel_distance(start_idx, goal_idx, res);
    open.push({h_start, h_start, start_flat});
    buf.in_open[start_flat] = 1;

    std::size_t best_open_flat = start_flat;
    double best_open_f = h_start;
    int nodes_expanded = 0;
    int nodes_since_feedback = 0;
    auto last_feedback_time = start_wall;
    bool reached = false;
    bool timed_out = false;

    auto publish_state_feedback =
      [&](bool include_best_path) {
        Feedback fb;
        fb.nodes_explored = nodes_expanded;

        std::vector<std::size_t> open_flats;
        {
          OpenQueue copy = open;
          while (!copy.empty() && open_flats.size() < kMaxMarkerVoxels) {
            open_flats.push_back(copy.top().flat);
            copy.pop();
          }
        }

        std::vector<std::size_t> closed_flats;
        const std::size_t closed_total = buf.closed.size();
        std::size_t closed_count = 0;
        for (std::size_t i = 0; i < closed_total; ++i) {
          if (buf.closed[i]) {
            ++closed_count;
          }
        }
        const std::size_t stride = std::max<std::size_t>(
          1, closed_count / kMaxMarkerVoxels);
        std::size_t seen = 0;
        for (std::size_t i = 0; i < closed_total && closed_flats.size() < kMaxMarkerVoxels; ++i) {
          if (buf.closed[i]) {
            if (seen % stride == 0) {
              closed_flats.push_back(i);
            }
            ++seen;
          }
        }

        const auto stamp = node()->now();
        const double cube_size = res * 0.6;

        auto open_marker = make_voxel_marker(
          stamp, "OPEN_SET", 0, cube_size, 1.0f, 1.0f, 0.0f, 0.7f);
        open_marker.points.reserve(open_flats.size());
        for (auto f : open_flats) {
          open_marker.points.push_back(voxel_point(grid, f));
        }

        auto closed_marker = make_voxel_marker(
          stamp, "CLOSED_SET", 1, cube_size, 0.5f, 0.5f, 0.5f, 0.4f);
        closed_marker.points.reserve(closed_flats.size());
        for (auto f : closed_flats) {
          closed_marker.points.push_back(voxel_point(grid, f));
        }

        auto path_marker = make_line_marker(
          stamp, "CURRENT_BEST_PATH", 2, res * 0.3, 0.0f, 1.0f, 0.0f, 0.9f);
        if (include_best_path) {
          const auto best_voxels = reconstruct_path(buf, grid, best_open_flat);
          path_marker.points.reserve(best_voxels.size());
          for (const auto & v : best_voxels) {
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
          cb.poses.reserve(best_voxels.size());
          for (const auto & v : best_voxels) {
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
          const auto best_v = grid.from_linear_index(best_open_flat);
          fb.best_cost_so_far = buf.g_score[best_open_flat] +
            voxel_distance(best_v, goal_idx, res);
        }

        fb.search_state.markers.push_back(open_marker);
        fb.search_state.markers.push_back(closed_marker);
        fb.search_state.markers.push_back(path_marker);

        publish_feedback(fb);
      };

    while (!open.empty()) {
      const auto now_wall = std::chrono::steady_clock::now();
      const double elapsed_sec = std::chrono::duration<double>(now_wall - start_wall).count();
      if (elapsed_sec > max_plan_time_sec) {
        timed_out = true;
        break;
      }

      OpenEntry cur = open.top();
      open.pop();

      if (buf.closed[cur.flat]) {
        continue;
      }
      buf.closed[cur.flat] = 1;
      buf.in_open[cur.flat] = 0;
      ++nodes_expanded;
      ++nodes_since_feedback;

      if (cur.flat == goal_flat) {
        reached = true;
        break;
      }

      if (cur.h < best_open_f) {
        best_open_f = cur.h;
        best_open_flat = cur.flat;
      }

      const VoxelIndex cv = grid.from_linear_index(cur.flat);
      for (const auto & nb : grid.neighbors_26(cv)) {
        const std::size_t nflat = grid.linear_index(nb);
        if (buf.closed[nflat]) {
          continue;
        }
        if (grid.is_occupied(nb)) {
          continue;
        }
        const double step = voxel_distance(cv, nb, res);
        const double tentative_g = buf.g_score[cur.flat] + step;
        if (tentative_g >= buf.g_score[nflat]) {
          continue;
        }
        buf.g_score[nflat] = tentative_g;
        buf.came_from[nflat] = static_cast<int32_t>(cur.flat);
        const double h = voxel_distance(nb, goal_idx, res);
        open.push({tentative_g + h, h, nflat});
        buf.in_open[nflat] = 1;
      }

      const double since_fb_sec =
        std::chrono::duration<double>(now_wall - last_feedback_time).count();
      if (nodes_since_feedback >= feedback_every_nodes ||
          since_fb_sec >= feedback_every_seconds)
      {
        publish_state_feedback(true);
        nodes_since_feedback = 0;
        last_feedback_time = now_wall;
      }
    }

    const auto end_wall = std::chrono::steady_clock::now();
    const double plan_duration_ms =
      std::chrono::duration<double, std::milli>(end_wall - start_wall).count();
    result.plan_duration_ms = plan_duration_ms;

    if (timed_out) {
      result.success = false;
      result.message = "time budget exceeded";
      RCLCPP_WARN(
        node()->get_logger(),
        "A* aborted after %.1f ms; expanded %d nodes",
        plan_duration_ms, nodes_expanded);
      return;
    }
    if (!reached) {
      result.success = false;
      result.message = "no path found";
      RCLCPP_WARN(
        node()->get_logger(),
        "A* failed: open set exhausted after %d nodes",
        nodes_expanded);
      return;
    }

    const auto path_voxels = reconstruct_path(buf, grid, goal_flat);
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
      path.poses.size(), nodes_expanded, plan_duration_ms);
  }
};

}  // namespace pathfinder_astar

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("astar_planner");
  pathfinder_astar::AstarServer server(node.get());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
