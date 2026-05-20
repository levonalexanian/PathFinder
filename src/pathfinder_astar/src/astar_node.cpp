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

#include <pathfinder_core/planner_action_server_base.hpp>
#include "pathfinder_astar/astar_grid.hpp"

namespace pathfinder_astar
{

namespace
{

constexpr char kActionName[] = "/astar/request_path";
constexpr char kNodeName[] = "astar_planner";
constexpr char kMapFrame[] = "map";
constexpr std::size_t kMaxMarkerVoxels = 500;

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

geometry_msgs::msg::Point voxel_point(const VoxelGrid & grid, std::size_t flat)
{
  const auto v = unflatten(flat, grid.dims);
  geometry_msgs::msg::Point p;
  grid.voxel_to_world(v.x, v.y, v.z, p.x, p.y, p.z);
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

    auto grid = build_inflated_grid(*tree, robot_radius);
    if (grid.dims.size() == 0) {
      result.success = false;
      result.message = "voxel grid is empty";
      return;
    }

    const double res = grid.resolution;
    RCLCPP_INFO(
      node()->get_logger(),
      "A* search on %dx%dx%d grid (res=%.3f m, radius=%.2f m)",
      grid.dims.nx, grid.dims.ny, grid.dims.nz, res, robot_radius);

    const auto start_v = grid.world_to_voxel(
      goal.start.pose.position.x,
      goal.start.pose.position.y,
      goal.start.pose.position.z);
    const auto goal_v = grid.world_to_voxel(
      goal.goal.pose.position.x,
      goal.goal.pose.position.y,
      goal.goal.pose.position.z);

    const int nudge_radius = std::max(
      4, static_cast<int>(std::ceil(0.5 / std::max(res, 1e-3))));
    const auto start_candidates =
      nudge_to_free(grid, start_v.x, start_v.y, start_v.z, nudge_radius);
    const auto goal_candidates =
      nudge_to_free(grid, goal_v.x, goal_v.y, goal_v.z, nudge_radius);
    if (start_candidates.empty()) {
      result.success = false;
      result.message = "start voxel is occupied and no nearby free voxel found";
      return;
    }
    if (goal_candidates.empty()) {
      result.success = false;
      result.message = "goal voxel is occupied and no nearby free voxel found";
      return;
    }
    const VoxelIndex start_idx = start_candidates.front();
    const VoxelIndex goal_idx = goal_candidates.front();
    const std::size_t start_flat = grid.dims.flat(start_idx.x, start_idx.y, start_idx.z);
    const std::size_t goal_flat = grid.dims.flat(goal_idx.x, goal_idx.y, goal_idx.z);

    SearchBuffers buf;
    buf.reset(grid.dims.size());
    buf.g_score[start_flat] = 0.0;

    const auto neighbors = neighbor_offsets_26();

    OpenQueue open;
    const double h_start = euclidean_voxel_distance(
      start_idx.x, start_idx.y, start_idx.z,
      goal_idx.x, goal_idx.y, goal_idx.z, res);
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

        // gather open-set voxel flats — copy queue (cheap pointer-wise)
        std::vector<std::size_t> open_flats;
        {
          OpenQueue copy = open;
          while (!copy.empty() && open_flats.size() < kMaxMarkerVoxels) {
            open_flats.push_back(copy.top().flat);
            copy.pop();
          }
        }

        // gather closed-set voxel flats with stride decimation
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
          const auto best_voxels = reconstruct_path(buf, grid.dims, best_open_flat);
          path_marker.points.reserve(best_voxels.size());
          for (const auto & v : best_voxels) {
            geometry_msgs::msg::Point p;
            grid.voxel_to_world(v.x, v.y, v.z, p.x, p.y, p.z);
            path_marker.points.push_back(p);
          }
          // also drop a "current_best" Path for the scheduler
          nav_msgs::msg::Path cb;
          cb.header.frame_id = kMapFrame;
          cb.header.stamp = stamp;
          cb.poses.reserve(best_voxels.size());
          for (const auto & v : best_voxels) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = cb.header;
            grid.voxel_to_world(v.x, v.y, v.z,
              ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
            ps.pose.orientation.w = 1.0;
            cb.poses.push_back(ps);
          }
          fb.current_best = cb;
          fb.best_cost_so_far = buf.g_score[best_open_flat] +
            euclidean_voxel_distance(
              unflatten(best_open_flat, grid.dims).x,
              unflatten(best_open_flat, grid.dims).y,
              unflatten(best_open_flat, grid.dims).z,
              goal_idx.x, goal_idx.y, goal_idx.z, res);
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

      const VoxelIndex cv = unflatten(cur.flat, grid.dims);
      for (const auto & off : neighbors) {
        const int nx = cv.x + off.x;
        const int ny = cv.y + off.y;
        const int nz = cv.z + off.z;
        if (!grid.dims.in_bounds(nx, ny, nz)) {
          continue;
        }
        const std::size_t nflat = grid.dims.flat(nx, ny, nz);
        if (buf.closed[nflat]) {
          continue;
        }
        if (grid.occupied[nflat]) {
          continue;
        }
        const double step = euclidean_voxel_distance(
          cv.x, cv.y, cv.z, nx, ny, nz, res);
        const double tentative_g = buf.g_score[cur.flat] + step;
        if (tentative_g >= buf.g_score[nflat]) {
          continue;
        }
        buf.g_score[nflat] = tentative_g;
        buf.came_from[nflat] = static_cast<int32_t>(cur.flat);
        const double h = euclidean_voxel_distance(
          nx, ny, nz, goal_idx.x, goal_idx.y, goal_idx.z, res);
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

    const auto path_voxels = reconstruct_path(buf, grid.dims, goal_flat);
    nav_msgs::msg::Path path;
    path.header.frame_id = kMapFrame;
    path.header.stamp = node()->now();
    path.poses.reserve(path_voxels.size() + 1);

    // ensure path starts exactly at requested start position
    {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose = goal.start.pose;
      path.poses.push_back(ps);
    }
    for (const auto & v : path_voxels) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      grid.voxel_to_world(v.x, v.y, v.z,
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    // ensure path ends exactly at requested goal position
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
