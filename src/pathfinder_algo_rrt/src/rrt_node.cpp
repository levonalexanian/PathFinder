#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
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
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pathfinder_core/planner_action_server_base.hpp>
#include <pathfinder_core/voxel_grid.hpp>
#include <pathfinder_algo_rrt/rrt_star.hpp>

namespace pathfinder_algo_rrt
{

namespace
{

constexpr std::size_t kMaxTreeEdgePoints = 1000;

geometry_msgs::msg::PoseStamped make_pose(
  const Point3 & p,
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
  const std::vector<Point3> & pts,
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

}  // namespace

class RRTServer : public pathfinder_core::PlannerActionServerBase
{
public:
  RRTServer(rclcpp::Node * node)
  : pathfinder_core::PlannerActionServerBase(node, "/rrt/request_path")
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
    const auto t_start = std::chrono::steady_clock::now();

    auto * abstract_tree = octomap_msgs::msgToMap(latest_map);
    auto * octree = dynamic_cast<octomap::OcTree *>(abstract_tree);
    std::unique_ptr<octomap::AbstractOcTree> tree_guard(abstract_tree);
    if (octree == nullptr) {
      result.success = false;
      result.message = "failed to decode octomap";
      return;
    }

    RRTStarParams params;
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
    const std::string frame = node()->get_parameter("output_frame").as_string();

    unsigned int seed = (seed_param > 0)
      ? static_cast<unsigned int>(seed_param)
      : static_cast<unsigned int>(
          std::chrono::steady_clock::now().time_since_epoch().count());
    params.random_seed = seed;

    Point3 start{
      goal.start.pose.position.x,
      goal.start.pose.position.y,
      goal.start.pose.position.z};
    Point3 goal_pt{
      goal.goal.pose.position.x,
      goal.goal.pose.position.y,
      goal.goal.pose.position.z};

    pathfinder_core::InflatedVoxelGrid grid(*octree, params.robot_radius);
    RRTStar rrt(grid, params);

    if (rrt.point_collision(start)) {
      result.success = false;
      result.message = "start in collision";
      RCLCPP_WARN(node()->get_logger(), "RRT*: start in collision");
      return;
    }
    if (rrt.point_collision(goal_pt)) {
      result.success = false;
      result.message = "goal in collision";
      RCLCPP_WARN(node()->get_logger(), "RRT*: goal in collision");
      return;
    }

    rrt.add_node(start, -1, 0.0);

    int best_goal_idx = -1;
    double best_cost = std::numeric_limits<double>::infinity();
    int iter = 0;
    int iter_at_first_goal = -1;
    auto last_feedback = std::chrono::steady_clock::now();

    for (; iter < params.max_iterations; ++iter) {
      const auto now_t = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(now_t - t_start).count();
      if (elapsed >= params.max_plan_time_sec) {
        break;
      }
      if (best_goal_idx >= 0 &&
          (iter - iter_at_first_goal) >= params.min_iterations_after_goal)
      {
        break;
      }

      const Point3 x_rand = rrt.sample(goal_pt);
      const int nearest_idx = rrt.nearest(x_rand);
      if (nearest_idx < 0) {
        continue;
      }
      const Point3 x_new = rrt.steer(rrt.nodes()[nearest_idx].position, x_rand);
      if (!rrt.point_in_bounds(x_new)) {
        continue;
      }
      if (rrt.point_collision(x_new)) {
        continue;
      }
      if (!rrt.segment_collision_free(rrt.nodes()[nearest_idx].position, x_new)) {
        continue;
      }

      auto near = rrt.neighbors_within(x_new, params.rewire_radius);
      const int parent_idx = rrt.choose_parent(near, x_new, nearest_idx);
      if (parent_idx < 0) {
        continue;
      }
      const double cost_to_new =
        rrt.nodes()[parent_idx].cost + distance(rrt.nodes()[parent_idx].position, x_new);
      rrt.add_node(x_new, parent_idx, cost_to_new);
      const int new_idx = static_cast<int>(rrt.size()) - 1;
      rrt.rewire(near, new_idx);

      if (distance(x_new, goal_pt) <= params.goal_tolerance &&
          rrt.segment_collision_free(x_new, goal_pt))
      {
        const double candidate_cost = cost_to_new + distance(x_new, goal_pt);
        if (candidate_cost < best_cost) {
          rrt.add_node(goal_pt, new_idx, candidate_cost);
          best_goal_idx = static_cast<int>(rrt.size()) - 1;
          best_cost = candidate_cost;
          if (iter_at_first_goal < 0) {
            iter_at_first_goal = iter;
          }
        }
      }

      const double since_feedback =
        std::chrono::duration<double>(now_t - last_feedback).count();
      if ((iter % 100) == 0 || since_feedback >= 0.2) {
        publish_feedback(make_feedback(rrt, best_goal_idx, best_cost, frame));
        last_feedback = now_t;
      }
    }

    const auto t_end = std::chrono::steady_clock::now();
    const double total_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    publish_feedback(make_feedback(rrt, best_goal_idx, best_cost, frame));

    result.plan_duration_ms = total_ms;
    if (best_goal_idx >= 0) {
      auto pts = rrt.reconstruct_from(best_goal_idx);
      result.path = make_path(pts, frame, node()->now());
      result.success = true;
      result.message = "ok";
      RCLCPP_INFO(
        node()->get_logger(),
        "RRT* succeeded: nodes=%zu cost=%.3f iters=%d time=%.1fms",
        rrt.size(), best_cost, iter, total_ms);
    } else {
      result.path = nav_msgs::msg::Path();
      result.path.header.frame_id = frame;
      result.path.header.stamp = node()->now();
      result.success = false;
      result.message = "no path found in budget";
      RCLCPP_WARN(
        node()->get_logger(),
        "RRT* failed: nodes=%zu iters=%d time=%.1fms",
        rrt.size(), iter, total_ms);
    }
  }

private:
  Feedback make_feedback(
    const RRTStar & rrt,
    int best_goal_idx,
    double best_cost,
    const std::string & frame)
  {
    Feedback fb;
    fb.nodes_explored = static_cast<int32_t>(rrt.size());
    fb.best_cost_so_far = std::isfinite(best_cost)
      ? best_cost
      : std::numeric_limits<double>::infinity();

    const auto stamp = node()->now();
    std::vector<Point3> best_path;
    if (best_goal_idx >= 0) {
      best_path = rrt.reconstruct_from(best_goal_idx);
      fb.current_best = make_path(best_path, frame, stamp);
    } else {
      fb.current_best.header.frame_id = frame;
      fb.current_best.header.stamp = stamp;
    }

    fb.search_state = build_markers(rrt, best_path, frame, stamp);
    return fb;
  }

  visualization_msgs::msg::MarkerArray build_markers(
    const RRTStar & rrt,
    const std::vector<Point3> & best_path,
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
    edges.color.a = 0.6f;
    edges.pose.orientation.w = 1.0;

    const auto & nodes = rrt.nodes();
    std::vector<std::size_t> edge_indices;
    edge_indices.reserve(nodes.size());
    for (std::size_t i = 1; i < nodes.size(); ++i) {
      if (nodes[i].parent_idx >= 0) {
        edge_indices.push_back(i);
      }
    }
    const std::size_t max_edges = kMaxTreeEdgePoints / 2;
    if (edge_indices.size() > max_edges) {
      std::mt19937 rng(42);
      std::shuffle(edge_indices.begin(), edge_indices.end(), rng);
      edge_indices.resize(max_edges);
    }
    edges.points.reserve(edge_indices.size() * 2);
    for (auto i : edge_indices) {
      const auto & child = nodes[i];
      const auto & parent = nodes[child.parent_idx];
      geometry_msgs::msg::Point a, b;
      a.x = parent.position.x;
      a.y = parent.position.y;
      a.z = parent.position.z;
      b.x = child.position.x;
      b.y = child.position.y;
      b.z = child.position.z;
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
    best_nodes.points.reserve(best_path.size());
    for (const auto & p : best_path) {
      geometry_msgs::msg::Point gp;
      gp.x = p.x;
      gp.y = p.y;
      gp.z = p.z;
      best_nodes.points.push_back(gp);
    }
    arr.markers.push_back(best_nodes);

    return arr;
  }
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
