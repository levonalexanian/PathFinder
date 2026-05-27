#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pathfinder_msgs/msg/algorithm_selection.hpp>
#include <pathfinder_msgs/msg/planner_status.hpp>
#include <pathfinder_msgs/action/request_path.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr char kPlannerStatusTopic[] = "/planner_status";
constexpr char kPlannedPathTopic[] = "/planned_path";
constexpr char kAlgorithmSelectionTopic[] = "/algorithm_selection";
constexpr char kGoalPoseTopic[] = "/goal_pose";
constexpr char kVoxelMapTopic[] = "/voxel_map";
constexpr char kSearchVisualizationTopic[] = "/search_visualization";
}  // namespace

class SchedulerNode : public rclcpp::Node
{
public:
  using RequestPath = pathfinder_msgs::action::RequestPath;
  using GoalHandle = rclcpp_action::ClientGoalHandle<RequestPath>;

  SchedulerNode()
  : rclcpp::Node("scheduler"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter<std::string>("default_algorithm", "astar");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<double>("viz_hold_sec", 2.0);
    active_algorithm_ = get_parameter("default_algorithm").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    viz_hold_sec_ = get_parameter("viz_hold_sec").as_double();

    status_pub_ = create_publisher<pathfinder_msgs::msg::PlannerStatus>(
      kPlannerStatusTopic, rclcpp::QoS(10));
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      kPlannedPathTopic, rclcpp::QoS(10));
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      kSearchVisualizationTopic, rclcpp::QoS(10).transient_local());

    algorithm_sub_ = create_subscription<pathfinder_msgs::msg::AlgorithmSelection>(
      kAlgorithmSelectionTopic, rclcpp::QoS(10),
      std::bind(&SchedulerNode::on_algorithm_selection, this, std::placeholders::_1));

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      kGoalPoseTopic, rclcpp::QoS(10),
      std::bind(&SchedulerNode::on_goal_pose, this, std::placeholders::_1));

    map_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      kVoxelMapTopic, rclcpp::QoS(1).reliable(),
      std::bind(&SchedulerNode::on_map, this, std::placeholders::_1));

    rebuild_action_client();

    status_timer_ = create_wall_timer(
      1s, std::bind(&SchedulerNode::publish_status, this));

    RCLCPP_INFO(
      get_logger(), "scheduler started; default algorithm=%s",
      active_algorithm_.c_str());
  }

private:
  std::string action_name_for(const std::string & algorithm) const
  {
    return "/" + algorithm + "/request_path";
  }

  void rebuild_action_client()
  {
    action_client_ = rclcpp_action::create_client<RequestPath>(
      this, action_name_for(active_algorithm_));
  }

  void on_algorithm_selection(const pathfinder_msgs::msg::AlgorithmSelection::SharedPtr msg)
  {
    if (msg->algorithm_name.empty()) {
      RCLCPP_WARN(get_logger(), "ignoring AlgorithmSelection with empty name");
      return;
    }
    if (msg->algorithm_name == active_algorithm_) {
      return;
    }
    RCLCPP_INFO(
      get_logger(), "switching active algorithm: %s -> %s",
      active_algorithm_.c_str(), msg->algorithm_name.c_str());
    if (planning_.load() && action_client_) {
      // cancel prior plan before switching algorithms — avoid stale callbacks
      action_client_->async_cancel_all_goals();
    }
    active_algorithm_ = msg->algorithm_name;
    if (!msg->map_frame.empty()) {
      map_frame_ = msg->map_frame;
    }
    planning_ = false;
    rebuild_action_client();
  }

  void on_map(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    have_map_ = true;
    if (!msg->header.frame_id.empty()) {
      map_frame_ = msg->header.frame_id;
    }
  }

  void on_goal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose)
  {
    if (!have_map_.load()) {
      RCLCPP_WARN(get_logger(), "goal received but no octomap yet; ignoring");
      planning_ = false;
      return;
    }
    if (!action_client_) {
      RCLCPP_WARN(get_logger(), "goal received but no action client; ignoring");
      planning_ = false;
      return;
    }
    if (!action_client_->action_server_is_ready()) {
      RCLCPP_WARN(
        get_logger(),
        "action server for '%s' not available; ignoring goal",
        active_algorithm_.c_str());
      planning_ = false;
      return;
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(),
        "no %s->%s transform yet (%s); skipping goal",
        map_frame_.c_str(), base_frame_.c_str(), e.what());
      planning_ = false;
      return;
    }

    RequestPath::Goal goal_msg;
    goal_msg.goal = *goal_pose;
    goal_msg.start.header.stamp = now();
    goal_msg.start.header.frame_id = map_frame_;
    goal_msg.start.pose.position.x = tf.transform.translation.x;
    goal_msg.start.pose.position.y = tf.transform.translation.y;
    goal_msg.start.pose.position.z = tf.transform.translation.z;
    goal_msg.start.pose.orientation = tf.transform.rotation;

    auto opts = rclcpp_action::Client<RequestPath>::SendGoalOptions{};
    opts.feedback_callback =
      [this](GoalHandle::SharedPtr,
        const std::shared_ptr<const RequestPath::Feedback> feedback) {
        RCLCPP_INFO(
          get_logger(),
          "planner feedback: explored=%d best_cost=%.3f",
          feedback->nodes_explored, feedback->best_cost_so_far);
        marker_pub_->publish(feedback->search_state);
      };
    opts.result_callback =
      [this](const GoalHandle::WrappedResult & wrapped) {
        planning_ = false;
        if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_WARN(get_logger(), "planner did not succeed");
          return;
        }
        if (wrapped.result->success) {
          last_plan_duration_ms_ = wrapped.result->plan_duration_ms;
          RCLCPP_INFO(
            get_logger(),
            "planner succeeded in %.2f ms; holding search viz %.1fs before moving",
            wrapped.result->plan_duration_ms, viz_hold_sec_);
          // Hold the search cubes on screen briefly, then clear them and only
          // THEN publish the path so the robot starts moving. Timer-based, not a
          // blocking sleep: this callback runs on the action client's executor
          // thread, so sleeping here would stall feedback/result handling.
          pending_path_ = wrapped.result->path;
          const auto hold = std::chrono::milliseconds(
            static_cast<int64_t>(viz_hold_sec_ * 1000.0));
          viz_hold_timer_ = create_wall_timer(
            hold,
            [this]() {
              viz_hold_timer_->cancel();
              publish_clear_markers();
              path_pub_->publish(pending_path_);
            });
        } else {
          RCLCPP_WARN(
            get_logger(), "planner reported failure: %s",
            wrapped.result->message.c_str());
        }
      };

    planning_ = true;
    action_client_->async_send_goal(goal_msg, opts);
  }

  void publish_status()
  {
    pathfinder_msgs::msg::PlannerStatus status;
    status.active_algorithm = active_algorithm_;
    status.planning = planning_.load();
    status.last_plan_duration_ms = last_plan_duration_ms_;
    status_pub_->publish(status);
  }

  void publish_clear_markers()
  {
    visualization_msgs::msg::MarkerArray clear;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = map_frame_;
    m.header.stamp = now();
    m.action = visualization_msgs::msg::Marker::DELETEALL;
    clear.markers.push_back(m);
    marker_pub_->publish(clear);
  }

  std::string active_algorithm_;
  std::string map_frame_ = "map";
  std::string base_frame_ = "base_link";
  std::atomic<bool> have_map_{false};
  std::atomic<bool> planning_{false};
  double last_plan_duration_ms_ = 0.0;
  double viz_hold_sec_ = 2.0;
  nav_msgs::msg::Path pending_path_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<pathfinder_msgs::msg::AlgorithmSelection>::SharedPtr algorithm_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_sub_;
  rclcpp::Publisher<pathfinder_msgs::msg::PlannerStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp_action::Client<RequestPath>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr viz_hold_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SchedulerNode>());
  rclcpp::shutdown();
  return 0;
}
