#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <octomap_msgs/msg/octomap.hpp>
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
    active_algorithm_ = get_parameter("default_algorithm").as_string();
    base_frame_ = get_parameter("base_frame").as_string();

    status_pub_ = create_publisher<pathfinder_msgs::msg::PlannerStatus>(
      kPlannerStatusTopic, rclcpp::QoS(10));
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      kPlannedPathTopic, rclcpp::QoS(10));

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
    active_algorithm_ = msg->algorithm_name;
    if (!msg->map_frame.empty()) {
      map_frame_ = msg->map_frame;
    }
    rebuild_action_client();
  }

  void on_map(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    have_map_ = true;
    if (!msg->header.frame_id.empty()) {
      map_frame_ = msg->header.frame_id;
    }
  }

  void on_goal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose)
  {
    bool have_map_local;
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      have_map_local = have_map_;
    }

    if (!have_map_local) {
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
      };
    opts.result_callback =
      [this](const GoalHandle::WrappedResult & wrapped) {
        planning_ = false;
        if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_WARN(get_logger(), "planner did not succeed");
          return;
        }
        if (wrapped.result->success) {
          path_pub_->publish(wrapped.result->path);
          last_plan_duration_ms_ = wrapped.result->plan_duration_ms;
          RCLCPP_INFO(
            get_logger(), "planner succeeded in %.2f ms",
            wrapped.result->plan_duration_ms);
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
    status.planning = planning_;
    status.last_plan_duration_ms = last_plan_duration_ms_;
    status_pub_->publish(status);
  }

  std::string active_algorithm_;
  std::string map_frame_ = "map";
  std::string base_frame_ = "base_link";
  bool have_map_ = false;
  bool planning_ = false;
  double last_plan_duration_ms_ = 0.0;

  std::mutex state_mutex_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<pathfinder_msgs::msg::AlgorithmSelection>::SharedPtr algorithm_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_sub_;
  rclcpp::Publisher<pathfinder_msgs::msg::PlannerStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp_action::Client<RequestPath>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SchedulerNode>());
  rclcpp::shutdown();
  return 0;
}
