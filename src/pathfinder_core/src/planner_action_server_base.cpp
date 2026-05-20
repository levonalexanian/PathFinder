#include "pathfinder_core/planner_action_server_base.hpp"

#include <memory>
#include <thread>
#include <utility>

namespace pathfinder_core
{

PlannerActionServerBase::PlannerActionServerBase(
  rclcpp::Node * node,
  const std::string & action_name,
  const std::string & map_topic)
: node_(node), action_name_(action_name)
{
  using namespace std::placeholders;

  action_server_ = rclcpp_action::create_server<RequestPath>(
    node_,
    action_name_,
    std::bind(&PlannerActionServerBase::handle_goal, this, _1, _2),
    std::bind(&PlannerActionServerBase::handle_cancel, this, _1),
    std::bind(&PlannerActionServerBase::handle_accepted, this, _1));

  map_sub_ = node_->create_subscription<octomap_msgs::msg::Octomap>(
    map_topic, rclcpp::QoS(1).reliable(),
    std::bind(&PlannerActionServerBase::on_map, this, _1));
}

rclcpp_action::GoalResponse PlannerActionServerBase::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const Goal> /*goal*/)
{
  std::lock_guard<std::mutex> lk(map_mutex_);
  if (!latest_map_.has_value()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[%s] rejecting goal: no octomap received yet",
      action_name_.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlannerActionServerBase::handle_cancel(
  const std::shared_ptr<GoalHandle> /*goal_handle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlannerActionServerBase::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread{std::bind(&PlannerActionServerBase::execute, this, goal_handle)}.detach();
}

void PlannerActionServerBase::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<Result>();

  octomap_msgs::msg::Octomap map_copy;
  {
    std::lock_guard<std::mutex> lk(map_mutex_);
    if (!latest_map_.has_value()) {
      result->success = false;
      result->message = "no octomap available";
      goal_handle->abort(result);
      return;
    }
    map_copy = latest_map_.value();
  }

  auto publish_feedback = [goal_handle](const Feedback & fb) {
    auto fb_ptr = std::make_shared<Feedback>(fb);
    goal_handle->publish_feedback(fb_ptr);
  };

  try {
    compute_path(*goal, map_copy, publish_feedback, *result);
  } catch (const std::exception & e) {
    result->success = false;
    result->message = std::string{"planner threw: "} + e.what();
    goal_handle->abort(result);
    return;
  }

  if (goal_handle->is_canceling()) {
    goal_handle->canceled(result);
  } else if (result->success) {
    goal_handle->succeed(result);
  } else {
    goal_handle->abort(result);
  }
}

void PlannerActionServerBase::on_map(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(map_mutex_);
  latest_map_ = *msg;
}

}  // namespace pathfinder_core
