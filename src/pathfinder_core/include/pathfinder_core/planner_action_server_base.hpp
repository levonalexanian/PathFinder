#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <pathfinder_msgs/action/request_path.hpp>

namespace pathfinder_core
{

class PlannerActionServerBase
{
public:
  using RequestPath = pathfinder_msgs::action::RequestPath;
  using Goal = RequestPath::Goal;
  using Result = RequestPath::Result;
  using Feedback = RequestPath::Feedback;
  using GoalHandle = rclcpp_action::ServerGoalHandle<RequestPath>;

  PlannerActionServerBase(
    rclcpp::Node * node,
    const std::string & action_name,
    const std::string & map_topic = "/voxel_map");

  virtual ~PlannerActionServerBase() = default;

  PlannerActionServerBase(const PlannerActionServerBase &) = delete;
  PlannerActionServerBase & operator=(const PlannerActionServerBase &) = delete;

protected:
  virtual void compute_path(
    const Goal & goal,
    const octomap_msgs::msg::Octomap & latest_map,
    std::function<void(const Feedback &)> publish_feedback,
    Result & result) = 0;

  rclcpp::Node * node() { return node_; }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  void on_map(const octomap_msgs::msg::Octomap::SharedPtr msg);

  rclcpp::Node * node_;
  std::string action_name_;
  rclcpp_action::Server<RequestPath>::SharedPtr action_server_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_sub_;

  std::mutex map_mutex_;
  std::optional<octomap_msgs::msg::Octomap> latest_map_;
};

}  // namespace pathfinder_core
