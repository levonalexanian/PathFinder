#pragma once

#include <rclcpp/rclcpp.hpp>

namespace pathfinder_core
{

struct BasePlannerParams
{
  double robot_radius{0.25};
  double max_plan_time_sec{5.0};
  int viz_delay_ms{0};
};

inline void declare_base_planner_params(rclcpp::Node * node)
{
  node->declare_parameter<double>("robot_radius", 0.25);
  node->declare_parameter<double>("max_plan_time_sec", 5.0);
  node->declare_parameter<int>("viz_delay_ms", 0);
}

inline void load_base_planner_params(rclcpp::Node * node, BasePlannerParams & p)
{
  p.robot_radius = node->get_parameter("robot_radius").as_double();
  p.max_plan_time_sec = node->get_parameter("max_plan_time_sec").as_double();
  p.viz_delay_ms = static_cast<int>(node->get_parameter("viz_delay_ms").as_int());
}

}  // namespace pathfinder_core
