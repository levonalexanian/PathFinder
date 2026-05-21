#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <pathfinder_core/path_follower_base.hpp>

namespace pathfinder_robot_car
{

class PathFollowerNode : public pathfinder_core::PathFollowerBase
{
public:
  PathFollowerNode()
  : pathfinder_core::PathFollowerBase("path_follower")
  {
    declare_parameter<double>("k_linear", 1.0);
    declare_parameter<double>("k_angular", 2.0);
    declare_parameter<double>("max_linear", 0.5);
    declare_parameter<double>("max_angular", 1.5);
    declare_parameter<double>("waypoint_tolerance", 0.15);
    declare_parameter<double>("update_rate", 20.0);
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("base_frame", "base_link");

    k_linear_ = get_parameter("k_linear").as_double();
    k_angular_ = get_parameter("k_angular").as_double();
    max_linear_ = get_parameter("max_linear").as_double();
    max_angular_ = get_parameter("max_angular").as_double();
    const double waypoint_tolerance = get_parameter("waypoint_tolerance").as_double();
    const double rate = get_parameter("update_rate").as_double();

    Config cfg;
    cfg.update_rate = rate;
    cfg.waypoint_tolerance = waypoint_tolerance;
    cfg.map_frame = get_parameter("map_frame").as_string();
    cfg.base_frame = get_parameter("base_frame").as_string();
    init_follower(cfg);

    RCLCPP_INFO(
      get_logger(),
      "diff-drive path_follower up: k_lin=%.2f k_ang=%.2f vmax=%.2f wmax=%.2f tol=%.2f rate=%.1fHz",
      k_linear_, k_angular_, max_linear_, max_angular_, waypoint_tolerance, rate);
  }

protected:
  geometry_msgs::msg::Twist compute_command(
    const geometry_msgs::msg::Pose & current,
    const geometry_msgs::msg::Pose & target) override
  {
    const double dx = target.position.x - current.position.x;
    const double dy = target.position.y - current.position.y;
    const double dist = std::hypot(dx, dy);

    const double yaw = yaw_from_quat(
      current.orientation.x,
      current.orientation.y,
      current.orientation.z,
      current.orientation.w);

    const double heading_to_target = std::atan2(dy, dx);
    const double heading_err = wrap_to_pi(heading_to_target - yaw);

    const double angular_z = clip(
      k_angular_ * heading_err, -max_angular_, max_angular_);
    const double linear_x = clip(
      k_linear_ * dist * std::cos(heading_err), 0.0, max_linear_);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_x;
    cmd.angular.z = angular_z;
    return cmd;
  }

private:
  double k_linear_{1.0};
  double k_angular_{2.0};
  double max_linear_{0.5};
  double max_angular_{1.5};
};

}  // namespace pathfinder_robot_car

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pathfinder_robot_car::PathFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
