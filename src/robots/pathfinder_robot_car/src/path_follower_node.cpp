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
    declare_parameter<double>("max_linear_vel_mps", 1.0);
    declare_parameter<double>("max_angular_vel_radps", 2.0);

    k_linear_ = get_parameter("k_linear").as_double();
    k_angular_ = get_parameter("k_angular").as_double();
    max_linear_vel_mps_ = get_parameter("max_linear_vel_mps").as_double();
    max_angular_vel_radps_ = get_parameter("max_angular_vel_radps").as_double();

    const Config cfg = declare_and_load_base_config();
    init_follower(cfg);

    RCLCPP_INFO(
      get_logger(),
      "diff-drive path_follower up: k_lin=%.2f k_ang=%.2f vmax=%.2f wmax=%.2f tol=%.2f rate=%.1fHz",
      k_linear_, k_angular_, max_linear_vel_mps_, max_angular_vel_radps_,
      cfg.waypoint_tolerance, cfg.update_rate);
  }

protected:
  geometry_msgs::msg::Twist compute_command(
    const geometry_msgs::msg::Pose & current,
    const geometry_msgs::msg::Pose & target) override
  {
    const double dx = target.position.x - current.position.x;
    const double dy = target.position.y - current.position.y;
    const double dist_2d_m = std::hypot(dx, dy);

    const double yaw = yaw_from_quat(
      current.orientation.x,
      current.orientation.y,
      current.orientation.z,
      current.orientation.w);

    const double heading_to_target = std::atan2(dy, dx);
    const double heading_err_rad = wrap_to_pi(heading_to_target - yaw);

    const double cmd_angular_z_radps = clip(
      k_angular_ * heading_err_rad, -max_angular_vel_radps_, max_angular_vel_radps_);
    // project forward speed onto heading to reduce linear vel during sharp turns
    const double cmd_linear_x_mps = clip(
      k_linear_ * dist_2d_m * std::cos(heading_err_rad), 0.0, max_linear_vel_mps_);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = cmd_linear_x_mps;
    cmd.angular.z = cmd_angular_z_radps;
    return cmd;
  }

private:
  double k_linear_;
  double k_angular_;
  double max_linear_vel_mps_;
  double max_angular_vel_radps_;
};

}  // namespace pathfinder_robot_car

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pathfinder_robot_car::PathFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
