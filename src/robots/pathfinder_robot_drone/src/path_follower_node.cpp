#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <pathfinder_core/path_follower_base.hpp>

namespace pathfinder_robot_drone
{

class PathFollowerNode : public pathfinder_core::PathFollowerBase
{
public:
  PathFollowerNode()
  : pathfinder_core::PathFollowerBase("path_follower")
  {
    declare_parameter<double>("k_linear", 1.0);
    declare_parameter<double>("max_speed", 1.5);
    declare_parameter<double>("goal_dwell_seconds", 1.5);
    declare_parameter<double>("k_yaw", 1.5);
    declare_parameter<double>("max_angular_vel_radps", 1.5);
    declare_parameter<double>("velocity_tau", 0.3);

    k_linear_ = get_parameter("k_linear").as_double();
    max_speed_ = get_parameter("max_speed").as_double();
    goal_dwell_seconds_ = get_parameter("goal_dwell_seconds").as_double();
    k_yaw_ = get_parameter("k_yaw").as_double();
    max_angular_vel_radps_ = get_parameter("max_angular_vel_radps").as_double();
    velocity_tau_ = get_parameter("velocity_tau").as_double();

    const Config cfg = declare_and_load_base_config(0.2);
    update_rate_ = cfg.update_rate;
    init_follower(cfg);

    RCLCPP_INFO(
      get_logger(),
      "path_follower up: k_linear=%.2f max_speed=%.2f waypoint_tol_m=%.2f rate=%.1fHz hold=%.2fs k_yaw=%.2f wmax=%.2f tau=%.2fs map=%s base=%s",
      k_linear_, max_speed_, cfg.waypoint_tolerance, update_rate_, goal_dwell_seconds_,
      k_yaw_, max_angular_vel_radps_, velocity_tau_,
      cfg.map_frame.c_str(), cfg.base_frame.c_str());
  }

protected:
  void on_path_reset() override
  {
    std::lock_guard<std::mutex> lk(hold_mutex_);
    hold_end_time_.reset();
    filtered_cmd_ = geometry_msgs::msg::Twist{};
  }

  bool tick_prologue() override
  {
    std::lock_guard<std::mutex> lk(hold_mutex_);
    if (!hold_end_time_.has_value()) {
      return false;
    }
    if (now() < hold_end_time_.value()) {
      publish_stop();
    } else {
      hold_end_time_.reset();
    }
    return true;
  }

  void on_no_path() override {}

  double distance_to_waypoint(
    const geometry_msgs::msg::Pose & current,
    const geometry_msgs::msg::Pose & target) override
  {
    const double dx = target.position.x - current.position.x;
    const double dy = target.position.y - current.position.y;
    const double dz = target.position.z - current.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  void on_waypoint_reached(std::size_t /*idx*/, bool finished) override
  {
    if (finished) {
      std::lock_guard<std::mutex> lk(hold_mutex_);
      hold_end_time_ = now() + rclcpp::Duration::from_seconds(goal_dwell_seconds_);
    }
    publish_stop();
  }

  geometry_msgs::msg::Twist compute_command(
    const geometry_msgs::msg::Pose & current,
    const geometry_msgs::msg::Pose & target) override
  {
    const double dx = target.position.x - current.position.x;
    const double dy = target.position.y - current.position.y;
    const double dz = target.position.z - current.position.z;

    double vx = k_linear_ * dx;
    double vy = k_linear_ * dy;
    double vz = k_linear_ * dz;
    const double desired_speed_mps = std::sqrt(vx * vx + vy * vy + vz * vz);
    if (desired_speed_mps > max_speed_) {
      const double scale = max_speed_ / desired_speed_mps;
      vx *= scale;
      vy *= scale;
      vz *= scale;
    }

    const double current_yaw = yaw_from_quat(
      current.orientation.x,
      current.orientation.y,
      current.orientation.z,
      current.orientation.w);
    const double desired_yaw = std::atan2(dy, dx);
    const double yaw_err = wrap_to_pi(desired_yaw - current_yaw);
    const double angular_z = clip(k_yaw_ * yaw_err, -max_angular_vel_radps_, max_angular_vel_radps_);

    const double cos_yaw = std::cos(current_yaw);
    const double sin_yaw = std::sin(current_yaw);
    const double body_vx = cos_yaw * vx + sin_yaw * vy;
    const double body_vy = -sin_yaw * vx + cos_yaw * vy;

    geometry_msgs::msg::Twist desired;
    desired.linear.x = body_vx;
    desired.linear.y = body_vy;
    desired.linear.z = vz;
    desired.angular.z = angular_z;
    return desired;
  }

  void publish_twist(const geometry_msgs::msg::Twist & desired) override
  {
    const double dt = 1.0 / std::max(1.0, update_rate_);
    const double alpha = dt / (velocity_tau_ + dt);
    filtered_cmd_.linear.x = alpha * desired.linear.x + (1.0 - alpha) * filtered_cmd_.linear.x;
    filtered_cmd_.linear.y = alpha * desired.linear.y + (1.0 - alpha) * filtered_cmd_.linear.y;
    filtered_cmd_.linear.z = alpha * desired.linear.z + (1.0 - alpha) * filtered_cmd_.linear.z;
    filtered_cmd_.angular.z = alpha * desired.angular.z + (1.0 - alpha) * filtered_cmd_.angular.z;
    pathfinder_core::PathFollowerBase::publish_twist(filtered_cmd_);
  }

private:
  double k_linear_{1.0};
  double max_speed_{1.5};
  double update_rate_{20.0};
  double goal_dwell_seconds_{1.5};
  double k_yaw_{1.5};
  double max_angular_vel_radps_{1.5};
  double velocity_tau_{0.3};

  std::mutex hold_mutex_;
  std::optional<rclcpp::Time> hold_end_time_;
  geometry_msgs::msg::Twist filtered_cmd_;
};

}  // namespace pathfinder_robot_drone

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pathfinder_robot_drone::PathFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
