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
    declare_parameter<double>("k_p", 1.0);
    declare_parameter<double>("max_speed", 1.5);
    declare_parameter<double>("waypoint_tolerance", 0.2);
    declare_parameter<double>("update_rate", 20.0);
    declare_parameter<double>("hold_down_seconds", 1.5);
    declare_parameter<double>("k_yaw", 1.5);
    declare_parameter<double>("max_angular", 1.5);
    declare_parameter<double>("velocity_tau", 0.3);
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("base_frame", "base_link");

    k_p_ = get_parameter("k_p").as_double();
    max_speed_ = get_parameter("max_speed").as_double();
    update_rate_ = get_parameter("update_rate").as_double();
    hold_down_seconds_ = get_parameter("hold_down_seconds").as_double();
    k_yaw_ = get_parameter("k_yaw").as_double();
    max_angular_ = get_parameter("max_angular").as_double();
    velocity_tau_ = get_parameter("velocity_tau").as_double();
    const double waypoint_tolerance = get_parameter("waypoint_tolerance").as_double();

    Config cfg;
    cfg.update_rate = update_rate_;
    cfg.waypoint_tolerance = waypoint_tolerance;
    cfg.map_frame = get_parameter("map_frame").as_string();
    cfg.base_frame = get_parameter("base_frame").as_string();
    init_follower(cfg);

    RCLCPP_INFO(
      get_logger(),
      "path_follower up: k_p=%.2f max_speed=%.2f tol=%.2f rate=%.1fHz hold=%.2fs k_yaw=%.2f wmax=%.2f tau=%.2fs map=%s base=%s",
      k_p_, max_speed_, waypoint_tolerance, update_rate_, hold_down_seconds_,
      k_yaw_, max_angular_, velocity_tau_,
      cfg.map_frame.c_str(), cfg.base_frame.c_str());
  }

protected:
  void on_path_reset() override
  {
    std::lock_guard<std::mutex> lk(drone_mutex_);
    hold_down_end_time_.reset();
    filtered_ = geometry_msgs::msg::Twist{};
  }

  bool tick_prologue() override
  {
    std::optional<rclcpp::Time> hold_end;
    {
      std::lock_guard<std::mutex> lk(drone_mutex_);
      hold_end = hold_down_end_time_;
    }

    if (hold_end.has_value()) {
      if (now() < hold_end.value()) {
        publish_stop();
      } else {
        std::lock_guard<std::mutex> lk(drone_mutex_);
        hold_down_end_time_.reset();
      }
      return true;
    }
    return false;
  }

  void on_no_path() override
  {
    // Drone intentionally does not publish anything when there is no path.
  }

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
      std::lock_guard<std::mutex> lk(drone_mutex_);
      hold_down_end_time_ = now() + rclcpp::Duration::from_seconds(hold_down_seconds_);
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

    double vx = k_p_ * dx;
    double vy = k_p_ * dy;
    double vz = k_p_ * dz;
    const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
    if (speed > max_speed_) {
      const double scale = max_speed_ / speed;
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
    const double wz = clip(k_yaw_ * yaw_err, -max_angular_, max_angular_);

    const double cos_yaw = std::cos(current_yaw);
    const double sin_yaw = std::sin(current_yaw);
    const double body_vx = cos_yaw * vx + sin_yaw * vy;
    const double body_vy = -sin_yaw * vx + cos_yaw * vy;

    geometry_msgs::msg::Twist desired;
    desired.linear.x = body_vx;
    desired.linear.y = body_vy;
    desired.linear.z = vz;
    desired.angular.z = wz;
    return desired;
  }

  void publish_twist(const geometry_msgs::msg::Twist & desired) override
  {
    const double dt = 1.0 / std::max(1.0, update_rate_);
    const double alpha = dt / (velocity_tau_ + dt);
    filtered_.linear.x = alpha * desired.linear.x + (1.0 - alpha) * filtered_.linear.x;
    filtered_.linear.y = alpha * desired.linear.y + (1.0 - alpha) * filtered_.linear.y;
    filtered_.linear.z = alpha * desired.linear.z + (1.0 - alpha) * filtered_.linear.z;
    filtered_.angular.z = alpha * desired.angular.z + (1.0 - alpha) * filtered_.angular.z;
    pathfinder_core::PathFollowerBase::publish_twist(filtered_);
  }

private:
  double k_p_{1.0};
  double max_speed_{1.5};
  double update_rate_{20.0};
  double hold_down_seconds_{1.5};
  double k_yaw_{1.5};
  double max_angular_{1.5};
  double velocity_tau_{0.3};

  std::mutex drone_mutex_;
  std::optional<rclcpp::Time> hold_down_end_time_;
  geometry_msgs::msg::Twist filtered_;
};

}  // namespace pathfinder_robot_drone

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pathfinder_robot_drone::PathFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
