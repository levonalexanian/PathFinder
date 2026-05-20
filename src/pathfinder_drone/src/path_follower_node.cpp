#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace pathfinder_drone
{

namespace
{

double yaw_from_quat(double x, double y, double z, double w)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double wrap_to_pi(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

double clip(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

}  // namespace

class PathFollowerNode : public rclcpp::Node
{
public:
  PathFollowerNode()
  : rclcpp::Node("path_follower"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
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
    waypoint_tolerance_ = get_parameter("waypoint_tolerance").as_double();
    update_rate_ = get_parameter("update_rate").as_double();
    hold_down_seconds_ = get_parameter("hold_down_seconds").as_double();
    k_yaw_ = get_parameter("k_yaw").as_double();
    max_angular_ = get_parameter("max_angular").as_double();
    velocity_tau_ = get_parameter("velocity_tau").as_double();
    map_frame_ = get_parameter("map_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/planned_path", rclcpp::QoS(10),
      std::bind(&PathFollowerNode::on_path, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10));

    const auto period =
      std::chrono::duration<double>(1.0 / std::max(1.0, update_rate_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PathFollowerNode::on_tick, this));

    RCLCPP_INFO(
      get_logger(),
      "path_follower up: k_p=%.2f max_speed=%.2f tol=%.2f rate=%.1fHz hold=%.2fs k_yaw=%.2f wmax=%.2f tau=%.2fs map=%s base=%s",
      k_p_, max_speed_, waypoint_tolerance_, update_rate_, hold_down_seconds_,
      k_yaw_, max_angular_, velocity_tau_,
      map_frame_.c_str(), base_frame_.c_str());
  }

private:
  void on_path(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    waypoints_ = msg->poses;
    current_idx_ = 0;
    hold_down_end_time_.reset();
    filtered_ = geometry_msgs::msg::Twist{};
    RCLCPP_INFO(
      get_logger(), "received path with %zu waypoint(s)", waypoints_.size());
  }

  void on_tick()
  {
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    std::size_t idx;
    std::optional<rclcpp::Time> hold_end;
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      waypoints = waypoints_;
      idx = current_idx_;
      hold_end = hold_down_end_time_;
    }

    if (hold_end.has_value()) {
      if (now() < hold_end.value()) {
        publish_zero();
      } else {
        std::lock_guard<std::mutex> lk(state_mutex_);
        hold_down_end_time_.reset();
      }
      return;
    }

    if (waypoints.empty() || idx >= waypoints.size()) {
      return;
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "no %s->%s transform yet: %s",
        map_frame_.c_str(), base_frame_.c_str(), e.what());
      publish_zero();
      return;
    }

    const auto & target = waypoints[idx].pose.position;
    const double dx = target.x - tf.transform.translation.x;
    const double dy = target.y - tf.transform.translation.y;
    const double dz = target.z - tf.transform.translation.z;
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (dist < waypoint_tolerance_) {
      bool finished = false;
      {
        std::lock_guard<std::mutex> lk(state_mutex_);
        ++current_idx_;
        finished = current_idx_ >= waypoints_.size();
        if (finished) {
          hold_down_end_time_ = now() + rclcpp::Duration::from_seconds(hold_down_seconds_);
        }
      }
      RCLCPP_INFO(
        get_logger(),
        "reached waypoint %zu (dist=%.2f m)%s",
        idx, dist, finished ? "; path complete" : "");
      publish_zero();
      return;
    }

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
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z,
      tf.transform.rotation.w);
    const double desired_yaw = std::atan2(dy, dx);
    const double yaw_err = wrap_to_pi(desired_yaw - current_yaw);
    const double wz = clip(k_yaw_ * yaw_err, -max_angular_, max_angular_);

    geometry_msgs::msg::Twist desired;
    desired.linear.x = vx;
    desired.linear.y = vy;
    desired.linear.z = vz;
    desired.angular.z = wz;

    publish_filtered(desired);
  }

  void publish_zero()
  {
    geometry_msgs::msg::Twist zero;
    publish_filtered(zero);
  }

  void publish_filtered(const geometry_msgs::msg::Twist & desired)
  {
    const double dt = 1.0 / std::max(1.0, update_rate_);
    const double alpha = dt / (velocity_tau_ + dt);
    filtered_.linear.x = alpha * desired.linear.x + (1.0 - alpha) * filtered_.linear.x;
    filtered_.linear.y = alpha * desired.linear.y + (1.0 - alpha) * filtered_.linear.y;
    filtered_.linear.z = alpha * desired.linear.z + (1.0 - alpha) * filtered_.linear.z;
    filtered_.angular.z = alpha * desired.angular.z + (1.0 - alpha) * filtered_.angular.z;
    cmd_pub_->publish(filtered_);
  }

  double k_p_{1.0};
  double max_speed_{1.5};
  double waypoint_tolerance_{0.2};
  double update_rate_{20.0};
  double hold_down_seconds_{1.5};
  double k_yaw_{1.5};
  double max_angular_{1.5};
  double velocity_tau_{0.3};
  std::string map_frame_;
  std::string base_frame_;

  std::mutex state_mutex_;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  std::size_t current_idx_{0};
  std::optional<rclcpp::Time> hold_down_end_time_;
  geometry_msgs::msg::Twist filtered_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace pathfinder_drone

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pathfinder_drone::PathFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
