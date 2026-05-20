#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace pathfinder_robot_car
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
    waypoint_tolerance_ = get_parameter("waypoint_tolerance").as_double();
    const double rate = get_parameter("update_rate").as_double();
    map_frame_ = get_parameter("map_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/planned_path", rclcpp::QoS(10),
      std::bind(&PathFollowerNode::on_path, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10));

    const auto period =
      std::chrono::duration<double>(1.0 / std::max(1.0, rate));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PathFollowerNode::on_tick, this));

    RCLCPP_INFO(
      get_logger(),
      "diff-drive path_follower up: k_lin=%.2f k_ang=%.2f vmax=%.2f wmax=%.2f tol=%.2f rate=%.1fHz",
      k_linear_, k_angular_, max_linear_, max_angular_, waypoint_tolerance_, rate);
  }

private:
  void on_path(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    waypoints_ = msg->poses;
    current_idx_ = 0;
    RCLCPP_INFO(
      get_logger(), "received path with %zu waypoint(s)", waypoints_.size());
  }

  void on_tick()
  {
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    std::size_t idx;
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      waypoints = waypoints_;
      idx = current_idx_;
    }

    if (waypoints.empty() || idx >= waypoints.size()) {
      publish_stop();
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
      publish_stop();
      return;
    }

    const double x = tf.transform.translation.x;
    const double y = tf.transform.translation.y;
    const double yaw = yaw_from_quat(
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z,
      tf.transform.rotation.w);

    const auto & target = waypoints[idx].pose.position;
    const double dx = target.x - x;
    const double dy = target.y - y;
    const double dist = std::hypot(dx, dy);

    if (dist < waypoint_tolerance_) {
      bool finished = false;
      {
        std::lock_guard<std::mutex> lk(state_mutex_);
        ++current_idx_;
        finished = current_idx_ >= waypoints_.size();
      }
      RCLCPP_INFO(
        get_logger(),
        "reached waypoint %zu (dist=%.2f m)%s",
        idx, dist, finished ? "; path complete" : "");
      publish_stop();
      return;
    }

    const double heading_to_target = std::atan2(dy, dx);
    const double heading_err = wrap_to_pi(heading_to_target - yaw);

    const double angular_z = clip(
      k_angular_ * heading_err, -max_angular_, max_angular_);
    const double linear_x = clip(
      k_linear_ * dist * std::cos(heading_err), 0.0, max_linear_);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_x;
    cmd.angular.z = angular_z;
    cmd_pub_->publish(cmd);
  }

  void publish_stop()
  {
    geometry_msgs::msg::Twist cmd;
    cmd_pub_->publish(cmd);
  }

  double k_linear_{1.0};
  double k_angular_{2.0};
  double max_linear_{0.5};
  double max_angular_{1.5};
  double waypoint_tolerance_{0.15};
  std::string map_frame_;
  std::string base_frame_;

  std::mutex state_mutex_;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  std::size_t current_idx_{0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace pathfinder_robot_car

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pathfinder_robot_car::PathFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
