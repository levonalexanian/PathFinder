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

namespace pathfinder_drone
{

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
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("base_frame", "base_link");

    k_p_ = get_parameter("k_p").as_double();
    max_speed_ = get_parameter("max_speed").as_double();
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
      "path_follower up: k_p=%.2f max_speed=%.2f tol=%.2f rate=%.1fHz map=%s base=%s",
      k_p_, max_speed_, waypoint_tolerance_, rate,
      map_frame_.c_str(), base_frame_.c_str());
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
      }
      RCLCPP_INFO(
        get_logger(),
        "reached waypoint %zu (dist=%.2f m)%s",
        idx, dist, finished ? "; path complete" : "");
      publish_stop();
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

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = vx;
    cmd.linear.y = vy;
    cmd.linear.z = vz;
    cmd_pub_->publish(cmd);
  }

  void publish_stop()
  {
    geometry_msgs::msg::Twist cmd;
    cmd_pub_->publish(cmd);
  }

  double k_p_{1.0};
  double max_speed_{1.5};
  double waypoint_tolerance_{0.2};
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

}  // namespace pathfinder_drone

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pathfinder_drone::PathFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
