#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace pathfinder_core
{

class PathFollowerBase : public rclcpp::Node
{
public:
  struct Config
  {
    double update_rate{20.0};
    double waypoint_tolerance{0.15};
    std::string map_frame{"map"};
    std::string base_frame{"base_link"};
    std::string path_topic{"/planned_path"};
    std::string cmd_topic{"/cmd_vel"};
  };

  ~PathFollowerBase() override = default;

  PathFollowerBase(const PathFollowerBase &) = delete;
  PathFollowerBase & operator=(const PathFollowerBase &) = delete;

protected:
  explicit PathFollowerBase(const std::string & node_name)
  : rclcpp::Node(node_name),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
  }

  Config declare_and_load_base_config(double default_waypoint_tolerance = 0.15)
  {
    declare_parameter<double>("waypoint_tolerance", default_waypoint_tolerance);
    declare_parameter<double>("update_rate", 20.0);
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("base_frame", "base_link");

    Config cfg;
    cfg.waypoint_tolerance = get_parameter("waypoint_tolerance").as_double();
    cfg.update_rate = get_parameter("update_rate").as_double();
    cfg.map_frame = get_parameter("map_frame").as_string();
    cfg.base_frame = get_parameter("base_frame").as_string();
    return cfg;
  }

  void init_follower(const Config & cfg)
  {
    cfg_ = cfg;

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      cfg_.path_topic, rclcpp::QoS(10),
      std::bind(&PathFollowerBase::on_path_internal, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      cfg_.cmd_topic, rclcpp::QoS(10));

    const auto period =
      std::chrono::duration<double>(1.0 / std::max(1.0, cfg_.update_rate));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PathFollowerBase::on_tick_internal, this));
  }

  // Pure-virtual: produce a velocity command for the given current/target pose.
  virtual geometry_msgs::msg::Twist compute_command(
    const geometry_msgs::msg::Pose & current,
    const geometry_msgs::msg::Pose & target) = 0;

  // Distance metric used to decide waypoint advance. Default = 2D xy distance.
  virtual double distance_to_waypoint(
    const geometry_msgs::msg::Pose & current,
    const geometry_msgs::msg::Pose & target)
  {
    const double dx = target.position.x - current.position.x;
    const double dy = target.position.y - current.position.y;
    return std::hypot(dx, dy);
  }

  // Called from on_path() before the path is stored. Subclasses can reset
  // additional state here.
  virtual void on_path_reset() {}

  // Hook called at the start of every tick. Returning true short-circuits the
  // rest of the tick (no TF lookup, no compute_command).
  virtual bool tick_prologue() { return false; }

  // Called when there is no path or all waypoints have been consumed.
  virtual void on_no_path() { publish_stop(); }

  // Called when the TF lookup for map->base fails.
  virtual void on_tf_failure() { publish_stop(); }

  // Called when the current waypoint is reached. finished == true when no
  // more waypoints remain.
  virtual void on_waypoint_reached(std::size_t /*idx*/, bool /*finished*/)
  {
    publish_stop();
  }

  // Publishes a Twist via cmd_pub_. Subclasses can override (e.g. filter).
  virtual void publish_twist(const geometry_msgs::msg::Twist & cmd)
  {
    cmd_pub_->publish(cmd);
  }

  void publish_stop()
  {
    geometry_msgs::msg::Twist zero;
    publish_twist(zero);
  }

  const Config & config() const { return cfg_; }

  // Inline math helpers (match signatures used by car/drone today).
  static inline double yaw_from_quat(double x, double y, double z, double w)
  {
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static inline double wrap_to_pi(double a)
  {
    while (a > M_PI) {
      a -= 2.0 * M_PI;
    }
    while (a < -M_PI) {
      a += 2.0 * M_PI;
    }
    return a;
  }

  static inline double clip(double v, double lo, double hi)
  {
    return std::max(lo, std::min(hi, v));
  }

private:
  void on_path_internal(const nav_msgs::msg::Path::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      waypoints_ = msg->poses;
      current_idx_ = 0;
    }
    on_path_reset();
    RCLCPP_INFO(
      get_logger(), "received path with %zu waypoint(s)", msg->poses.size());
  }

  void on_tick_internal()
  {
    if (tick_prologue()) {
      return;
    }

    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    std::size_t idx;
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      waypoints = waypoints_;
      idx = current_idx_;
    }

    if (waypoints.empty() || idx >= waypoints.size()) {
      on_no_path();
      return;
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(
        cfg_.map_frame, cfg_.base_frame, tf2::TimePointZero);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "no %s->%s transform yet: %s",
        cfg_.map_frame.c_str(), cfg_.base_frame.c_str(), e.what());
      on_tf_failure();
      return;
    }

    geometry_msgs::msg::Pose current;
    current.position.x = tf.transform.translation.x;
    current.position.y = tf.transform.translation.y;
    current.position.z = tf.transform.translation.z;
    current.orientation = tf.transform.rotation;

    const auto & target = waypoints[idx].pose;
    const double dist = distance_to_waypoint(current, target);

    if (dist < cfg_.waypoint_tolerance) {
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
      on_waypoint_reached(idx, finished);
      return;
    }

    const auto cmd = compute_command(current, target);
    publish_twist(cmd);
  }

  Config cfg_;

  std::mutex state_mutex_;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  std::size_t current_idx_{0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace pathfinder_core
