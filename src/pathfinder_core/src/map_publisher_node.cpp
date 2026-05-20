#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

using namespace std::chrono_literals;

class MapPublisherNode : public rclcpp::Node
{
public:
  MapPublisherNode()
  : rclcpp::Node("map_publisher")
  {
    declare_parameter<std::string>("map_file", "");
    declare_parameter<std::string>("map_frame", "map");

    map_file_ = get_parameter("map_file").as_string();
    map_frame_ = get_parameter("map_frame").as_string();

    if (map_file_.empty()) {
      RCLCPP_ERROR(get_logger(), "parameter 'map_file' is required");
      throw std::runtime_error("map_file parameter not set");
    }

    tree_ = std::make_unique<octomap::OcTree>(map_file_);
    if (!tree_ || tree_->size() == 0) {
      RCLCPP_ERROR(
        get_logger(), "failed to load octomap from %s",
        map_file_.c_str());
      throw std::runtime_error("octomap load failed");
    }
    RCLCPP_INFO(
      get_logger(),
      "loaded octomap from %s (%zu nodes, resolution=%.3f)",
      map_file_.c_str(), tree_->size(), tree_->getResolution());

    publisher_ = create_publisher<octomap_msgs::msg::Octomap>(
      "/voxel_map", rclcpp::QoS(1).reliable().transient_local());

    timer_ = create_wall_timer(
      1s, std::bind(&MapPublisherNode::publish_once, this));
    publish_once();
  }

private:
  void publish_once()
  {
    octomap_msgs::msg::Octomap msg;
    if (!octomap_msgs::binaryMapToMsg(*tree_, msg)) {
      RCLCPP_ERROR(get_logger(), "failed to serialize octomap");
      return;
    }
    msg.header.frame_id = map_frame_;
    msg.header.stamp = now();
    publisher_->publish(msg);
  }

  std::string map_file_;
  std::string map_frame_;
  std::unique_ptr<octomap::OcTree> tree_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MapPublisherNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("map_publisher"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
