#include <algorithm>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace pathfinder_core
{

namespace
{

std_msgs::msg::ColorRGBA color_for_height(double z, double z_min, double z_max)
{
  std_msgs::msg::ColorRGBA c;
  c.a = 0.85f;
  const double t = (z_max > z_min) ? std::clamp((z - z_min) / (z_max - z_min), 0.0, 1.0) : 0.5;
  // 4-stop heatmap: blue → cyan → yellow → red
  if (t < 1.0 / 3.0) {
    const double s = t * 3.0;
    c.r = 0.0f;
    c.g = static_cast<float>(s);
    c.b = 1.0f;
  } else if (t < 2.0 / 3.0) {
    const double s = (t - 1.0 / 3.0) * 3.0;
    c.r = static_cast<float>(s);
    c.g = 1.0f;
    c.b = static_cast<float>(1.0 - s);
  } else {
    const double s = (t - 2.0 / 3.0) * 3.0;
    c.r = 1.0f;
    c.g = static_cast<float>(1.0 - s);
    c.b = 0.0f;
  }
  return c;
}

}  // namespace

class VoxelMapVisualizerNode : public rclcpp::Node
{
public:
  VoxelMapVisualizerNode()
  : rclcpp::Node("voxel_map_visualizer")
  {
    const auto qos = rclcpp::QoS(1).reliable().transient_local();

    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/voxel_map_markers", qos);

    sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      "/voxel_map", qos,
      std::bind(&VoxelMapVisualizerNode::on_map, this, std::placeholders::_1));
  }

private:
  void on_map(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    std::unique_ptr<octomap::AbstractOcTree> abstract_tree{
      octomap_msgs::msgToMap(*msg)};
    if (!abstract_tree) {
      RCLCPP_WARN(get_logger(), "failed to deserialize /voxel_map");
      return;
    }
    auto * tree = dynamic_cast<octomap::OcTree *>(abstract_tree.get());
    if (tree == nullptr) {
      RCLCPP_WARN(get_logger(), "/voxel_map payload is not an OcTree");
      return;
    }

    // Expand pruned internal nodes so every leaf has size == resolution.
    // Without this, leaves at coarser depth render as undersized cubes with visible gaps.
    tree->expand();

    const double res = tree->getResolution();

    visualization_msgs::msg::Marker marker;
    marker.header = msg->header;
    marker.ns = "voxel_map";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = res;
    marker.scale.y = res;
    marker.scale.z = res;
    marker.frame_locked = true;

    double z_min = std::numeric_limits<double>::infinity();
    double z_max = -std::numeric_limits<double>::infinity();
    for (auto it = tree->begin_leafs(); it != tree->end_leafs(); ++it) {
      if (tree->isNodeOccupied(*it)) {
        z_min = std::min(z_min, it.getZ());
        z_max = std::max(z_max, it.getZ());
      }
    }

    marker.points.reserve(tree->size());
    marker.colors.reserve(tree->size());
    for (auto it = tree->begin_leafs(); it != tree->end_leafs(); ++it) {
      if (tree->isNodeOccupied(*it)) {
        geometry_msgs::msg::Point p;
        p.x = it.getX();
        p.y = it.getY();
        p.z = it.getZ();
        marker.points.push_back(p);
        marker.colors.push_back(color_for_height(p.z, z_min, z_max));
      }
    }

    visualization_msgs::msg::MarkerArray arr;
    arr.markers.push_back(std::move(marker));
    pub_->publish(arr);

    RCLCPP_INFO(
      get_logger(),
      "republished /voxel_map as /voxel_map_markers (%zu occupied voxels, resolution=%.3f)",
      arr.markers.front().points.size(), res);
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_;
};

}  // namespace pathfinder_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pathfinder_core::VoxelMapVisualizerNode>());
  rclcpp::shutdown();
  return 0;
}
