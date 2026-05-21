#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace pathfinder_core
{

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
    // 10% overlap so adjacent cubes have no visible seam at any view angle.
    const double cube_scale = res * 1.10;

    visualization_msgs::msg::Marker marker;
    marker.header = msg->header;
    marker.ns = "voxel_map";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = cube_scale;
    marker.scale.y = cube_scale;
    marker.scale.z = cube_scale;
    marker.frame_locked = true;
    marker.color.r = 0.45f;
    marker.color.g = 0.50f;
    marker.color.b = 0.62f;
    marker.color.a = 1.0f;

    marker.points.reserve(tree->size());
    for (auto it = tree->begin_leafs(); it != tree->end_leafs(); ++it) {
      if (tree->isNodeOccupied(*it)) {
        geometry_msgs::msg::Point p;
        p.x = it.getX();
        p.y = it.getY();
        p.z = it.getZ();
        marker.points.push_back(p);
      }
    }

    visualization_msgs::msg::MarkerArray arr;
    arr.markers.push_back(std::move(marker));
    pub_->publish(arr);

    RCLCPP_INFO(
      get_logger(),
      "republished /voxel_map as /voxel_map_markers (%zu occupied voxels, resolution=%.3f, cube_scale=%.3f)",
      arr.markers.front().points.size(), res, cube_scale);
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
