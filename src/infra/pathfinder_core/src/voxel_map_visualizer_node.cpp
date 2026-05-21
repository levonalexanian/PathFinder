#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace pathfinder_core
{

class VoxelMapVisualizerNode : public rclcpp::Node
{
public:
  VoxelMapVisualizerNode()
  : rclcpp::Node("voxel_map_visualizer")
  {
    const auto qos = rclcpp::QoS(1).reliable().transient_local();

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/voxel_map_cloud", qos);

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

    std::vector<std::array<float, 3>> points;
    points.reserve(tree->size());
    for (auto it = tree->begin_leafs(); it != tree->end_leafs(); ++it) {
      if (tree->isNodeOccupied(*it)) {
        points.push_back({
          static_cast<float>(it.getX()),
          static_cast<float>(it.getY()),
          static_cast<float>(it.getZ()),
        });
      }
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = msg->header;
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    for (const auto & p : points) {
      *iter_x = p[0];
      *iter_y = p[1];
      *iter_z = p[2];
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }

    pub_->publish(cloud);
    RCLCPP_INFO(
      get_logger(),
      "republished /voxel_map as /voxel_map_cloud (%zu occupied voxels, resolution=%.3f)",
      points.size(), tree->getResolution());
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
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
