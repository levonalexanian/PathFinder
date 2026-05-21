#include <chrono>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace pathfinder_core
{
namespace
{

struct Box
{
  double cx, cy, cz;  // center
  double sx, sy, sz;  // full size
  float r, g, b;
};

// Demo world geometry — MUST match generate_demo_map.cpp and the gazebo world SDFs.
// Rendering primitives directly (instead of per-voxel cubes) avoids the z-fighting
// stripes you get from 15k overlapping CUBE_LIST faces.
constexpr float kObstacleR = 0.55f;
constexpr float kObstacleG = 0.60f;
constexpr float kObstacleB = 0.70f;
constexpr float kGroundR = 0.30f;
constexpr float kGroundG = 0.33f;
constexpr float kGroundB = 0.40f;

const std::vector<Box> kObstacles = {
  // Ground (z=0..0.1 over the full 10x10 area)
  {5.0, 5.0, 0.05, 10.0, 10.0, 0.1, kGroundR, kGroundG, kGroundB},
  // 3 posts (z=0.1..5.0, 0.5x0.5 cross-section)
  {2.0, 2.0, 2.55, 0.5, 0.5, 4.9, kObstacleR, kObstacleG, kObstacleB},
  {5.0, 7.0, 2.55, 0.5, 0.5, 4.9, kObstacleR, kObstacleG, kObstacleB},
  {8.0, 3.0, 2.55, 0.5, 0.5, 4.9, kObstacleR, kObstacleG, kObstacleB},
  // 2 overhangs (z=2.0..2.5, 2x2 footprint)
  {4.0, 5.0, 2.25, 2.0, 2.0, 0.5, kObstacleR, kObstacleG, kObstacleB},
  {7.5, 2.0, 2.25, 2.0, 2.0, 0.5, kObstacleR, kObstacleG, kObstacleB},
  // Low ceiling (z=1.0..1.2, 2x2 footprint)
  {1.5, 1.5, 1.1, 2.0, 2.0, 0.2, kObstacleR, kObstacleG, kObstacleB},
};

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

    timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() { publish(); });
    publish();
  }

private:
  void publish()
  {
    visualization_msgs::msg::MarkerArray arr;
    arr.markers.reserve(kObstacles.size());
    int id = 0;
    for (const auto & box : kObstacles) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = now();
      m.ns = "voxel_map";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = box.cx;
      m.pose.position.y = box.cy;
      m.pose.position.z = box.cz;
      m.pose.orientation.w = 1.0;
      m.scale.x = box.sx;
      m.scale.y = box.sy;
      m.scale.z = box.sz;
      m.color.r = box.r;
      m.color.g = box.g;
      m.color.b = box.b;
      m.color.a = 1.0f;
      m.frame_locked = true;
      arr.markers.push_back(m);
    }
    pub_->publish(arr);
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace pathfinder_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pathfinder_core::VoxelMapVisualizerNode>());
  rclcpp::shutdown();
  return 0;
}
