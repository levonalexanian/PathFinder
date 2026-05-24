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

// Demo world geometry — primitive boxes for clean rendering (no z-fighting from
// 15k overlapping voxel cubes). MUST match generate_demo_map.cpp and the gazebo
// world SDFs *spatially*; visual thicknesses can differ slightly for aesthetics.
// Color palette: muted distinct hues per obstacle type so they're identifiable
// at a glance without being garish.
constexpr float kGroundR = 0.28f, kGroundG = 0.32f, kGroundB = 0.40f;   // dark slate
constexpr float kPostR   = 0.85f, kPostG   = 0.55f, kPostB   = 0.20f;   // warm amber
constexpr float kOverR   = 0.20f, kOverG   = 0.62f, kOverB   = 0.70f;   // teal
constexpr float kCeilR   = 0.58f, kCeilG   = 0.42f, kCeilB   = 0.75f;   // muted purple

const std::vector<Box> kObstacles = {
  // Ground (top at z=0.03, ~2cm below the car's wheel bottoms at z=0.05). The
  // small gap prevents z-fighting between wheel and ground faces and gives the
  // car a natural-looking ground clearance.
  {5.0, 5.0, 0.015, 10.0, 10.0, 0.03, kGroundR, kGroundG, kGroundB},
  // 3 posts (rooted at z=0, extend to z=5.0). The octomap actually has them
  // start at z=0.1 (above the ground voxels), but rendering them from z=0
  // makes them look planted in the floor instead of floating above it.
  {2.0, 2.0, 2.5, 0.5, 0.5, 5.0, kPostR, kPostG, kPostB},
  {5.0, 7.0, 2.5, 0.5, 0.5, 5.0, kPostR, kPostG, kPostB},
  {8.0, 3.0, 2.5, 0.5, 0.5, 5.0, kPostR, kPostG, kPostB},
  // 2 overhangs (z=2.0..2.5, 2x2 footprint)
  {4.0, 5.0, 2.25, 2.0, 2.0, 0.5, kOverR, kOverG, kOverB},
  {7.5, 2.0, 2.25, 2.0, 2.0, 0.5, kOverR, kOverG, kOverB},
  // Low ceiling (z=1.0..1.2, 2x2 footprint)
  {1.5, 1.5, 1.1, 2.0, 2.0, 0.2, kCeilR, kCeilG, kCeilB},
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
