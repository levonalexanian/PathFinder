#include <chrono>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "pathfinder_core/demo_world_geometry.hpp"
#include "pathfinder_core/search_viz.hpp"

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
const std::vector<Box> kObstacles = {
  // Ground (top at z=0, slab 5cm thick extending down to z=-0.05). The car
  // spawns with wheel bottoms exactly at z=0 so the contact patch sits cleanly
  // on the slab surface with no settling.
  {
    demo_world::kGroundVizCX, demo_world::kGroundVizCY, demo_world::kGroundVizCZ,
    demo_world::kGroundVizSX, demo_world::kGroundVizSY, demo_world::kGroundVizSZ,
    demo_world::kGroundR, demo_world::kGroundG, demo_world::kGroundB,
  },
  // 3 posts (rooted at z=0, extend to z=5.0). The octomap actually has them
  // start at z=0.1 (above the ground voxels), but rendering them from z=0
  // makes them look planted in the floor instead of floating above it.
  {
    demo_world::kPostCenters[0][0], demo_world::kPostCenters[0][1], demo_world::kPostVizCZ,
    demo_world::kPostSize, demo_world::kPostSize, demo_world::kPostVizSZ,
    demo_world::kPostR, demo_world::kPostG, demo_world::kPostB,
  },
  {
    demo_world::kPostCenters[1][0], demo_world::kPostCenters[1][1], demo_world::kPostVizCZ,
    demo_world::kPostSize, demo_world::kPostSize, demo_world::kPostVizSZ,
    demo_world::kPostR, demo_world::kPostG, demo_world::kPostB,
  },
  {
    demo_world::kPostCenters[2][0], demo_world::kPostCenters[2][1], demo_world::kPostVizCZ,
    demo_world::kPostSize, demo_world::kPostSize, demo_world::kPostVizSZ,
    demo_world::kPostR, demo_world::kPostG, demo_world::kPostB,
  },
  // 2 overhangs (z=2.0..2.5, 2x2 footprint)
  {
    demo_world::kOverhangs[0].cx, demo_world::kOverhangs[0].cy, demo_world::kOverhangVizCZ,
    demo_world::kOverhangSX, demo_world::kOverhangSY, demo_world::kOverhangSZ,
    demo_world::kOverR, demo_world::kOverG, demo_world::kOverB,
  },
  {
    demo_world::kOverhangs[1].cx, demo_world::kOverhangs[1].cy, demo_world::kOverhangVizCZ,
    demo_world::kOverhangSX, demo_world::kOverhangSY, demo_world::kOverhangSZ,
    demo_world::kOverR, demo_world::kOverG, demo_world::kOverB,
  },
  // Low ceiling (z=1.0..1.2, 2x2 footprint)
  {
    demo_world::kCeilVizCX, demo_world::kCeilVizCY, demo_world::kCeilVizCZ,
    demo_world::kCeilVizSX, demo_world::kCeilVizSY, demo_world::kCeilVizSZ,
    demo_world::kCeilR, demo_world::kCeilG, demo_world::kCeilB,
  },
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
      viz::fill_marker_header(m, now(), id++, "voxel_map");
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.pose.position.x = box.cx;
      m.pose.position.y = box.cy;
      m.pose.position.z = box.cz;
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
