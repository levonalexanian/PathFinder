#pragma once

#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "pathfinder_core/voxel_grid.hpp"

namespace pathfinder_core
{
namespace viz
{

// Search visualization markers expire after 3 s; documented in README.
inline const rclcpp::Duration kSearchMarkerLifetime =
  rclcpp::Duration::from_seconds(3.0);

// Cube side length as a fraction of voxel resolution for the open/closed sets.
constexpr double kVoxelCubeFraction = 0.6;

// LINE_STRIP width as a fraction of voxel resolution for the best-path marker.
constexpr double kVoxelLineFraction = 0.3;

// Standard RGBA for the open set (yellow, semi-transparent).
constexpr float kOpenSetR = 1.0f, kOpenSetG = 1.0f, kOpenSetB = 0.0f, kOpenSetA = 0.35f;

// Standard RGBA for the closed/locked set (grey, very transparent).
constexpr float kClosedSetR = 0.5f, kClosedSetG = 0.5f, kClosedSetB = 0.5f, kClosedSetA = 0.2f;

// Standard RGBA for the current best path (green, nearly opaque).
constexpr float kBestPathR = 0.0f, kBestPathG = 1.0f, kBestPathB = 0.0f, kBestPathA = 0.9f;

// Initialise the 6 invariant fields every search-visualization marker shares.
// Callers set type, scale, color, and points after calling this.
inline void fill_marker_header(
  visualization_msgs::msg::Marker & m,
  const rclcpp::Time & stamp,
  int id,
  const std::string & ns,
  const std::string & frame_id = "map")
{
  m.header.frame_id = frame_id;
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = id;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.w = 1.0;
}

// Build an empty CUBE_LIST marker. The caller fills marker.points afterward.
inline visualization_msgs::msg::Marker make_cube_list_marker(
  const rclcpp::Time & stamp,
  const std::string & ns,
  int id,
  double scale,
  float r, float g, float b, float a,
  const std::string & frame_id = "map")
{
  visualization_msgs::msg::Marker m;
  fill_marker_header(m, stamp, id, ns, frame_id);
  m.type = visualization_msgs::msg::Marker::CUBE_LIST;
  m.scale.x = scale;
  m.scale.y = scale;
  m.scale.z = scale;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
  m.lifetime = kSearchMarkerLifetime;
  return m;
}

// Build an empty LINE_STRIP marker. The caller fills marker.points afterward.
inline visualization_msgs::msg::Marker make_line_strip_marker(
  const rclcpp::Time & stamp,
  const std::string & ns,
  int id,
  double width,
  float r, float g, float b, float a,
  const std::string & frame_id = "map")
{
  visualization_msgs::msg::Marker m;
  fill_marker_header(m, stamp, id, ns, frame_id);
  m.type = visualization_msgs::msg::Marker::LINE_STRIP;
  m.scale.x = width;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
  m.lifetime = kSearchMarkerLifetime;
  return m;
}

// Convert a flat linear index to a world-space Point via the voxel grid.
inline geometry_msgs::msg::Point flat_to_point(
  const InflatedVoxelGrid & grid, std::size_t flat)
{
  const auto v = grid.from_linear_index(flat);
  const auto w = grid.voxel_to_world(v);
  geometry_msgs::msg::Point p;
  p.x = w[0];
  p.y = w[1];
  p.z = w[2];
  return p;
}

// Convert a VoxelIndex to a world-space Point via the voxel grid.
inline geometry_msgs::msg::Point voxel_to_point(
  const InflatedVoxelGrid & grid, const VoxelIndex & v)
{
  const auto w = grid.voxel_to_world(v);
  geometry_msgs::msg::Point p;
  p.x = w[0];
  p.y = w[1];
  p.z = w[2];
  return p;
}

// Build a PoseStamped from a world (x,y,z) triple and a pre-built header.
inline geometry_msgs::msg::PoseStamped make_pose_stamped(
  double x, double y, double z,
  const std_msgs::msg::Header & header)
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header = header;
  ps.pose.position.x = x;
  ps.pose.position.y = y;
  ps.pose.position.z = z;
  ps.pose.orientation.w = 1.0;
  return ps;
}

// Convert a voxel path to a nav_msgs::Path with the given header.
inline nav_msgs::msg::Path voxels_to_path(
  const InflatedVoxelGrid & grid,
  const std::vector<VoxelIndex> & voxels,
  const std_msgs::msg::Header & header)
{
  nav_msgs::msg::Path path;
  path.header = header;
  path.poses.reserve(voxels.size());
  for (const auto & v : voxels) {
    const auto w = grid.voxel_to_world(v);
    path.poses.push_back(make_pose_stamped(w[0], w[1], w[2], header));
  }
  return path;
}

}  // namespace viz
}  // namespace pathfinder_core
