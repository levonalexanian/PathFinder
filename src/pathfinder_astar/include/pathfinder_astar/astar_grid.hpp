#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <vector>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>

namespace pathfinder_astar
{

struct VoxelIndex
{
  int x = 0;
  int y = 0;
  int z = 0;
};

struct GridDims
{
  int nx = 0;
  int ny = 0;
  int nz = 0;

  std::size_t size() const
  {
    return static_cast<std::size_t>(nx) *
           static_cast<std::size_t>(ny) *
           static_cast<std::size_t>(nz);
  }

  bool in_bounds(int x, int y, int z) const
  {
    return x >= 0 && y >= 0 && z >= 0 && x < nx && y < ny && z < nz;
  }

  std::size_t flat(int x, int y, int z) const
  {
    return (static_cast<std::size_t>(z) * static_cast<std::size_t>(ny) +
            static_cast<std::size_t>(y)) *
           static_cast<std::size_t>(nx) +
           static_cast<std::size_t>(x);
  }
};

struct VoxelGrid
{
  GridDims dims;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_z = 0.0;
  std::vector<uint8_t> occupied;

  bool is_occupied(int x, int y, int z) const
  {
    if (!dims.in_bounds(x, y, z)) {
      return true;
    }
    return occupied[dims.flat(x, y, z)] != 0;
  }

  VoxelIndex world_to_voxel(double wx, double wy, double wz) const
  {
    VoxelIndex v;
    v.x = static_cast<int>(std::floor((wx - origin_x) / resolution));
    v.y = static_cast<int>(std::floor((wy - origin_y) / resolution));
    v.z = static_cast<int>(std::floor((wz - origin_z) / resolution));
    return v;
  }

  void voxel_to_world(int x, int y, int z, double & wx, double & wy, double & wz) const
  {
    wx = origin_x + (static_cast<double>(x) + 0.5) * resolution;
    wy = origin_y + (static_cast<double>(y) + 0.5) * resolution;
    wz = origin_z + (static_cast<double>(z) + 0.5) * resolution;
  }
};

inline std::unique_ptr<octomap::OcTree> octree_from_msg(
  const octomap_msgs::msg::Octomap & msg)
{
  std::unique_ptr<octomap::AbstractOcTree> abstract_tree{
    octomap_msgs::msgToMap(msg)};
  if (!abstract_tree) {
    return nullptr;
  }
  auto * raw = dynamic_cast<octomap::OcTree *>(abstract_tree.get());
  if (raw == nullptr) {
    return nullptr;
  }
  abstract_tree.release();
  return std::unique_ptr<octomap::OcTree>(raw);
}

inline VoxelGrid build_inflated_grid(
  const octomap::OcTree & tree,
  double robot_radius)
{
  VoxelGrid grid;
  grid.resolution = tree.getResolution();
  if (grid.resolution <= 0.0) {
    return grid;
  }

  double min_x, min_y, min_z, max_x, max_y, max_z;
  tree.getMetricMin(min_x, min_y, min_z);
  tree.getMetricMax(max_x, max_y, max_z);

  // snap origin to a multiple of resolution so voxel centers are stable
  const double res = grid.resolution;
  grid.origin_x = std::floor(min_x / res) * res;
  grid.origin_y = std::floor(min_y / res) * res;
  grid.origin_z = std::floor(min_z / res) * res;

  grid.dims.nx = std::max(1, static_cast<int>(std::ceil((max_x - grid.origin_x) / res)) + 1);
  grid.dims.ny = std::max(1, static_cast<int>(std::ceil((max_y - grid.origin_y) / res)) + 1);
  grid.dims.nz = std::max(1, static_cast<int>(std::ceil((max_z - grid.origin_z) / res)) + 1);

  grid.occupied.assign(grid.dims.size(), 0);

  const int inflate = std::max(
    0, static_cast<int>(std::ceil(std::max(0.0, robot_radius) / res)));
  const int inflate_sq = inflate * inflate;

  for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
    if (!tree.isNodeOccupied(*it)) {
      continue;
    }
    const double size = it.getSize();
    const int span = std::max(1, static_cast<int>(std::round(size / res)));
    const auto center = it.getCoordinate();
    const int cx = static_cast<int>(std::floor((center.x() - grid.origin_x) / res));
    const int cy = static_cast<int>(std::floor((center.y() - grid.origin_y) / res));
    const int cz = static_cast<int>(std::floor((center.z() - grid.origin_z) / res));
    const int half = span / 2;
    const int lo_x = cx - half;
    const int lo_y = cy - half;
    const int lo_z = cz - half;
    const int hi_x = cx - half + span - 1;
    const int hi_y = cy - half + span - 1;
    const int hi_z = cz - half + span - 1;

    const int x0 = std::max(0, lo_x - inflate);
    const int y0 = std::max(0, lo_y - inflate);
    const int z0 = std::max(0, lo_z - inflate);
    const int x1 = std::min(grid.dims.nx - 1, hi_x + inflate);
    const int y1 = std::min(grid.dims.ny - 1, hi_y + inflate);
    const int z1 = std::min(grid.dims.nz - 1, hi_z + inflate);

    for (int z = z0; z <= z1; ++z) {
      const int dz = (z < lo_z) ? (lo_z - z) : (z > hi_z ? z - hi_z : 0);
      for (int y = y0; y <= y1; ++y) {
        const int dy = (y < lo_y) ? (lo_y - y) : (y > hi_y ? y - hi_y : 0);
        for (int x = x0; x <= x1; ++x) {
          const int dx = (x < lo_x) ? (lo_x - x) : (x > hi_x ? x - hi_x : 0);
          if (inflate == 0 || dx * dx + dy * dy + dz * dz <= inflate_sq) {
            grid.occupied[grid.dims.flat(x, y, z)] = 1;
          }
        }
      }
    }
  }

  return grid;
}

inline std::array<VoxelIndex, 26> neighbor_offsets_26()
{
  std::array<VoxelIndex, 26> out{};
  int idx = 0;
  for (int dz = -1; dz <= 1; ++dz) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }
        out[idx++] = {dx, dy, dz};
      }
    }
  }
  return out;
}

inline double euclidean_voxel_distance(
  int ax, int ay, int az,
  int bx, int by, int bz,
  double resolution)
{
  const double dx = static_cast<double>(ax - bx);
  const double dy = static_cast<double>(ay - by);
  const double dz = static_cast<double>(az - bz);
  return std::sqrt(dx * dx + dy * dy + dz * dz) * resolution;
}

struct OpenEntry
{
  double f = 0.0;
  double h = 0.0;
  std::size_t flat = 0;

  bool operator>(const OpenEntry & other) const
  {
    if (f != other.f) {
      return f > other.f;
    }
    // tie-break on h to prefer goal-directed motion
    return h > other.h;
  }
};

using OpenQueue = std::priority_queue<
  OpenEntry, std::vector<OpenEntry>, std::greater<OpenEntry>>;

struct SearchBuffers
{
  std::vector<double> g_score;
  std::vector<int32_t> came_from;
  std::vector<uint8_t> closed;
  std::vector<uint8_t> in_open;

  void reset(std::size_t n)
  {
    g_score.assign(n, std::numeric_limits<double>::infinity());
    came_from.assign(n, -1);
    closed.assign(n, 0);
    in_open.assign(n, 0);
  }
};

inline VoxelIndex unflatten(std::size_t flat, const GridDims & dims)
{
  VoxelIndex v;
  v.x = static_cast<int>(flat % static_cast<std::size_t>(dims.nx));
  const std::size_t r = flat / static_cast<std::size_t>(dims.nx);
  v.y = static_cast<int>(r % static_cast<std::size_t>(dims.ny));
  v.z = static_cast<int>(r / static_cast<std::size_t>(dims.ny));
  return v;
}

inline std::vector<VoxelIndex> nudge_to_free(
  const VoxelGrid & grid, int x, int y, int z, int max_radius)
{
  std::vector<VoxelIndex> out;
  if (!grid.is_occupied(x, y, z)) {
    out.push_back({x, y, z});
    return out;
  }
  for (int r = 1; r <= max_radius; ++r) {
    for (int dz = -r; dz <= r; ++dz) {
      for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
          if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != r) {
            continue;
          }
          const int nx = x + dx;
          const int ny = y + dy;
          const int nz = z + dz;
          if (grid.dims.in_bounds(nx, ny, nz) && !grid.is_occupied(nx, ny, nz)) {
            out.push_back({nx, ny, nz});
            return out;
          }
        }
      }
    }
  }
  return out;
}

inline std::vector<VoxelIndex> reconstruct_path(
  const SearchBuffers & buf,
  const GridDims & dims,
  std::size_t goal_flat)
{
  std::vector<VoxelIndex> path;
  std::size_t cur = goal_flat;
  while (true) {
    path.push_back(unflatten(cur, dims));
    const int32_t prev = buf.came_from[cur];
    if (prev < 0) {
      break;
    }
    cur = static_cast<std::size_t>(prev);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

}  // namespace pathfinder_astar
