// Dense uint8 occupancy grid with true Euclidean-ball inflation. Both choices
// are load-bearing: the dense layout gives ~12x throughput over a hash map
// (Dijkstra benchmark) and Euclidean balls preserve A*'s configuration-space
// accuracy vs. cheaper Chebyshev cubes.

#include "pathfinder_core/voxel_grid.hpp"

#include <algorithm>
#include <cmath>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

namespace pathfinder_core
{

InflatedVoxelGrid::InflatedVoxelGrid(
  std::array<int, 3> dims,
  std::array<double, 3> min_bound,
  double resolution,
  std::vector<std::uint8_t> occupied)
: occupied_(std::move(occupied)),
  dims_(dims),
  min_(min_bound),
  resolution_(resolution)
{
  max_[0] = min_[0] + dims_[0] * resolution_;
  max_[1] = min_[1] + dims_[1] * resolution_;
  max_[2] = min_[2] + dims_[2] * resolution_;
}

InflatedVoxelGrid::InflatedVoxelGrid(const octomap::OcTree & tree, double robot_radius)
{
  resolution_ = tree.getResolution();
  if (resolution_ <= 0.0) {
    return;
  }

  double min_x, min_y, min_z, max_x, max_y, max_z;
  tree.getMetricMin(min_x, min_y, min_z);
  tree.getMetricMax(max_x, max_y, max_z);

  const double res = resolution_;
  min_[0] = std::floor(min_x / res) * res;
  min_[1] = std::floor(min_y / res) * res;
  min_[2] = std::floor(min_z / res) * res;

  dims_[0] = std::max(1, static_cast<int>(std::ceil((max_x - min_[0]) / res)) + 1);
  dims_[1] = std::max(1, static_cast<int>(std::ceil((max_y - min_[1]) / res)) + 1);
  dims_[2] = std::max(1, static_cast<int>(std::ceil((max_z - min_[2]) / res)) + 1);

  max_[0] = min_[0] + dims_[0] * res;
  max_[1] = min_[1] + dims_[1] * res;
  max_[2] = min_[2] + dims_[2] * res;

  const std::size_t n =
    static_cast<std::size_t>(dims_[0]) *
    static_cast<std::size_t>(dims_[1]) *
    static_cast<std::size_t>(dims_[2]);
  occupied_.assign(n, 0);

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
    const int cx = static_cast<int>(std::floor((center.x() - min_[0]) / res));
    const int cy = static_cast<int>(std::floor((center.y() - min_[1]) / res));
    const int cz = static_cast<int>(std::floor((center.z() - min_[2]) / res));
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
    const int x1 = std::min(dims_[0] - 1, hi_x + inflate);
    const int y1 = std::min(dims_[1] - 1, hi_y + inflate);
    const int z1 = std::min(dims_[2] - 1, hi_z + inflate);

    for (int z = z0; z <= z1; ++z) {
      const int dz = (z < lo_z) ? (lo_z - z) : (z > hi_z ? z - hi_z : 0);
      for (int y = y0; y <= y1; ++y) {
        const int dy = (y < lo_y) ? (lo_y - y) : (y > hi_y ? y - hi_y : 0);
        for (int x = x0; x <= x1; ++x) {
          const int dx = (x < lo_x) ? (lo_x - x) : (x > hi_x ? x - hi_x : 0);
          if (inflate == 0 || dx * dx + dy * dy + dz * dz <= inflate_sq) {
            occupied_[linear_index({x, y, z})] = 1;
          }
        }
      }
    }
  }
}

bool InflatedVoxelGrid::is_segment_free(
  double x0, double y0, double z0,
  double x1, double y1, double z1) const noexcept
{
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double dz = z1 - z0;
  const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
  const double step = std::max(resolution_ * 0.5, 1e-3);
  if (dist < 1e-9) {
    return !is_occupied_at_point(x0, y0, z0);
  }
  const int steps = std::max(1, static_cast<int>(std::ceil(dist / step)));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    const double px = x0 + dx * t;
    const double py = y0 + dy * t;
    const double pz = z0 + dz * t;
    if (is_occupied_at_point(px, py, pz)) {
      return false;
    }
  }
  return true;
}

std::vector<VoxelIndex> InflatedVoxelGrid::neighbors_26(const VoxelIndex & idx) const
{
  std::vector<VoxelIndex> out;
  out.reserve(26);
  for (const auto & d : kNeighborOffsets26) {
    VoxelIndex nb{idx.x + d[0], idx.y + d[1], idx.z + d[2]};
    if (in_bounds(nb)) {
      out.push_back(nb);
    }
  }
  return out;
}

std::optional<VoxelIndex> InflatedVoxelGrid::nudge_to_free(
  const VoxelIndex & idx, int max_radius) const
{
  if (in_bounds(idx) && !is_occupied(idx)) {
    return idx;
  }
  for (int r = 1; r <= max_radius; ++r) {
    for (int dz = -r; dz <= r; ++dz) {
      for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
          if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != r) {
            continue;
          }
          VoxelIndex nb{idx.x + dx, idx.y + dy, idx.z + dz};
          if (in_bounds(nb) && !is_occupied(nb)) {
            return nb;
          }
        }
      }
    }
  }
  return std::nullopt;
}

}  // namespace pathfinder_core
