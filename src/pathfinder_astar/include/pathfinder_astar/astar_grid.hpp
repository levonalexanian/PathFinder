#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <vector>

#include <pathfinder_core/voxel_grid.hpp>

namespace pathfinder_astar
{

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

inline std::vector<pathfinder_core::VoxelIndex> reconstruct_path(
  const SearchBuffers & buf,
  const pathfinder_core::InflatedVoxelGrid & grid,
  std::size_t goal_flat)
{
  std::vector<pathfinder_core::VoxelIndex> path;
  std::size_t cur = goal_flat;
  while (true) {
    path.push_back(grid.from_linear_index(cur));
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
