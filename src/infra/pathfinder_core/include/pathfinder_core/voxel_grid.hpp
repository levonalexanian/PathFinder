#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace octomap
{
class OcTree;
}

namespace pathfinder_core
{

struct VoxelIndex
{
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const VoxelIndex & other) const noexcept
  {
    return x == other.x && y == other.y && z == other.z;
  }
  bool operator!=(const VoxelIndex & other) const noexcept
  {
    return !(*this == other);
  }
};

// All 26 face/edge/corner neighbor offsets in (dx,dy,dz) form. Exposed so
// callers that iterate neighbors by linear arithmetic (e.g. Dijkstra) can
// share the table instead of maintaining their own copy.
constexpr std::array<std::array<int, 3>, 26> kNeighborOffsets26 = []() {
  std::array<std::array<int, 3>, 26> out{};
  int i = 0;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }
        out[i++] = {dx, dy, dz};
      }
    }
  }
  return out;
}();

// Euclidean distance between two VoxelIndex values scaled to metres.
inline double voxel_distance(
  const VoxelIndex & a, const VoxelIndex & b, double resolution) noexcept
{
  const double dx = static_cast<double>(a.x - b.x);
  const double dy = static_cast<double>(a.y - b.y);
  const double dz = static_cast<double>(a.z - b.z);
  return std::sqrt(dx * dx + dy * dy + dz * dz) * resolution;
}

class InflatedVoxelGrid
{
public:
  InflatedVoxelGrid(const octomap::OcTree & tree, double robot_radius);

  // Raw-data constructor for unit testing: supply dims, min_bound, resolution,
  // and a pre-built occupancy vector (length must equal dims[0]*dims[1]*dims[2]).
  InflatedVoxelGrid(
    std::array<int, 3> dims,
    std::array<double, 3> min_bound,
    double resolution,
    std::vector<std::uint8_t> occupied);

  InflatedVoxelGrid(const InflatedVoxelGrid &) = delete;
  InflatedVoxelGrid & operator=(const InflatedVoxelGrid &) = delete;
  InflatedVoxelGrid(InflatedVoxelGrid &&) = default;
  InflatedVoxelGrid & operator=(InflatedVoxelGrid &&) = default;

  bool in_bounds(const VoxelIndex & idx) const noexcept
  {
    return idx.x >= 0 && idx.y >= 0 && idx.z >= 0 &&
           idx.x < dims_[0] && idx.y < dims_[1] && idx.z < dims_[2];
  }

  bool is_occupied(const VoxelIndex & idx) const noexcept
  {
    if (!in_bounds(idx)) {
      return true;
    }
    return occupied_[linear_index(idx)] != 0;
  }

  bool is_occupied_at_point(double x, double y, double z) const noexcept
  {
    return is_occupied(world_to_voxel(x, y, z));
  }

  bool is_segment_free(
    double x0, double y0, double z0,
    double x1, double y1, double z1) const noexcept;

  VoxelIndex world_to_voxel(double x, double y, double z) const noexcept
  {
    VoxelIndex v;
    v.x = static_cast<int>(std::floor((x - min_[0]) / resolution_));
    v.y = static_cast<int>(std::floor((y - min_[1]) / resolution_));
    v.z = static_cast<int>(std::floor((z - min_[2]) / resolution_));
    return v;
  }

  std::array<double, 3> voxel_to_world(const VoxelIndex & idx) const noexcept
  {
    return {
      min_[0] + (static_cast<double>(idx.x) + 0.5) * resolution_,
      min_[1] + (static_cast<double>(idx.y) + 0.5) * resolution_,
      min_[2] + (static_cast<double>(idx.z) + 0.5) * resolution_,
    };
  }

  std::vector<VoxelIndex> neighbors_26(const VoxelIndex & idx) const;

  std::optional<VoxelIndex> nudge_to_free(
    const VoxelIndex & idx, int max_radius) const;

  std::array<double, 3> min_bound() const noexcept { return min_; }
  std::array<double, 3> max_bound() const noexcept { return max_; }
  std::array<int, 3> dims() const noexcept { return dims_; }
  double resolution() const noexcept { return resolution_; }

  std::size_t linear_index(const VoxelIndex & idx) const noexcept
  {
    return (static_cast<std::size_t>(idx.z) * static_cast<std::size_t>(dims_[1]) +
            static_cast<std::size_t>(idx.y)) *
           static_cast<std::size_t>(dims_[0]) +
           static_cast<std::size_t>(idx.x);
  }

  VoxelIndex from_linear_index(std::size_t lin) const noexcept
  {
    VoxelIndex v;
    const std::size_t nx = static_cast<std::size_t>(dims_[0]);
    const std::size_t ny = static_cast<std::size_t>(dims_[1]);
    v.x = static_cast<int>(lin % nx);
    const std::size_t r = lin / nx;
    v.y = static_cast<int>(r % ny);
    v.z = static_cast<int>(r / ny);
    return v;
  }

  std::size_t cell_count() const noexcept { return occupied_.size(); }

private:
  std::vector<std::uint8_t> occupied_;
  std::array<int, 3> dims_{0, 0, 0};
  std::array<double, 3> min_{0.0, 0.0, 0.0};
  std::array<double, 3> max_{0.0, 0.0, 0.0};
  double resolution_{0.0};
};

}  // namespace pathfinder_core
