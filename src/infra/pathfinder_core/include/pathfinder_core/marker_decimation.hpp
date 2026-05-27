#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace pathfinder_core
{

// Maximum number of voxels emitted per feedback frame for search visualization.
// Caps MarkerArray point counts so Foxglove stays interactive during planning.
constexpr std::size_t kMaxVizVoxels = 500;

// Returns up to max_count indices i where flags[i] != 0, sampled uniformly by
// stride so the output is spread across the whole flag vector.
inline std::vector<std::size_t> sample_flag_indices(
  const std::vector<std::uint8_t> & flags,
  std::size_t max_count)
{
  std::vector<std::size_t> out;
  if (max_count == 0) {
    return out;
  }
  std::size_t count = 0;
  for (auto f : flags) {
    if (f) {
      ++count;
    }
  }
  if (count == 0) {
    return out;
  }
  const std::size_t stride = std::max<std::size_t>(1, count / max_count);
  out.reserve(std::min(count, max_count));
  std::size_t seen = 0;
  for (std::size_t i = 0; i < flags.size(); ++i) {
    if (!flags[i]) {
      continue;
    }
    if ((seen++ % stride) == 0) {
      out.push_back(i);
    }
    if (out.size() >= max_count) {
      break;
    }
  }
  return out;
}

}  // namespace pathfinder_core
