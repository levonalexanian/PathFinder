#pragma once

#include <cstddef>
#include <vector>

namespace pathfinder_core
{

template <typename T>
std::vector<T> stride_decimate(const std::vector<T> & items, std::size_t max_count)
{
  if (items.size() <= max_count || max_count == 0) {
    return items;
  }
  std::vector<T> out;
  out.reserve(max_count);
  const std::size_t stride = items.size() / max_count;
  for (std::size_t i = 0; i < items.size(); i += stride) {
    out.push_back(items[i]);
  }
  while (out.size() > max_count) {
    out.pop_back();
  }
  return out;
}

}  // namespace pathfinder_core
