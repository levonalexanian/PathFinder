#pragma once

#include <memory>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>

namespace pathfinder_core
{

inline std::unique_ptr<octomap::OcTree> octree_from_msg(
  const octomap_msgs::msg::Octomap & msg)
{
  std::unique_ptr<octomap::AbstractOcTree> abstract{octomap_msgs::msgToMap(msg)};
  if (!abstract) {
    return nullptr;
  }
  auto * raw = dynamic_cast<octomap::OcTree *>(abstract.get());
  if (raw == nullptr) {
    return nullptr;
  }
  abstract.release();
  return std::unique_ptr<octomap::OcTree>(raw);
}

}  // namespace pathfinder_core
