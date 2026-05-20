#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include <pathfinder_core/voxel_grid.hpp>

namespace octomap
{
class OcTree;
}

namespace pathfinder_algo_dstar_lite
{

struct Key
{
  double k1;
  double k2;

  bool operator<(const Key & other) const noexcept
  {
    if (k1 != other.k1) {
      return k1 < other.k1;
    }
    return k2 < other.k2;
  }
  bool operator>(const Key & other) const noexcept { return other < *this; }
  bool operator==(const Key & other) const noexcept
  {
    return k1 == other.k1 && k2 == other.k2;
  }
  bool operator!=(const Key & other) const noexcept { return !(*this == other); }
};

struct QueueEntry
{
  Key key;
  std::size_t flat;

  bool operator>(const QueueEntry & other) const noexcept { return other.key < key; }
};

using OpenQueue = std::priority_queue<
  QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>>;

struct ComputeStats
{
  std::int32_t expansions{0};
  bool reached_start{false};
  bool timed_out{false};
};

using FeedbackHook = std::function<void(std::int32_t expansions)>;

class DstarLiteGrid
{
public:
  DstarLiteGrid(
    std::unique_ptr<octomap::OcTree> tree,
    double robot_radius,
    std::uint64_t map_signature);

  DstarLiteGrid(const DstarLiteGrid &) = delete;
  DstarLiteGrid & operator=(const DstarLiteGrid &) = delete;

  const pathfinder_core::InflatedVoxelGrid & grid() const { return grid_; }
  std::uint64_t map_signature() const { return map_signature_; }
  std::size_t s_start_flat() const { return s_start_flat_; }
  std::size_t s_goal_flat() const { return s_goal_flat_; }
  double k_m() const { return k_m_; }
  double g(std::size_t flat) const { return g_[flat]; }
  double rhs(std::size_t flat) const { return rhs_[flat]; }

  void initialize(std::size_t start_flat, std::size_t goal_flat);

  // k_m offset compensates for moved start without re-keying entire queue
  void shift_start(std::size_t new_start_flat);

  void update_vertex(std::size_t flat);

  ComputeStats compute_shortest_path(
    double max_time_sec,
    std::int32_t feedback_every_nodes,
    double feedback_every_seconds,
    const FeedbackHook & on_feedback);

  // Reconstructs path by greedily descending min(g + c) successors from
  // s_start. Returns an empty vector if start is unreachable.
  std::vector<pathfinder_core::VoxelIndex> extract_path(std::size_t max_steps) const;

  std::vector<std::size_t> neighbors(std::size_t flat) const;

  double heuristic(std::size_t a, std::size_t b) const;
  double edge_cost(std::size_t a, std::size_t b) const;

  Key calculate_key(std::size_t flat) const;
  bool in_queue(std::size_t flat) const { return in_queue_[flat] != 0; }
  std::size_t queue_size() const { return open_.size(); }

  std::vector<std::size_t> sample_open_flats(std::size_t max_count) const;
  std::vector<std::size_t> sample_locked_flats(std::size_t max_count) const;

private:
  std::unique_ptr<octomap::OcTree> tree_;
  pathfinder_core::InflatedVoxelGrid grid_;
  std::uint64_t map_signature_{0};

  std::vector<double> g_;
  std::vector<double> rhs_;
  std::vector<std::uint8_t> in_queue_;
  std::vector<Key> queue_key_;

  OpenQueue open_;
  double k_m_{0.0};
  std::size_t s_start_flat_{0};
  std::size_t s_goal_flat_{0};
  double resolution_{0.0};

  static constexpr double kInf = std::numeric_limits<double>::infinity();
};

}  // namespace pathfinder_algo_dstar_lite
