#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <queue>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/logger.hpp>

#include <pathfinder_core/voxel_grid.hpp>

namespace pathfinder_algo_dstar_lite
{

struct DstarLiteParams
{
  double robot_radius{0.25};
  double max_plan_time_sec{5.0};
  int feedback_every_nodes{200};
  double feedback_every_seconds{0.2};
  int viz_delay_ms{0};
};

struct DstarLiteFeedback
{
  int nodes_explored{0};
  std::vector<pathfinder_core::VoxelIndex> best_partial_path;
  double best_cost_so_far{0.0};
  // Newly-discovered cubes for visualization (deltas since last feedback).
  std::vector<std::size_t> sampled_open_flat;
  std::vector<std::size_t> sampled_closed_flat;
};

struct DstarLiteResult
{
  bool success{false};
  std::string message;
  std::vector<pathfinder_core::VoxelIndex> path;
  int nodes_expanded{0};
  double plan_duration_ms{0.0};
};

class DstarLiteCore
{
public:
  using FeedbackCallback = std::function<void(const DstarLiteFeedback &)>;

  explicit DstarLiteCore(const rclcpp::Logger & logger);

  DstarLiteCore(const DstarLiteCore &) = delete;
  DstarLiteCore & operator=(const DstarLiteCore &) = delete;

  // Plans from `start` to `goal`. If the goal matches the previous goal AND
  // the grid is unchanged, performs incremental replan (only updating k_m if
  // the start moved); otherwise resets state and replans from scratch.
  DstarLiteResult plan(
    const pathfinder_core::InflatedVoxelGrid & grid,
    const pathfinder_core::VoxelIndex & start,
    const pathfinder_core::VoxelIndex & goal,
    const DstarLiteParams & params,
    FeedbackCallback feedback);

  // Force a full state reset (forget previous goal & maintained data).
  void reset();

private:
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

  void initialize_search(
    const pathfinder_core::InflatedVoxelGrid & grid,
    std::size_t start_flat,
    std::size_t goal_flat);
  void shift_start(
    const pathfinder_core::InflatedVoxelGrid & grid,
    std::size_t new_start_flat);
  void update_vertex(
    const pathfinder_core::InflatedVoxelGrid & grid, std::size_t flat);
  ComputeStats compute_shortest_path(
    const pathfinder_core::InflatedVoxelGrid & grid,
    const DstarLiteParams & params,
    const FeedbackCallback & feedback);

  std::vector<std::size_t> neighbors(
    const pathfinder_core::InflatedVoxelGrid & grid, std::size_t flat) const;
  double heuristic(
    const pathfinder_core::InflatedVoxelGrid & grid,
    std::size_t a, std::size_t b) const;
  double edge_cost(
    const pathfinder_core::InflatedVoxelGrid & grid,
    std::size_t a, std::size_t b) const;
  Key calculate_key(
    const pathfinder_core::InflatedVoxelGrid & grid, std::size_t flat) const;

  std::vector<pathfinder_core::VoxelIndex> extract_path(
    const pathfinder_core::InflatedVoxelGrid & grid,
    std::size_t max_steps) const;

  std::vector<std::size_t> sample_open_flats(std::size_t max_count);
  std::vector<std::size_t> sample_locked_flats(std::size_t max_count);

  rclcpp::Logger logger_;

  std::unordered_set<std::size_t> emitted_open_;
  std::unordered_set<std::size_t> emitted_closed_;

  // Persistent D* Lite state across plan() calls.
  std::vector<double> g_;
  std::vector<double> rhs_;
  std::vector<std::uint8_t> in_queue_;
  std::vector<Key> queue_key_;
  OpenQueue open_;
  double k_m_{0.0};
  std::size_t s_start_flat_{0};
  std::size_t s_goal_flat_{0};
  double resolution_{0.0};

  // Identity of the grid backing the persistent state. If a different grid
  // pointer or different dims arrive on the next plan(), we drop state.
  const pathfinder_core::InflatedVoxelGrid * last_grid_{nullptr};
  std::size_t last_grid_cells_{0};

  std::optional<std::size_t> last_start_flat_;
  std::optional<std::size_t> last_goal_flat_;
  bool initialized_{false};

  static constexpr double kInf = std::numeric_limits<double>::infinity();
};

}  // namespace pathfinder_algo_dstar_lite
