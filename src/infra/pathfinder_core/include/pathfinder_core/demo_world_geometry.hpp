#pragma once

#include <array>
#include <cstddef>

namespace pathfinder_core
{
namespace demo_world
{

// ---------------------------------------------------------------------------
// Coordinate / size constants shared by generate_demo_map.cpp and
// voxel_map_visualizer_node.cpp.  Where the two files intentionally differ
// for cosmetic reasons, BOTH values are provided as separate named constants
// (see "Visualizer vs. octomap" notes below).
// ---------------------------------------------------------------------------

// World bounds — the map covers a 10 m × 10 m × 7 m volume.
constexpr double kWorldXY = 10.0;
constexpr double kMapResolution = 0.1;

// --- Ground ----------------------------------------------------------------
// Octomap: a 10×10×0.1 m slab with top face at z=0.1 (one voxel layer).
// Visualizer: a 10×10×0.05 m slab centered at z=-0.025 so the top face is
// exactly z=0, where car wheels touch, hiding the slab beneath the floor.
constexpr double kGroundXY = 10.0;
// octomap: fill_box(0,0,0 -> 10,10,0.1)
constexpr double kGroundOctomapZ0 = 0.0;
constexpr double kGroundOctomapZ1 = 0.1;
// visualizer: center and half-thickness
constexpr double kGroundVizCX = 5.0, kGroundVizCY = 5.0, kGroundVizCZ = -0.025;
constexpr double kGroundVizSX = 10.0, kGroundVizSY = 10.0, kGroundVizSZ = 0.05;

// --- Posts (3 vertical pillars) --------------------------------------------
// Size: 0.5 × 0.5 m footprint, 5 m tall.
// Octomap: z starts at 0.1 (above ground voxel layer).
// Visualizer: z starts at 0.0 so posts appear planted in the floor.
constexpr double kPostSize = 0.5;
constexpr double kPostHeight = 5.0;
constexpr double kPostOctomapZStart = 0.1;
constexpr double kPostVizZStart = 0.0;

// Centers (cx, cy) of the three posts.
constexpr std::array<std::array<double, 2>, 3> kPostCenters = {{
  {2.0, 2.0},
  {5.0, 7.0},
  {8.0, 3.0},
}};

// Visualizer center z and half-height for the posts.
constexpr double kPostVizCZ = 2.5;   // (0 + 5.0) / 2
constexpr double kPostVizSZ = 5.0;

// --- Overhangs (2 horizontal slabs) ----------------------------------------
// Each is a 2 × 2 × 0.5 m box spanning z=2.0..2.5.
// No cosmetic deviation between octomap and visualizer for overhangs.
struct OverhangDef
{
  double cx, cy;     // horizontal center
  double x0, y0;    // octomap fill start (corner)
  double x1, y1;    // octomap fill end   (corner)
};
constexpr std::array<OverhangDef, 2> kOverhangs = {{
  {4.0, 5.0,  3.0, 4.0,  5.0, 6.0},
  {7.5, 2.0,  6.5, 1.0,  8.5, 3.0},
}};
constexpr double kOverhangZ0 = 2.0, kOverhangZ1 = 2.5;
constexpr double kOverhangVizCZ = 2.25;   // (2.0 + 2.5) / 2
constexpr double kOverhangSX = 2.0, kOverhangSY = 2.0, kOverhangSZ = 0.5;

// --- Low ceiling block -----------------------------------------------------
// A 2 × 2 × 0.2 m block at z=1.0..1.2.
// No cosmetic deviation.
constexpr double kCeilBX0 = 0.5, kCeilBY0 = 0.5;
constexpr double kCeilBX1 = 2.5, kCeilBY1 = 2.5;
constexpr double kCeilBZ0 = 1.0, kCeilBZ1 = 1.2;
// Visualizer: center + size
constexpr double kCeilVizCX = 1.5, kCeilVizCY = 1.5, kCeilVizCZ = 1.1;
constexpr double kCeilVizSX = 2.0, kCeilVizSY = 2.0, kCeilVizSZ = 0.2;

// --- Invisible ceiling at z=7 (octomap only) --------------------------------
// Raises the planning grid's z-extent above the 5 m posts so the drone can
// plan a path directly over a post cap.  Not mirrored in the visualizer or
// Gazebo (a full roof would obscure the scene and the drone never flies near
// z=7).
constexpr double kTopCeilingZ0 = 6.9, kTopCeilingZ1 = 7.0;

// ---------------------------------------------------------------------------
// Visualizer color palette — muted distinct hues, identifiable at a glance.
// ---------------------------------------------------------------------------
constexpr float kGroundR = 0.28f, kGroundG = 0.32f, kGroundB = 0.40f;  // dark slate
constexpr float kPostR   = 0.85f, kPostG   = 0.55f, kPostB   = 0.20f;  // warm amber
constexpr float kOverR   = 0.20f, kOverG   = 0.62f, kOverB   = 0.70f;  // teal
constexpr float kCeilR   = 0.58f, kCeilG   = 0.42f, kCeilB   = 0.75f;  // muted purple

}  // namespace demo_world
}  // namespace pathfinder_core
