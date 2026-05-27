#include <cstdio>
#include <string>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include "pathfinder_core/demo_world_geometry.hpp"

namespace
{

void fill_box(
  octomap::OcTree & tree,
  double x0, double y0, double z0,
  double x1, double y1, double z1,
  double res)
{
  for (double x = x0; x <= x1; x += res) {
    for (double y = y0; y <= y1; y += res) {
      for (double z = z0; z <= z1; z += res) {
        tree.updateNode(octomap::point3d(x, y, z), true);
      }
    }
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  using namespace pathfinder_core::demo_world;

  const std::string out_path =
    (argc > 1) ? std::string(argv[1]) : std::string("maps/demo_world.bt");

  octomap::OcTree tree(kMapResolution);

  fill_box(tree, 0.0, 0.0, kGroundOctomapZ0, kWorldXY, kWorldXY, kGroundOctomapZ1, kMapResolution);

  for (const auto & p : kPostCenters) {
    fill_box(
      tree,
      p[0] - kPostSize / 2.0, p[1] - kPostSize / 2.0, kPostOctomapZStart,
      p[0] + kPostSize / 2.0, p[1] + kPostSize / 2.0, kPostHeight,
      kMapResolution);
  }

  for (const auto & oh : kOverhangs) {
    fill_box(tree, oh.x0, oh.y0, kOverhangZ0, oh.x1, oh.y1, kOverhangZ1, kMapResolution);
  }

  fill_box(tree, kCeilBX0, kCeilBY0, kCeilBZ0, kCeilBX1, kCeilBY1, kCeilBZ1, kMapResolution);

  // Invisible ceiling at z=7: raises the planning grid's z-extent above the 5 m
  // posts so the drone can reach a free cell directly over a post cap and land
  // on top. Octomap-only by design — not mirrored in the visualizer or Gazebo
  // world (a full roof would obscure the scene; the drone never flies near z=7).
  fill_box(tree, 0.0, 0.0, kTopCeilingZ0, kWorldXY, kWorldXY, kTopCeilingZ1, kMapResolution);

  tree.updateInnerOccupancy();

  if (!tree.writeBinary(out_path)) {
    std::fprintf(stderr, "failed to write %s\n", out_path.c_str());
    return 1;
  }
  std::printf(
    "wrote %s (%zu nodes, resolution=%.3f)\n",
    out_path.c_str(), tree.size(), tree.getResolution());
  return 0;
}
