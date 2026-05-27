#include <cstdio>
#include <string>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

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
  const std::string out_path =
    (argc > 1) ? std::string(argv[1]) : std::string("maps/demo_world.bt");

  const double res = 0.1;
  octomap::OcTree tree(res);

  fill_box(tree, 0.0, 0.0, 0.0, 10.0, 10.0, 0.1, res);

  const double post_size = 0.5;
  const double post_height = 5.0;
  const std::pair<double, double> posts[3] = {{2.0, 2.0}, {5.0, 7.0}, {8.0, 3.0}};
  for (const auto & p : posts) {
    fill_box(
      tree,
      p.first - post_size / 2.0, p.second - post_size / 2.0, 0.1,
      p.first + post_size / 2.0, p.second + post_size / 2.0, post_height,
      res);
  }

  fill_box(tree, 3.0, 4.0, 2.0, 5.0, 6.0, 2.5, res);
  fill_box(tree, 6.5, 1.0, 2.0, 8.5, 3.0, 2.5, res);

  fill_box(tree, 0.5, 0.5, 1.0, 2.5, 2.5, 1.2, res);

  // Invisible ceiling at z=7: raises the planning grid's z-extent above the 5 m
  // posts so the drone can reach a free cell directly over a post cap and land
  // on top. Octomap-only by design — not mirrored in the visualizer or Gazebo
  // world (a full roof would obscure the scene; the drone never flies near z=7).
  fill_box(tree, 0.0, 0.0, 6.9, 10.0, 10.0, 7.0, res);

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
