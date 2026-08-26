#pragma once
#include <string>
#include <vector>

namespace mppi {

// Euclidean distance (m) from each free cell to the nearest occupied cell,
// built from a ROS map yaml + PGM (InferEnv.load_wall_distance_field).
struct WallSdf {
  int h = 0, w = 0;
  float ox = 0.f, oy = 0.f, res = 0.05f;
  std::vector<float> data;  // [h][w], row 0 = bottom (flipud of the image)

  bool valid() const { return !data.empty(); }
  static WallSdf load(const std::string& map_yaml);
  // InferEnv.sample_wall_distance: nearest cell, 0 outside the map, 100 if no SDF.
  float sample(float x, float y) const;
};

}  // namespace mppi
