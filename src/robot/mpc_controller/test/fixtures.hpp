#pragma once
#include "mpc_controller/scene.hpp"

inline void circleTrack(mpc::Reference & reference, double width=2.)
{
  std::vector<local_planning::Point> points;
  std::vector<local_planning::TrackWidth> widths;
  for (int i = 0; i < 160; ++i) {
    const double a = -std::acos(-1.)/2+i*2*std::acos(-1.)/160;
    points.emplace_back(10*std::cos(a), 10+10*std::sin(a), 1.);
    widths.push_back({width, width});
  }
  reference.set(points, widths);
}

inline mpc::Grid freeGrid()
{
  mpc::Grid g;
  g.width = 140; g.height = 100; g.resolution = .1; g.origin = {-3,-3,0};
  g.cells.assign(g.width*g.height, 0);
  return g;
}
