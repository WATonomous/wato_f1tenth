#pragma once
#include "mpc_controller/solver.hpp"
#include "local_planning/reference/raceline_reference.hpp"
#include <vector>

namespace mpc {
class Reference {
public:
  void set(const std::vector<local_planning::Point> & points,
    const std::vector<local_planning::TrackWidth> & widths,
    const std::array<std::vector<Pose>, 2> & legal_boundaries = {});
  const local_planning::RacelineReference & geometry() const {return geometry_;}
  uint64_t version() const {return version_;}
  double trackClearance(Pose map) const;
  const std::array<std::vector<Pose>, 2> & edges() const {return edges_;}
private:
  local_planning::RacelineReference geometry_;
  std::array<std::vector<Pose>, 2> edges_;
  uint64_t version_ = 0;
};
struct Grid {
  int width = 0, height = 0;
  double resolution = 0;
  Pose origin;
  std::vector<int8_t> cells;
};
// Pose arguments are rear-axle poses; corridor faces constrain coveringCenter(pose).
class Scene {
public:
  Scene(Grid grid, const Reference & reference, uint64_t version);
  bool safe(Pose map, double extra=0) const;
  double clearanceAt(Pose map) const;
  std::array<Face, 4> rectangle(Pose map, Pose solve_frame) const;
  std::vector<Pose> route(Pose start, Pose goal, Clock::time_point deadline,
    int max_expansions = 12000) const;
  const Grid & grid() const {return grid_;}
  const std::vector<uint8_t> & safeMask() const {return safe_;}
  uint64_t version() const {return version_;}
private:
  int cell(Pose map) const;
  Pose rearAtCell(int index) const;
  Grid grid_;
  const Reference & reference_;
  uint64_t version_;
  std::vector<uint8_t> safe_;
  std::vector<int> blocked_prefix_;
  int blocked(int x0, int y0, int x1, int y1) const;
  bool freeDisk(int x,int y,double radius) const;
};
}  // namespace mpc
