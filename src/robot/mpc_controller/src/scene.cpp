#include "mpc_controller/scene.hpp"
#include <limits>
#include <queue>

namespace mpc {
namespace {
double segmentDistance(Pose p, Pose a, Pose b)
{
  const double dx = b.x-a.x, dy = b.y-a.y;
  const double len = dx*dx+dy*dy;
  const double t = len > 0 ? std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/len, 0., 1.) : 0;
  return std::hypot(p.x-a.x-t*dx, p.y-a.y-t*dy);
}
bool inside(Pose p, const std::vector<Pose> & polygon)
{
  bool result = false;
  for (size_t i = 0, j = polygon.size()-1; i < polygon.size(); j = i++) {
    const auto a = polygon[i], b = polygon[j];
    if ((a.y > p.y) != (b.y > p.y) && p.x < (b.x-a.x)*(p.y-a.y)/(b.y-a.y)+a.x) {
      result = !result;
    }
  }
  return result;
}
}  // namespace

void Reference::set(const std::vector<local_planning::Point> & points,
  const std::vector<local_planning::TrackWidth> & widths)
{
  if (points.size() < 4 || points.size() > 20000 || points.size() != widths.size()) {
    throw std::invalid_argument("Reference/width size mismatch");
  }
  for (size_t i = 0; i < points.size(); ++i) {
    auto p = points[i]; auto w = widths[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.velocity) ||
      !std::isfinite(w.left_m) || !std::isfinite(w.right_m) || p.velocity < 0 ||
      w.left_m <= 0 || w.right_m <= 0 || w.left_m>100 || w.right_m>100 ||
      std::abs(p.x)>1e5 || std::abs(p.y)>1e5) {
      throw std::invalid_argument("Invalid reference geometry");
    }
  }
  local_planning::RacelineReference candidate;
  if (!candidate.setRacingLine(points) || !std::isfinite(candidate.totalLength()) ||
    candidate.totalLength()>2000 || !candidate.setTrackWidths(widths, .05)) {
    throw std::invalid_argument("Degenerate reference");
  }
  std::array<std::vector<Pose>, 2> boundaries;
  const int count = std::max(100, static_cast<int>(std::ceil(candidate.totalLength()/.05)));
  if (count > 40000) {throw std::invalid_argument("Reference capacity exceeded");}
  std::vector<double> stations;
  stations.reserve(count+candidate.widthSampleCount());
  for (int i=0;i<count;++i) {stations.push_back(candidate.totalLength()*i/count);}
  for (size_t i=0;i<candidate.widthSampleCount();++i) {stations.push_back(candidate.widthSample(i).s);}
  std::sort(stations.begin(),stations.end());
  stations.erase(std::unique(stations.begin(),stations.end(),
    [](double a,double b) {return std::abs(a-b)<1e-9;}),stations.end());
  for (double station:stations) {
    auto g=candidate.sampleAtS(station);
    auto w=candidate.rawBounds(station);
    const auto before=candidate.rawBounds(station-1e-8);
    // Both ends of each chord must respect the smaller adjacent width bin.
    w.left_magnitude=std::min(w.left_magnitude,before.left_magnitude);
    w.right_magnitude=std::min(w.right_magnitude,before.right_magnitude);
    if (!std::isfinite(g.parameter_speed) || g.parameter_speed < 1e-6) {
      throw std::invalid_argument("Invalid spline metric");
    }
    if (!std::isfinite(g.curvature) ||
      std::abs(g.curvature)*std::max(w.left_magnitude,w.right_magnitude)>=.8) {
      throw std::invalid_argument("Unsupported offset-boundary fold");
    }
    // shortcut: 5 cm chords plus 5 mm tolerance are fixture-validated, not a proof for arbitrary tracks.
    boundaries[0].push_back({g.x+g.normal_x*w.left_magnitude,
      g.y+g.normal_y*w.left_magnitude, 0});
    boundaries[1].push_back({g.x-g.normal_x*w.right_magnitude,
      g.y-g.normal_y*w.right_magnitude, 0});
  }
  geometry_ = std::move(candidate);
  edges_ = std::move(boundaries);
  ++version_;
}

double Reference::trackClearance(Pose p) const
{
  if (edges_[0].empty() || !finite(p)) {return -INFINITY;}
  double distance = INFINITY;
  for (const auto & polygon : edges_) {
    for (size_t i = 0; i < polygon.size(); ++i) {
      distance = std::min(distance, segmentDistance(p, polygon[i], polygon[(i+1)%polygon.size()]));
    }
  }
  return inside(p, edges_[0]) != inside(p, edges_[1]) ? distance : -distance;
}

Scene::Scene(Grid grid, const Reference & reference, uint64_t version)
: grid_(std::move(grid)), reference_(reference), version_(version)
{
  if (grid_.width <= 0 || grid_.height <= 0 || grid_.width > 1000 || grid_.height > 1000 ||
    !std::isfinite(grid_.resolution) || grid_.resolution < .01 || grid_.resolution > 1 ||
    !finite(grid_.origin) || grid_.cells.size() != size_t(grid_.width)*grid_.height ||
    reference.edges()[0].empty()) {throw std::invalid_argument("Malformed scene");}
  for (auto value : grid_.cells) {
    if (value < -1 || value > 100) {throw std::invalid_argument("Invalid occupancy value");}
  }
  const int w = grid_.width, h = grid_.height;
  std::vector<uint8_t> legal(w*h, 0);
  // Scan-line XOR fills the annulus between the two closed track boundaries.
  std::vector<double> crossings;
  std::array<std::vector<Pose>, 2> edges;
  for (int side = 0; side < 2; ++side) {
    for (auto p : reference.edges()[side]) {edges[side].push_back(toLocal(p, grid_.origin));}
  }
  for (int row = 0; row < h; ++row) {
    crossings.clear();
    const double y = (row+.5)*grid_.resolution;
    for (const auto & polygon : edges) {
      for (size_t i = 0; i < polygon.size(); ++i) {
        auto a = polygon[i], b = polygon[(i+1)%polygon.size()];
        if ((a.y > y) != (b.y > y)) {crossings.push_back(a.x+(y-a.y)*(b.x-a.x)/(b.y-a.y));}
      }
    }
    std::sort(crossings.begin(), crossings.end());
    for (size_t j = 0; j+1 < crossings.size(); j += 2) {
      const int x0 = std::clamp(int(std::ceil(crossings[j]/grid_.resolution-.5)), 0, w);
      const int x1 = std::clamp(int(std::ceil(crossings[j+1]/grid_.resolution-.5)), 0, w);
      for (int x = x0; x < x1; ++x) {legal[row*w+x] = 1;}
    }
  }
  // Mark every cell touched by a boundary; center-only rasterization can miss slivers.
  const double diagonal = grid_.resolution/std::sqrt(2.);
  for (const auto & polygon : edges) {
    for (size_t i = 0; i < polygon.size(); ++i) {
      auto a = polygon[i], b = polygon[(i+1)%polygon.size()];
      int x0 = std::clamp(int(std::floor((std::min(a.x,b.x)-diagonal)/grid_.resolution)), 0, w-1);
      int x1 = std::clamp(int(std::floor((std::max(a.x,b.x)+diagonal)/grid_.resolution)), 0, w-1);
      int y0 = std::clamp(int(std::floor((std::min(a.y,b.y)-diagonal)/grid_.resolution)), 0, h-1);
      int y1 = std::clamp(int(std::floor((std::max(a.y,b.y)+diagonal)/grid_.resolution)), 0, h-1);
      for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
          if (segmentDistance({(x+.5)*grid_.resolution,(y+.5)*grid_.resolution,0},a,b) <= diagonal+.005) {
            legal[y*w+x] = 0;
          }
        }
      }
    }
  }
  blocked_prefix_.assign((w+1)*(h+1), 0);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const bool occupied = grid_.cells[y*w+x] < 0 || grid_.cells[y*w+x] >= 50;
      blocked_prefix_[(y+1)*(w+1)+x+1] = int(occupied || !legal[y*w+x]) +
        blocked_prefix_[y*(w+1)+x+1]+blocked_prefix_[(y+1)*(w+1)+x]-blocked_prefix_[y*(w+1)+x];
    }
  }
  safe_.assign(w*h, 0);
  const int radius = int(std::ceil((coveringRadius()+clearance+.005)/grid_.resolution))+1;
  for (int y = radius; y < h-radius; ++y) {
    for (int x = radius; x < w-radius; ++x) {
      safe_[y*w+x] = blocked(x-radius,y-radius,x+radius,y+radius) == 0;
    }
  }
}

int Scene::blocked(int x0, int y0, int x1, int y1) const
{
  const int stride = grid_.width+1;
  return blocked_prefix_[(y1+1)*stride+x1+1]-blocked_prefix_[y0*stride+x1+1]-
         blocked_prefix_[(y1+1)*stride+x0]+blocked_prefix_[y0*stride+x0];
}
int Scene::cell(Pose map) const
{
  if (!finite(map)) {return -1;}
  auto p = toLocal(map, grid_.origin);
  const double x = std::floor(p.x/grid_.resolution), y = std::floor(p.y/grid_.resolution);
  if (x < 0 || y < 0 || x >= grid_.width || y >= grid_.height) {return -1;}
  return int(y)*grid_.width+int(x);
}
Pose Scene::center(int index) const
{return toMap({(index%grid_.width+.5)*grid_.resolution,
    (index/grid_.width+.5)*grid_.resolution, 0}, grid_.origin);}
bool Scene::safe(Pose map, double extra) const
{
  const int i=cell(map);
  if (i<0 || !safe_[i] || !std::isfinite(extra) || extra<0) {return false;}
  if (extra==0) {return true;}
  const int r=int(std::ceil((coveringRadius()+clearance+.005+extra)/grid_.resolution))+1;
  const int x=i%grid_.width,y=i/grid_.width;
  return x>=r && y>=r && x+r<grid_.width && y+r<grid_.height &&
    blocked(x-r,y-r,x+r,y+r)==0;
}

double Scene::clearanceAt(Pose map) const
{
  if (cell(map) < 0) {return -INFINITY;}
  const double radius = coveringRadius();
  const auto p = toLocal(map, grid_.origin);
  double available = std::min({p.x, p.y, grid_.width*grid_.resolution-p.x,
    grid_.height*grid_.resolution-p.y, reference_.trackClearance(map)});
  const int extent = int(std::ceil((radius+clearance+.1)/grid_.resolution))+1;
  const int cx = int(p.x/grid_.resolution), cy = int(p.y/grid_.resolution);
  for (int y = std::max(0,cy-extent); y <= std::min(grid_.height-1,cy+extent); ++y) {
    for (int x = std::max(0,cx-extent); x <= std::min(grid_.width-1,cx+extent); ++x) {
      auto value = grid_.cells[y*grid_.width+x];
      if (value >= 0 && value < 50) {continue;}
      const double dx = std::max({x*grid_.resolution-p.x, 0., p.x-(x+1)*grid_.resolution});
      const double dy = std::max({y*grid_.resolution-p.y, 0., p.y-(y+1)*grid_.resolution});
      available = std::min(available, std::hypot(dx,dy));
    }
  }
  return available-radius;
}

std::array<Face, 4> Scene::rectangle(Pose map, Pose frame) const
{
  int index = cell(map);
  if (index < 0 || !safe_[index]) {throw std::invalid_argument("Unsafe corridor seed");}
  int x0 = index%grid_.width, x1 = x0, y0 = index/grid_.width, y1 = y0;
  const int max_cells = int(std::ceil(1.5/grid_.resolution));
  for (int n = 0; n < max_cells; ++n) {
    if (x0 > 0) {
      bool ok = true; for (int y=y0; y<=y1; ++y) {ok &= safe_[y*grid_.width+x0-1];}
      if (ok) {--x0;}
    }
    if (x1+1 < grid_.width) {
      bool ok = true; for (int y=y0; y<=y1; ++y) {ok &= safe_[y*grid_.width+x1+1];}
      if (ok) {++x1;}
    }
    if (y0 > 0) {
      bool ok = true; for (int x=x0; x<=x1; ++x) {ok &= safe_[(y0-1)*grid_.width+x];}
      if (ok) {--y0;}
    }
    if (y1+1 < grid_.height) {
      bool ok = true; for (int x=x0; x<=x1; ++x) {ok &= safe_[(y1+1)*grid_.width+x];}
      if (ok) {++y1;}
    }
  }
  const double a = grid_.origin.yaw-frame.yaw, c = std::cos(a), s = std::sin(a);
  const auto origin = toLocal(grid_.origin, frame);
  const double offset_x = c*origin.x+s*origin.y, offset_y = -s*origin.x+c*origin.y;
  const double h = grid_.resolution, eps = 1e-6;
  return {{{c,s,(x1+1)*h+offset_x-eps}, {-c,-s,-x0*h-offset_x-eps},
    {-s,c,(y1+1)*h+offset_y-eps}, {s,-c,-y0*h-offset_y-eps}}};
}

std::vector<Pose> Scene::route(Pose start, Pose goal, Clock::time_point deadline,
  int max_expansions) const
{
  const int first = cell(start), last = cell(goal);
  if (first < 0 || last < 0 || !safe_[first] || !safe(center(last),.2)) {return {};}
  const int count = grid_.width*grid_.height;
  std::vector<double> costs(count, INFINITY);
  std::vector<int> parents(count, -1);
  std::vector<uint8_t> closed(count, 0);
  using Entry = std::pair<double,int>;
  std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> queue;
  costs[first] = 0; queue.push({0,first});
  int expanded = 0;
  while (!queue.empty() && expanded < max_expansions) {
    if ((expanded&63) == 0 && Clock::now() > deadline) {return {};}
    const int current = queue.top().second; queue.pop();
    if (closed[current]) {continue;}
    closed[current] = 1;
    if (current == last) {
      std::vector<Pose> result{goal};
      for (int at = last; at != first; at = parents[at]) {
        if (at < 0) {return {};}
        result.push_back(center(at));
      }
      result.push_back(start); std::reverse(result.begin(),result.end()); return result;
    }
    ++expanded;
    for (int dy=-1; dy<=1; ++dy) {
      for (int dx=-1; dx<=1; ++dx) {
        if (!dx && !dy) {continue;}
        const int x=current%grid_.width+dx, y=current/grid_.width+dy;
        if (x<0 || y<0 || x>=grid_.width || y>=grid_.height) {continue;}
        const int next=y*grid_.width+x;
        if (!safe(center(next),.2) || (dx && dy &&
          (!safe_[(y-dy)*grid_.width+x] || !safe_[y*grid_.width+x-dx]))) {continue;}
        const double cost=costs[current]+std::hypot(dx,dy);
        if (cost >= costs[next]) {continue;}
        costs[next]=cost; parents[next]=current;
        const double heuristic=std::hypot(x-last%grid_.width,y-last/grid_.width);
        queue.push({cost+heuristic,next});
      }
    }
  }
  return {};
}
}  // namespace mpc
