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
  const std::vector<local_planning::TrackWidth> & widths,
  const std::array<std::vector<Pose>, 2> & legal_boundaries)
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
  if (!legal_boundaries[0].empty() || !legal_boundaries[1].empty()) {
    boundaries=legal_boundaries;
    for (const auto & loop:boundaries) {
      if (loop.size()<3 || loop.size()>2000) {throw std::invalid_argument("Boundary capacity");}
      for (size_t i=0;i<loop.size();++i) {
        if (!finite(loop[i]) || std::abs(loop[i].x)>1e5 || std::abs(loop[i].y)>1e5 ||
          std::hypot(loop[i].x-loop[(i+1)%loop.size()].x,
          loop[i].y-loop[(i+1)%loop.size()].y)<1e-8) {
          throw std::invalid_argument("Invalid boundary vertex");
        }
      }
    }
    auto intersects=[](Pose a,Pose b,Pose c,Pose d) {
      auto cross=[](Pose p,Pose q,Pose r) {return (q.x-p.x)*(r.y-p.y)-(q.y-p.y)*(r.x-p.x);};
      if (segmentDistance(a,c,d)<1e-8 || segmentDistance(b,c,d)<1e-8 ||
        segmentDistance(c,a,b)<1e-8 || segmentDistance(d,a,b)<1e-8) {return true;}
      return (cross(a,b,c)>0)!=(cross(a,b,d)>0) && (cross(c,d,a)>0)!=(cross(c,d,b)>0);
    };
    for (int first=0;first<2;++first) {
      const auto & a=boundaries[first];
      for (int second=first;second<2;++second) {
        const auto & b=boundaries[second];
        for (size_t i=0;i<a.size();++i) {
          for (size_t j=0;j<b.size();++j) {
            if (first==second && (j<=i || j==i+1 || (i==0 && j==a.size()-1))) {continue;}
            if (intersects(a[i],a[(i+1)%a.size()],b[j],b[(j+1)%b.size()])) {
              throw std::invalid_argument("Intersecting legal boundaries");
            }
          }
        }
      }
    }
    if (inside(boundaries[0][0],boundaries[1])==inside(boundaries[1][0],boundaries[0])) {
      throw std::invalid_argument("Legal boundaries must be nested");
    }
    for (const auto & p:points) {
      if (inside({p.x,p.y,0},boundaries[0])==inside({p.x,p.y,0},boundaries[1])) {
        throw std::invalid_argument("Reference outside legal boundaries");
      }
    }
  } else {
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
      safe_[y*w+x] = freeDisk(x,y,coveringRadius()+clearance+.005);
    }
  }
}

int Scene::blocked(int x0, int y0, int x1, int y1) const
{
  const int stride = grid_.width+1;
  return blocked_prefix_[(y1+1)*stride+x1+1]-blocked_prefix_[y0*stride+x1+1]-
         blocked_prefix_[(y1+1)*stride+x0]+blocked_prefix_[y0*stride+x0];
}
bool Scene::freeDisk(int x,int y,double radius) const
{
  const double h=grid_.resolution;
  const int extent=int(std::ceil(radius/h))+1;
  if (x<extent || y<extent || x+extent>=grid_.width || y+extent>=grid_.height) {return false;}
  // Minimum distance between two entire cell squares, including subcell uncertainty.
  for (int dy=-extent;dy<=extent;++dy) {
    const double distance=std::max(0,std::abs(dy)-1)*h;
    if (distance>radius) {continue;}
    const int dx=int(std::floor(std::sqrt(std::max(0.,radius*radius-distance*distance))/h))+1;
    if (blocked(x-dx,y+dy,x+dx,y+dy)) {return false;}
  }
  return true;
}
int Scene::cell(Pose map) const
{
  if (!finite(map)) {return -1;}
  auto p = toLocal(coveringCenter(map), grid_.origin);
  const double x = std::floor(p.x/grid_.resolution), y = std::floor(p.y/grid_.resolution);
  if (x < 0 || y < 0 || x >= grid_.width || y >= grid_.height) {return -1;}
  return int(y)*grid_.width+int(x);
}
Pose Scene::rearAtCell(int index) const
{return toMap({(index%grid_.width+.5)*grid_.resolution-(front_extent-rear_extent)/2,
    (index/grid_.width+.5)*grid_.resolution, 0}, grid_.origin);}
bool Scene::safe(Pose map, double extra) const
{
  const int i=cell(map);
  if (i<0 || !std::isfinite(extra) || extra<0) {return false;}
  if (extra==0) {return safe_[i] || clearanceAt(map)>=clearance+.005;}
  if (!safe_[i]) {return false;}
  const int x=i%grid_.width,y=i/grid_.width;
  return freeDisk(x,y,coveringRadius()+clearance+.005+extra);
}

double Scene::clearanceAt(Pose map) const
{
  if (cell(map) < 0) {return -INFINITY;}
  map=coveringCenter(map);
  const double radius = coveringRadius();
  const auto p = toLocal(map, grid_.origin);
  double available = std::min({p.x, p.y, grid_.width*grid_.resolution-p.x,
    grid_.height*grid_.resolution-p.y, reference_.trackClearance(map),radius+clearance+.1});
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
  if (index < 0) {throw std::invalid_argument("Unsafe corridor seed");}
  if (!safe_[index]) {
    // A full-cell mask can reject a safe measured pose. Certify a small continuous box
    // from original obstacle/track distance; clearanceAt caps unsearched distances.
    const double half=(clearanceAt(map)-clearance-.005)/std::sqrt(2.);
    if (!std::isfinite(half) || half<=1e-6) {throw std::invalid_argument("Unsafe corridor seed");}
    const auto center=toLocal(coveringCenter(map),frame);
    return {{{1,0,center.x+half},{-1,0,-center.x+half},
      {0,1,center.y+half},{0,-1,-center.y+half}}};
  }
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
  const double h = grid_.resolution;
  return {{{c,s,(x1+1)*h+offset_x}, {-c,-s,-x0*h-offset_x},
    {-s,c,(y1+1)*h+offset_y}, {s,-c,-y0*h-offset_y}}};
}

std::vector<Pose> Scene::route(Pose start, Pose goal, Clock::time_point deadline,
  int max_expansions) const
{
  const int first = cell(start), last = cell(goal);
  if (first < 0 || last < 0 || !safe_[first] || !safe(rearAtCell(last),.2)) {return {};}
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
        result.push_back(rearAtCell(at));
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
        if (!safe(rearAtCell(next),.2) || (dx && dy &&
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
