#include "mpc_controller/controller.hpp"

namespace mpc {
namespace {
double milliseconds(Clock::time_point start)
{return std::chrono::duration<double,std::milli>(Clock::now()-start).count();}

bool clearSegment(const Scene & scene, Pose a, Pose b, double extra=0)
{
  const int count=std::max(1,int(std::ceil(std::hypot(b.x-a.x,b.y-a.y)/.025)));
  for (int i=0;i<=count;++i) {
    const double t=double(i)/count;
    if (!scene.safe({a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),0},extra)) {return false;}
  }
  return true;
}

Pose along(const std::vector<Pose> & path, double distance, Pose ego)
{
  for (size_t i=1;i<path.size();++i) {
    const auto a=i==1 ? ego : path[i-1],b=path[i];
    const double len=std::hypot(b.x-a.x,b.y-a.y);
    if (len < 1e-9) {continue;}
    if (distance <= len) {
      const double t=distance/len;
      return {a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),std::atan2(b.y-a.y,b.x-a.x)};
    }
    distance-=len;
  }
  auto p=path.back(),a=path[path.size()-2];
  p.yaw=std::atan2(p.y-a.y,p.x-a.x); return p;
}
}  // namespace

void Controller::reset()
{route_.clear();warm_.reset();station_.reset();solver_.reset();}

Problem Controller::prepare(Pose ego, Command previous, double elapsed,
  const Reference & reference,const Scene & scene,Clock::time_point deadline)
{
  if (!finite(ego) || !validCommand(previous) || !std::isfinite(elapsed) ||
    elapsed <= 0 || elapsed > .25 || Clock::now()>deadline || !scene.safe(ego)) {
    throw std::runtime_error("InvalidStateOrDeadline");
  }
  if (track_version_ != reference.version()) {reset();track_version_=reference.version();}
  const auto & geometry=reference.geometry();
  auto projected=station_ ? geometry.project({ego.x,ego.y,0},ego.yaw,*station_) :
    geometry.projectGlobal({ego.x,ego.y,0},ego.yaw,1.2);
  if (station_ && (projected.seed_was_stale ||
    std::abs(geometry.deltaS(*station_,projected.s)) > speed_cap*elapsed+.3)) {
    reset(); throw std::runtime_error("ProjectionJump");
  }
  station_=station_ ? *station_+geometry.deltaS(*station_,projected.s) : projected.s;
  auto tangent=geometry.sampleAtS(*station_);
  if (std::abs(angleDifference(ego.yaw,tangent.heading))>1.2) {
    reset();throw std::runtime_error("WrongReferenceBranch");
  }

  // Keep one active route until its goal is near; this preserves the chosen detour.
  if (route_.size()>1) {
    size_t nearest=0; double best=INFINITY;
    for (size_t i=0;i+1<route_.size();++i) {
      const auto a=route_[i],b=route_[i+1];
      const double dx=b.x-a.x,dy=b.y-a.y,len=dx*dx+dy*dy;
      const double t=len>0 ? std::clamp(((ego.x-a.x)*dx+(ego.y-a.y)*dy)/len,0.,1.) : 0;
      const double d=std::hypot(ego.x-a.x-t*dx,ego.y-a.y-t*dy);
      if (d<best) {best=d;nearest=i;}
    }
    if (best>.6 || std::hypot(route_.back().x-ego.x,route_.back().y-ego.y)<1.5) {
      route_.clear();
    } else {
      route_.erase(route_.begin(),route_.begin()+nearest);
      for (size_t i=1;i<route_.size();++i) {
        if (Clock::now()>deadline) {throw std::runtime_error("Deadline");}
        if (!clearSegment(scene,route_[i-1],route_[i])) {route_.clear();break;}
      }
    }
  }
  if (route_.size()<2) {
    std::vector<Pose> forward{ego};
    double station=*station_;
    for (int i=0;i<45;++i) {
      auto g=geometry.sampleAtS(station);
      station+=.1/g.parameter_speed;
      g=geometry.sampleAtS(station);
      forward.push_back({g.x,g.y,g.heading});
    }
    while (forward.size()>2 && !scene.safe(forward.back(),.2)) {forward.pop_back();}
    bool clear=true;
    for (size_t i=1;i<forward.size();++i) {clear &= clearSegment(scene,forward[i-1],forward[i]);}
    route_=clear ? std::move(forward) : scene.route(ego,forward.back(),deadline);
    if (route_.size()<2) {throw std::runtime_error("NoSeed");}
    if (!clear || std::abs(projected.d)>.3) {
      // Preserve clear raceline curvature; simplify only grid routes or an off-line rejoin.
      std::vector<Pose> reduced{route_.front()};
      for (size_t i=0;i+1<route_.size();) {
        if (Clock::now()>deadline) {throw std::runtime_error("Deadline");}
        size_t next=i+1;
        for (size_t j=i+2;j<route_.size();++j) {
          if (Clock::now()>deadline) {throw std::runtime_error("Deadline");}
          if (!clearSegment(scene,route_[i],route_[j],.2)) {break;}
          next=j;
        }
        reduced.push_back(route_[next]);i=next;
      }
      route_=std::move(reduced);
    }
    const double direction=std::atan2(route_[1].y-ego.y,route_[1].x-ego.x);
    if (std::abs(angleDifference(direction,ego.yaw))>1.2) {
      route_.clear();throw std::runtime_error("WrongWaySeed");
    }
  }
  Problem problem;
  problem.frame=ego;problem.previous=previous;problem.first_interval=elapsed;
  problem.scene_version=scene.version();problem.track_version=reference.version();
  double distance=0;
  double speed_station=*station_;
  for (int k=0;k<=horizon;++k) {
    if (Clock::now()>deadline) {throw std::runtime_error("Deadline");}
    Pose target=along(route_,distance,ego);
    problem.targets[k]=toLocal(target,ego);
    if (k) {problem.targets[k].yaw=problem.targets[k-1].yaw+
      angleDifference(problem.targets[k].yaw,problem.targets[k-1].yaw);}

    if (k<horizon) {
      const double guided=std::min(speed_cap,geometry.velocityAtS(speed_station));
      problem.speeds[k]=std::max(0.,std::min({guided,previous.speed+speed_rate*(elapsed+k*dt),
        speed_rate*(horizon-1-k)*dt}));
      distance+=problem.speeds[k]*dt;
      speed_station+=problem.speeds[k]*dt/geometry.sampleAtS(speed_station).parameter_speed;
    }
  }
  Pose seed;
  Command memory=previous;
  problem.seed_states[0]={0,0,0,memory.speed,memory.steering};
  for (int k=0;k<horizon;++k) {
    const double h=k==0 ? elapsed : dt;
    Command u;
    if (warm_) {
      const double time=k*dt+elapsed;
      const int idx=std::min(horizon-1,int(time/dt));
      u=warm_->commands[idx];
    } else {
      const auto target=problem.targets[std::min(horizon,k+3)];
      const double desired=std::atan2(target.y-seed.y,target.x-seed.x);
      u={problem.speeds[k],std::atan(wheelbase*angleDifference(desired,seed.yaw)/.6)};
    }
    u.speed=std::clamp(u.speed,std::max(0.,memory.speed-speed_rate*h),
      std::min(speed_cap,memory.speed+speed_rate*h));
    u.speed=std::min(u.speed,speed_rate*(horizon-1-k)*dt);
    u.steering=std::clamp(u.steering,std::max(steering_min,memory.steering-steering_rate*h),
      std::min(steering_max,memory.steering+steering_rate*h));
    seed=step(seed,u,dt);memory=u;
    problem.seed_commands[k]=u;
    problem.seed_states[k+1]={seed.x,seed.y,seed.yaw,u.speed,u.steering};
  }
  for (int k=0;k<=horizon;++k) {
    const auto x=problem.seed_states[k];
    const auto seed_map=toMap({x[0],x[1],x[2]},ego);
    // Certify a feasible rollout where available; route timing is only a cost target.
    const auto anchor=scene.safe(seed_map) ? seed_map : toMap(problem.targets[k],ego);
    problem.corridor[k]=scene.rectangle(anchor,ego);
  }
  return problem;
}

Validation validate(const Problem & problem,const Solution & solution,
  const Scene & scene,Clock::time_point deadline)
{
  Validation result;
  auto reject=[&](const char * reason) {result.reason=reason;return result;};
  if (solution.status != 0 || !finite(solution.frame) ||
    solution.scene_version != problem.scene_version || scene.version()!=problem.scene_version ||
    solution.track_version != problem.track_version || solution.frame.x!=problem.frame.x ||
    solution.frame.y!=problem.frame.y || solution.frame.yaw!=problem.frame.yaw ||
    !std::isfinite(solution.inequality_residual) || solution.inequality_residual>1e-5 ||
    !std::isfinite(solution.dynamics_residual) || solution.dynamics_residual>1e-3) {
    return reject("SolverOrVersion");
  }
  Pose state;
  Command memory=problem.previous;
  for (int k=0;k<=horizon;++k) {
    if (Clock::now()>deadline) {return reject("Deadline");}
    auto x=solution.states[k];
    for (double value:x) {if (!std::isfinite(value)) {return reject("Nonfinite");}}
    if (std::hypot(x[0]-state.x,x[1]-state.y)>1e-3 ||
      std::abs(x[2]-state.yaw)>1e-3 || std::abs(x[3]-memory.speed)>1e-6 ||
      std::abs(x[4]-memory.steering)>1e-6) {return reject("RolloutDefect");}
    for (auto face:problem.corridor[k]) {
      if (face.x*state.x+face.y*state.y>face.bound+1e-6) {return reject("Corridor");}
    }
    if (k==horizon) {break;}
    const auto u=solution.commands[k];
    const double h=k==0 ? problem.first_interval : dt;
    if (!validCommand(u) || std::abs(u.speed-memory.speed)>speed_rate*h+1e-6 ||
      std::abs(u.steering-memory.steering)>steering_rate*h+1e-6) {return reject("CommandLimit");}
    const int samples=std::max(1,int(std::ceil(u.speed*dt/.02)));
    const double sweep=u.speed*dt/(2*samples);
    for (int j=0;j<=samples;++j) {
      auto p=toMap(step(state,u,dt*j/samples),problem.frame);
      const double available=scene.clearanceAt(p);
      result.minimum_clearance=std::min(result.minimum_clearance,available);
      if (available < clearance+.005+sweep) {return reject("CollisionOrTrack");}
    }
    state=step(state,u,dt);memory=u;
  }
  if (std::abs(solution.commands.back().speed)>1e-7) {return reject("TerminalStop");}
  result.accepted=true;result.reason="Approved";return result;
}

Cycle Controller::cycle(Pose ego,Command previous,double elapsed,const Reference & reference,
  const Scene & scene,Clock::time_point deadline)
{
  Cycle result;
  const auto start=Clock::now();
  try {
    result.problem=prepare(ego,previous,elapsed,reference,scene,deadline);
    result.prepare_ms=milliseconds(start);
    if (Clock::now()>deadline) {throw std::runtime_error("Deadline");}
    result.solution=solver_.solve(result.problem);
    const auto checking=Clock::now();
    result.validation=validate(result.problem,result.solution,scene,deadline);
    result.validation_ms=milliseconds(checking);
    if (result.solution.status==0 && std::all_of(result.solution.commands.begin(),
      result.solution.commands.end(),[](Command command) {return validCommand(command);})) {
      warm_=result.solution;
    } else {warm_.reset();}
  } catch (const std::exception & error) {result.validation.reason=error.what();}
  result.total_ms=milliseconds(start);
  return result;
}
}  // namespace mpc
