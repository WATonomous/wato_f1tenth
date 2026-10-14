#include "mpc_controller/controller.hpp"
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <sys/resource.h>

int main(int argc,char ** argv)
{
  const std::string scene_name=argc>1 ? argv[1] : "solo";
  const int cycles=argc>2 ? std::stoi(argv[2]) : 1000;
  if (cycles<1 || (scene_name!="solo" && scene_name!="obstacle" && scene_name!="blocked")) {
    std::cerr<<"Usage: mpc_benchmark [solo|obstacle|blocked] [cycles]\n";return 2;
  }
  mpc::Reference reference;
  std::vector<local_planning::Point> points;
  std::vector<local_planning::TrackWidth> widths;
  for (int i=0;i<160;++i) {
    const double a=-std::acos(-1.)/2+i*2*std::acos(-1.)/160;
    points.emplace_back(10*std::cos(a),10+10*std::sin(a),1.);
    widths.push_back({3.,3.});
  }
  reference.set(points,widths);
  mpc::Grid grid;
  grid.width=260;grid.height=260;grid.resolution=.1;grid.origin={-13,-3,0};
  grid.cells.assign(grid.width*grid.height,0);
  if (scene_name!="solo") {
    for (int y=0;y<grid.height;++y) {
      for (int x=0;x<grid.width;++x) {
        const double mx=grid.origin.x+(x+.5)*grid.resolution;
        const double my=grid.origin.y+(y+.5)*grid.resolution;
        if (mx>2.8 && mx<3.2 && (scene_name=="blocked" || (my>-.3 && my<.9))) {
          grid.cells[y*grid.width+x]=100;
        }
      }
    }
  }
  mpc::Controller controller;
  mpc::Pose pose;
  mpc::Command previous;
  std::unique_ptr<mpc::Scene> scene;
  std::vector<double> times;
  std::map<std::string,int> reasons;
  int misses=0,approved=0,collisions=0;
  bool passed_obstacle=false;
  double minimum=INFINITY,maximum_error=0,progress=0,last_station=0;
  for (int i=0;i<cycles;++i) {
    const auto start=mpc::Clock::now();
    if (i%5==0) {scene=std::make_unique<mpc::Scene>(grid,reference,i/5+1);}
    auto result=controller.cycle(pose,previous,.02,reference,*scene,
      start+std::chrono::milliseconds(20));
    previous=result.validation.accepted ? result.solution.commands[0] : mpc::Command{};
    approved+=result.validation.accepted;
    ++reasons[result.validation.reason];
    pose=mpc::step(pose,previous,.02);
    passed_obstacle |= pose.x>4.5;
    const double elapsed=std::chrono::duration<double,std::milli>(mpc::Clock::now()-start).count();
    times.push_back(elapsed);misses+=elapsed>20;
    const double available=scene->clearanceAt(pose);
    minimum=std::min(minimum,available);collisions+=available<mpc::clearance;
    auto projected=reference.geometry().projectGlobal({pose.x,pose.y,0},pose.yaw,1.2);
    maximum_error=std::max(maximum_error,std::abs(projected.d));
    progress+=reference.geometry().deltaS(last_station,projected.s);last_station=projected.s;
  }
  std::sort(times.begin(),times.end());
  auto percentile=[&](double p) {return times[std::min(times.size()-1,size_t(p*(times.size()-1)))];};
  rusage usage{};getrusage(RUSAGE_SELF,&usage);
  std::cout<<"scene="<<scene_name<<" cycles="<<cycles<<" approved="<<approved
    <<" misses="<<misses<<" collisions="<<collisions<<" x="<<pose.x<<" y="<<pose.y
    <<" progress_m="<<progress<<" lap_equivalents="<<progress/reference.geometry().totalLength()
    <<" peak_rss_KiB="<<usage.ru_maxrss<<" clearance="<<minimum<<" max_raceline_error="<<maximum_error<<'\n'
    <<"full_cycle_ms p50="<<percentile(.5)<<" p95="<<percentile(.95)
    <<" p99="<<percentile(.99)<<" p999="<<percentile(.999)<<" max="<<times.back()<<'\n';
  for (const auto & [reason,count]:reasons) {std::cout<<reason<<'='<<count<<'\n';}
  const bool advanced=scene_name=="blocked" || cycles<1500 || passed_obstacle;
  const bool laps=scene_name=="blocked" || cycles<10000 ||
    progress>1.5*reference.geometry().totalLength();
  return collisions || !advanced || !laps ? 1 : 0;
}
