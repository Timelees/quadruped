// Simulation-configured local replay, not a robot execution success-rate test.
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include "ocs2_quadruped_controller/interface/constraint/SwingTrajectoryPlanner.h"
using namespace ocs2::legged_robot;
using Clock=std::chrono::steady_clock;
namespace {
class Bar final:public TerrainInterface {
 public:
 double height=.02,x=0,y=0;
 double getHeight(double px,double py) const override {
  return std::abs(px-x)<=.015 && std::abs(py-y)<=.5?height:0.;
 }
 std::vector<TerrainPrism> getPrisms(const Eigen::Vector2d&,const Eigen::Vector2d&) const override {
  return {{{-10,-10},{10,10},0},{{x-.015,y-.5},{x+.015,y+.5},height}};
 }
};
double ms(Clock::time_point a){return std::chrono::duration<double,std::milli>(Clock::now()-a).count();}
void matrix(std::ostream& f,const char* name,const Eigen::MatrixXd& m){
 f<<",\""<<name<<"\":[";
 for(int i=0;i<m.rows();++i){f<<(i?",":"")<<'[';for(int j=0;j<m.cols();++j)f<<(j?",":"")<<m(i,j);f<<']';}f<<']';
}
void spline(std::ostream& f,const BSpline& s){
 f<<"\"knots\":[";for(int i=0;i<s.getKnots().size();++i)f<<(i?",":"")<<s.getKnots()[i];
 f<<"]";matrix(f,"controls",s.getControlPoints());
}
}
int main(int argc,char**argv){
 if(argc!=5){std::cerr<<"usage: feasibility_sweep TASK_INFO REPLAY_JSONL OUTPUT SWING_SECONDS\n";return 2;}
 const auto cfg=loadSwingTrajectorySettings(argv[1],"swing_trajectory_config",false);
 const double duration=std::stod(argv[4]);
 if(!(duration>.1 && duration<1))throw std::runtime_error("invalid runtime gait duration");
 std::vector<BSpline> sources;std::ifstream input(argv[2]);std::string line;
 while(std::getline(input,line)){
  boost::property_tree::ptree pt;std::istringstream buf(line);boost::property_tree::read_json(buf,pt);
  const double start=pt.get<double>("start"),end=pt.get<double>("end");
  if(std::abs((end-start)-duration)>1e-5)continue;
  std::vector<double> kv;for(const auto& v:pt.get_child("knots"))kv.push_back(v.second.get_value<double>()-start);
  for(auto& t:kv){if(std::abs(t)<1e-8)t=0.;if(std::abs(t-duration)<1e-8)t=duration;}
  if(std::abs(kv.front())>1e-6)continue; // unmodified executing reference only
  std::vector<Eigen::Vector3d> cv;
  for(const auto& r:pt.get_child("controls")){Eigen::Vector3d c;int k=0;for(const auto& v:r.second)c[k++]=v.second.get_value<double>();cv.push_back(c);}
  BSpline s(5);Eigen::VectorXd x(cv.size()),y(cv.size()),z(cv.size());
  for(size_t i=0;i<cv.size();++i){x[i]=cv[i].x();y[i]=cv[i].y();z[i]=cv[i].z();}
  s.setKnots(Eigen::Map<Eigen::VectorXd>(kv.data(),kv.size()));s.setControlPoints(x,y,z);
  const auto p0=s.getPosition(0),p1=s.getPosition(duration);
  if(std::abs(p1.x()-p0.x())<.025 || std::abs(p1.z()-p0.z())>.008 || p0.z()>.045)continue;
  // Local flat-ground replay: preserve the actual shape/derivatives, translate only.
  const auto mid=s.getPosition(duration/2);x.array()-=mid.x();y.array()-=mid.y();z.array()-=p0.z();
  s.setControlPoints(x,y,z);sources.push_back(s);
 }
 if(sources.empty())throw std::runtime_error("No moving level-ground MPC references; cannot manufacture replay");
 const std::filesystem::path out(argv[3]);std::filesystem::create_directories(out);
 if(std::filesystem::exists(out/"trials.csv"))throw std::runtime_error("outputs already exist");
 std::ofstream scan(out/"trials.csv"),hulls(out/"hulls.jsonl"),accepted(out/"accepted.jsonl");
 scan<<std::setprecision(17)<<"height_m,remaining_s,trial,source,obstacle_dx,dp_norm,dv_norm,da_norm,candidate_certified,instantiated,terminal_preserved,corridor_ms,admission_ms,instantiate_ms,total_ms,reason\n";
 hulls<<std::setprecision(17);accepted<<std::setprecision(17);
 // Export every complete physical polynomial span, allowing a disclosed best-span selection.
 for(size_t i=0;i<sources.size();++i)for(const auto& seg:extractPolynomialSegments(sources[i])){
  hulls<<"{\"source\":"<<i<<",\"start\":"<<seg.start<<",\"end\":"<<seg.end;
  matrix(hulls,"bernstein",hullMap(seg,HullBasis::Bezier)*sources[i].getControlPoints());
  matrix(hulls,"minvo",hullMap(seg,HullBasis::Minvo)*sources[i].getControlPoints());
  hulls<<",\"spline\":{";spline(hulls,sources[i]);hulls<<"}}\n";
 }
 int n=0,pass=0;auto terrain=std::make_shared<Bar>();
 for(int h=1;h<=8;++h)for(int bin=1;bin<=8;++bin)for(int trial=0;trial<100;++trial){
  // Paired, independently seeded uncertainty draws; identical trial seeds across cells.
  std::mt19937 gen(20261121u+trial);std::uniform_real_distribution<double> u(-1,1);
  const size_t index=gen()%sources.size();const auto& old=sources[index];
  auto draw=[&](double radius){Eigen::Vector3d d;do{for(int k=0;k<3;++k)d[k]=u(gen);}while(d.norm()>1);return (radius*d).eval();};
  const auto dp=draw(cfg.replanConfig.positionUncertainty),dv=draw(cfg.replanConfig.velocityUncertainty),da=draw(cfg.replanConfig.disturbanceBound);
  terrain->x=.01*u(gen);terrain->height=.02*h;
  const double remaining=duration*bin/8.,now=duration-remaining;
  const auto begin=Clock::now();auto normalized=old;normalized.setKnots((old.getKnots()/duration).eval());
  const auto t0=Clock::now();
  TerrainSafetyCorridor corridor;
  if(remaining>=cfg.replanConfig.minRemainingTime && now/duration<1-cfg.terrainQpConfig.corridorConfig.contactPhase)
  corridor=buildTerrainSafetyCorridor(*terrain,normalized,cfg.terrainQpConfig.corridorConfig,
    cfg.terrainQpConfig.subdivisionsPerSpan,cfg.terrainQpConfig.collisionCheck.basis,{0,1},duration,
    std::max(cfg.terrainQpConfig.corridorConfig.contactPhase,now/duration),1-cfg.terrainQpConfig.corridorConfig.contactPhase);
  const double corridorMs=ms(t0);SwingOnlineReplanner planner(cfg.replanConfig);
  const auto t1=Clock::now();
  const auto planned=planner.admit(old,corridor,0,now,duration,old.getPosition(now)+dp,old.getVelocity(now)+dv,old.getAcceleration(now)+da);
  const double admissionMs=ms(t1);SwingReplanDecision final;
  const auto t2=Clock::now();if(planned.admitted())final=planner.instantiate(planned.candidate,corridor,0,now,duration,
    old.getPosition(now)+dp,old.getVelocity(now)+dv,old.getAcceleration(now)+da);
  const double instMs=ms(t2),totalMs=ms(begin);bool terminal=false;
  if(final.admitted()){
   terminal=(final.candidate.getPosition(duration)-old.getPosition(duration)).norm()<1e-7 &&
    (final.candidate.getVelocity(duration)-old.getVelocity(duration)).norm()<1e-6 &&
    (final.candidate.getAcceleration(duration)-old.getAcceleration(duration)).norm()<1e-4;
   if(!terminal)throw std::runtime_error("changed fixed terminal boundary");
   ++pass;accepted<<"{\"height\":"<<terrain->height<<",\"remaining\":"<<remaining<<",\"trial\":"<<trial<<",\"source\":"<<index<<",\"new\":{";
   spline(accepted,final.candidate);accepted<<"}}\n";
  }
  scan<<terrain->height<<','<<remaining<<','<<trial<<','<<index<<','<<terrain->x<<','<<dp.norm()<<','<<dv.norm()<<','<<da.norm()<<','
   <<planned.admitted()<<','<<final.admitted()<<','<<terminal<<','<<corridorMs<<','<<admissionMs<<','<<instMs<<','<<totalMs<<",\""
   <<(planned.admitted()?final.reason:planned.reason)<<"\"\n";++n;
 }
 std::ofstream summary(out/"summary.json");summary<<"{\"n\":"<<n<<",\"instantiated\":"<<pass<<",\"sources\":"<<sources.size()
  <<",\"swing_s\":"<<duration<<",\"seed\":20261121,\"physical_trials\":0,\"workspace_gate\":false,\"wbc_gate\":false}\n";
 std::cout<<"n="<<n<<" instantiated="<<pass<<" sources="<<sources.size()<<'\n';
}
