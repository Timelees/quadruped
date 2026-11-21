// Production algorithm experiment. This is NOT robot execution or a success-rate trial.
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include "ocs2_quadruped_controller/interface/constraint/SwingTrajectoryPlanner.h"
using namespace ocs2::legged_robot;
namespace {
class Bar final: public TerrainInterface {
 public:
    double height=.10, centre=-1.2;
    double getHeight(double x,double y) const override {
        return std::abs(x-centre)<=.015 && std::abs(y)<=.5 ? height : 0.;
    }
    std::vector<TerrainPrism> getPrisms(const Eigen::Vector2d&,const Eigen::Vector2d&) const override {
        return {{{-3,-2},{1,2},0},{{centre-.015,-.5},{centre+.015,.5},height}};
    }
};
void require(bool b,const std::string& message) {if(!b)throw std::runtime_error(message);}
BSpline reference() {
    auto s=createSwingHeightBSpline(0,.4,0,.10,.085,0,-.03);
    const auto c=s.getControlPoints();
    Eigen::VectorXd x(7);x<<-.98,-.98,-.98,-1.15,-1.32,-1.32,-1.32;
    s.setControlPoints(x,Eigen::VectorXd::Zero(7),c.col(2));return s;
}
// Sphere versus ground and axis-aligned solid box: independent signed-distance oracle.
double distance(const Eigen::Vector3d& tip,const Bar& bar) {
    const Eigen::Vector3d p=tip+.019*Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d centre(bar.centre,0,bar.height/2);
    const Eigen::Vector3d q=(p-centre).cwiseAbs()-Eigen::Vector3d(.015,.5,bar.height/2);
    return std::min(tip.z(),q.cwiseMax(0.).norm()+std::min(q.maxCoeff(),0.)-.019);
}
double minimum(const BSpline& s,double after,const Bar& b) {
    double m=1.;for(int i=0;i<=3000;++i){double t=after+(.30-after)*i/3000.;m=std::min(m,distance(s.getPosition(t),b));}return m;
}
void emit(std::ostream& f,const char* key,const BSpline& s) {
    f<<",\""<<key<<"\":{\"knots\":[";
    for(int i=0;i<s.getKnots().size();++i)f<<(i?",":"")<<s.getKnots()[i];
    f<<"],\"controls\":[";const auto c=s.getControlPoints();
    for(int i=0;i<c.rows();++i)f<<(i?",":"")<<"["<<c(i,0)<<","<<c(i,1)<<","<<c(i,2)<<"]";
    f<<"]}";
}
void emitCertificate(std::ostream& f,const BSpline& s,const TerrainSafetyCorridor& corridor) {
    const auto mapped=SwingOnlineReplanner::remapCorridor(s,corridor,0,.4);
    f<<",\"certificate\":{\"numerical_margin\":"<<mapped.numericalMargin
     <<",\"foot_radius\":"<<mapped.footRadius<<",\"cells\":[";
    for(size_t i=0;i<mapped.cells.size();++i){
        const auto& c=mapped.cells[i];f<<(i?",":"")<<"{\"mapping\":[";
        for(int row=0;row<c.hullMapping.rows();++row){
            f<<(row?",":"")<<"[";
            for(int col=0;col<c.hullMapping.cols();++col)f<<(col?",":"")<<c.hullMapping(row,col);
            f<<"]";
        }
        f<<"],\"planes\":[";
        for(size_t j=0;j<c.halfSpaces.size();++j){const auto& p=c.halfSpaces[j];
            f<<(j?",":"")<<"["<<p.normal.x()<<","<<p.normal.y()<<","<<p.normal.z()<<","
             <<p.planningOffset<<","<<p.basisMargin<<"]";
        }
        f<<"]}";
    }
    f<<"]}";
}
}
int main(int argc,char**argv) {
    if(argc!=2){std::cerr<<"usage: mechanism_experiment OUTPUT_DIRECTORY\n";return 2;}
    const std::filesystem::path out(argv[1]);
    require(!std::filesystem::exists(out/"mechanism_scan.csv"),"output already exists; use a new directory");
    std::filesystem::create_directories(out);
    std::ofstream scan(out/"mechanism_scan.csv"),events(out/"mechanism_events.jsonl");
    require(bool(scan)&&bool(events),"cannot open outputs");
    scan<<std::setprecision(17)<<"height_m,reveal_s,transport_s,planned,instantiated,old_clearance_m,new_clearance_m,solve_ms,activation_ms,reason\n";
    events<<std::setprecision(17);
    auto terrain=std::make_shared<Bar>();const auto old=reference();
    SwingTrajectoryQpConfig qp;qp.safetyCorridorEnabled=true;qp.corridorConfig.contactPhase=.25;
    qp.corridorConfig.verticalExtent=.15;qp.checkReference=false;qp.verifyOutput=false;
    SwingReplanConfig cfg;cfg.enabled=true;cfg.correctionDuration=.16;cfg.maxTrajectoryCorrection=.12;
    cfg.maxMarginFraction=.8;cfg.minRemainingTime=.10;
    SwingOnlineReplanner replanner(cfg);
    int admitted=0,repairs=0,total=0;bool saved=false;
    for(double height:{.10,.11,.12,.13})for(double reveal:{.02,.04,.06,.08,.10,.12,.14})for(double delay:{0.,.005,.015,.03}) {
        ++total;terrain->height=height;
        auto normalized=old;normalized.setKnots((old.getKnots()/.4).eval());
        auto corridor=buildTerrainSafetyCorridor(*terrain,normalized,qp.corridorConfig,4,HullBasis::Minvo,{0,1},.4,
            std::max(.25,reveal/.4),.75);
        const auto begin=std::chrono::steady_clock::now();
        const auto planned=replanner.admit(old,corridor,0,reveal,.4,old.getPosition(reveal),old.getVelocity(reveal),old.getAcceleration(reveal));
        const double solveMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
        const double now=reveal+delay;
        SwingReplanDecision instance;
        const auto gate=std::chrono::steady_clock::now();
        if(planned.admitted())instance=replanner.instantiate(planned.candidate,corridor,0,now,.4,
            old.getPosition(now),old.getVelocity(now),old.getAcceleration(now));
        const double gateMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-gate).count();
        const double oldMin=minimum(old,std::max(.1,now),*terrain);
        const double newMin=instance.admitted()?minimum(instance.candidate,std::max(.1,now),*terrain):oldMin;
        scan<<height<<','<<reveal<<','<<delay<<','<<planned.admitted()<<','<<instance.admitted()<<','<<oldMin<<','<<newMin<<','<<solveMs<<','<<gateMs<<','
            <<'"'<<(planned.admitted()?instance.reason:planned.reason)<<'"'<<'\n';
        if(!instance.admitted())continue;
        ++admitted;
        const auto& ref=instance.candidate;
        require((ref.getPosition(now)-old.getPosition(now)).norm()<1e-8,"actual position boundary");
        require((ref.getVelocity(now)-old.getVelocity(now)).norm()<1e-6,"actual velocity boundary");
        require((ref.getAcceleration(now)-old.getAcceleration(now)).norm()<1e-4,"actual allocated acceleration boundary");
        for(int order=0;order<3;++order) {
            const auto end=order==0?ref.getPosition(.4):order==1?ref.getVelocity(.4):ref.getAcceleration(.4);
            const auto target=order==0?old.getPosition(.4):order==1?old.getVelocity(.4):old.getAcceleration(.4);
            require((end-target).norm()<1e-5,"terminal changed");
        }
        if(oldMin<0 && newMin>0)++repairs;
        events<<"{\"evidence\":\"production_algorithm_offline\",\"physical_commit\":false,\"height\":"<<height<<",\"reveal\":"<<reveal
              <<",\"now\":"<<now<<",\"start\":0,\"end\":0.4,\"old_clearance\":"<<oldMin<<",\"new_clearance\":"<<newMin;
        emit(events,"old",old);emit(events,"planned",planned.candidate);emit(events,"new",ref);
        emitCertificate(events,ref,corridor);events<<"}\n";
        if(!saved && oldMin<0 && newMin>0) {
            saved=true;std::ofstream trace(out/"mechanism_trace.csv");trace<<std::setprecision(17);
            trace<<"time_s,old_x,old_z,old_vz,old_az,planned_z,planned_vz,planned_az,new_x,new_z,new_vz,new_az,old_clearance,new_clearance\n";
            for(int i=0;i<=4000;++i){const double t=.4*i/4000.;const bool active=t>=now;
                const auto p=old.getPosition(t);const auto& s=active?ref:old;const auto pn=s.getPosition(t);
                const auto& direct=t>=reveal?planned.candidate:old;
                trace<<t<<','<<p.x()<<','<<p.z()<<','<<old.getVelocity(t).z()<<','<<old.getAcceleration(t).z()<<','
                     <<direct.getPosition(t).z()<<','<<direct.getVelocity(t).z()<<','<<direct.getAcceleration(t).z()<<','
                     <<pn.x()<<','<<pn.z()<<','<<s.getVelocity(t).z()<<','<<s.getAcceleration(t).z()<<','
                     <<distance(p,*terrain)<<','<<distance(pn,*terrain)<<'\n';}
        }
    }
    // Invalid workspace must reject, including a fixed activation outside the interval.
    const auto box=[](){TerrainSafetyCorridor c;c.valid=true;c.start=0;c.end=1;CorridorPolytope cell;cell.start=0;cell.end=1;
        for(int k=0;k<3;++k)for(double sign:{-1.,1.})cell.halfSpaces.push_back({sign*Eigen::Vector3d::Unit(k),3,2.9});c.cells.push_back(cell);return c;}();
    const double t=.1;
    require(!replanner.admit(old,box,0,t,.4,old.getPosition(t),old.getVelocity(t),old.getAcceleration(t),
        [](double,const Eigen::Vector3d&){return Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN()).eval();}).admitted(),"invalid IK admitted");
    require(!replanner.instantiate(old,box,0,t,.4,old.getPosition(t),old.getVelocity(t),Eigen::Vector3d::Constant(NAN)).admitted(),"missing allocated acceleration admitted");
    require(!replanner.admit(old,box,0,t,.4,old.getPosition(t),old.getVelocity(t),old.getAcceleration(t),
        [](double,const Eigen::Vector3d&){return Eigen::Vector2d(-.1,-.05);}).admitted(),"unreachable activation admitted");
    require(!replanner.admit(old,box,0,t,.4,old.getPosition(t),old.getVelocity(t),old.getAcceleration(t),
        [](double time,const Eigen::Vector3d&){return time>.399?Eigen::Vector2d(-.1,-.05):Eigen::Vector2d(-1,1);}).admitted(),
        "unreachable fixed terminal admitted");
    BSpline constant(5);Eigen::VectorXd zeros=Eigen::VectorXd::Zero(6),height=Eigen::VectorXd::Constant(6,.0495);
    constant.setControlPoints(zeros,zeros,height);constant.setKnots(constant.generateKnots(6,0,.4));
    auto tight=box;
    for(auto& plane:tight.cells[0].halfSpaces)if(plane.normal.z()>.5)plane.geometricOffset=plane.planningOffset=.05;
    require(replanner.validate(constant,tight,0,.4),"family fixture nominal must fit");
    require(!replanner.instantiate(constant,tight,0,.1,.4,constant.getPosition(.1),
        constant.getVelocity(.1),constant.getAcceleration(.1)).admitted(),"nonzero activation family escaped its planning face");
    // Both noncontact entry points must retain the target-top obstacle row.
    class Raised final:public TerrainInterface {
      public:
        double getHeight(double,double)const override{return .10;}
        std::vector<TerrainPrism> getPrisms(const Eigen::Vector2d&,const Eigen::Vector2d&)const override{
            return {{{-2,-2},{2,2},.10},{{.4,-.1},{.5,.1},.10}};
        }
    } raised;
    BSpline flat(5);height.setConstant(.20);flat.setControlPoints(zeros,zeros,height);
    flat.setKnots(flat.generateKnots(6,0,1));
    CorridorConfig geometry;geometry.contactPhase=.25;
    const auto full=buildTerrainSafetyCorridor(raised,flat,geometry);
    const auto remaining=buildTerrainSafetyCorridor(raised,flat,geometry,4,HullBasis::Minvo,{},0,.25,.75);
    require(full.valid && remaining.valid && full.cells.size()==remaining.cells.size(),"corridor interval fixture");
    for(size_t i=0;i<full.cells.size();++i){
        require(full.cells[i].halfSpaces.size()==remaining.cells[i].halfSpaces.size(),"mode-dependent target exclusion");
        for(size_t j=0;j<full.cells[i].halfSpaces.size();++j)
            require(std::abs(full.cells[i].halfSpaces[j].planningOffset-remaining.cells[i].halfSpaces[j].planningOffset)<1e-10,
                "full and remaining noncontact target constraints differ");
    }
    // Approach may touch its support, but a neighboring same-height prism
    // still consumes full clearance and tracking margin, in either row order.
    class Landing final:public TerrainInterface {
      public:
        bool neighbor=false,reverse=false;
        double getHeight(double,double)const override{return .1;}
        std::vector<TerrainPrism> getPrisms(const Eigen::Vector2d&,const Eigen::Vector2d&)const override{
            std::vector<TerrainPrism> p{{{-.01,-.1},{.01,.1},.1}};
            if(neighbor)p.push_back({{.021,-.1},{.03,.1},.1});
            if(reverse)std::reverse(p.begin(),p.end());return p;
        }
    } landing;
    auto descent=createSwingHeightBSpline(0,1,.1,0,.2,.1,0);
    auto support=buildTerrainSafetyCorridor(landing,descent,geometry);
    require(support.valid,"raised target noncontact corridor");
    buildLandingCorridor(support,landing,descent,geometry,0,1,.001);
    require(!support.approachCells.empty(),"raised target approach unavailable");
    const auto lower=[](const TerrainSafetyCorridor& c){
        double value=-1e9;
        for(const auto& cell:c.approachCells)for(const auto& p:cell.halfSpaces)
            if(p.normal.z()<-.5)value=std::max(value,-p.planningOffset);
        return value;
    };
    require(lower(support)>=.119-1e-10,"target nonpenetration row removed");
    landing.neighbor=true;
    for(bool reverse:{false,true}){
        landing.reverse=reverse;auto c=buildTerrainSafetyCorridor(landing,descent,geometry);
        buildLandingCorridor(c,landing,descent,geometry,0,1,.001);
        require(!c.approachCells.empty() && lower(c)>.139,
            "neighbor obstacle lost tracking shrink when merging target row");
    }
    std::ofstream summary(out/"mechanism_summary.json");summary<<"{\"cases\":"<<total<<",\"instantiated\":"<<admitted<<",\"collision_repairs\":"<<repairs<<",\"robot_execution\":false}\n";
    std::cout<<"cases="<<total<<" instantiated="<<admitted<<" collision_repairs="<<repairs<<'\n';
    return 0;
}
