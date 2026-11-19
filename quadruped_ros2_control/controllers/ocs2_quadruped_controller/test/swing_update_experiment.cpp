// Controlled map revelation using the production planner/QP. This is a
// trajectory mechanism experiment, independent of Gazebo/full-body execution.
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include "ocs2_quadruped_controller/interface/constraint/SwingTrajectoryPlanner.h"
using namespace ocs2::legged_robot;
namespace {
class RevealedTerrain final : public TerrainInterface {
 public:
    double height=.09;
    double getHeight(double x,double y) const override {
        return std::abs(x-.18)<.015 && std::abs(y)<.4 ? height : 0.0;
    }
    std::vector<TerrainPrism> getPrisms(const Eigen::Vector2d&,const Eigen::Vector2d&) const override {
        return {{{-2,-2},{2,2},0.0},{{.165,-.4},{.195,.4},height}};
    }
};
BSpline curve(double peak) {
    auto reference=createSwingHeightBSpline(0,.4,0,0,peak,0,0);
    const auto controls=reference.getControlPoints();
    Eigen::VectorXd x(7);x<<0,0,0,.15,.3,.3,.3;
    reference.setControlPoints(x,Eigen::VectorXd::Zero(7),controls.col(2));
    return reference;
}
// Independent geometry oracle for the known plane and axis-aligned box.
// Dense evaluation is reported as numerical evidence, not a formal certificate.
double clearance(const BSpline& reference,double after,double height) {
    double minimum=1.0;
    constexpr double radius=.019;
    for (double t=std::max(after,.10);t<=.30;t+=.0001) {
        auto p=reference.getPosition(t);p.z()+=radius;
        const double dx=std::max({.165-p.x(),p.x()-.195,0.0});
        const double dy=std::max(std::abs(p.y())-.4,0.0);
        const double dz=std::max(p.z()-height,0.0);
        const double box=std::sqrt(dx*dx+dy*dy+dz*dz)-radius;
        minimum=std::min({minimum,p.z()-radius,box});
    }
    return minimum;
}
}
int main(int argc,char** argv) {
    if (argc!=3) {std::cerr<<"usage: swing_update_experiment scan.csv trace.csv\n";return 2;}
    std::ofstream scan(argv[1]),trace(argv[2]);
    if (!scan || !trace) return 2;
    scan<<std::setprecision(12)<<"phase,state_shift_mm,delay_ms,height_m,method,admitted,reference_safe,old_valid,splice_mm,velocity_jump_mps,minimum_clearance_mm,compute_ms,outcome,velocity_upper_bound_mps,acceleration_upper_bound_mps2,jerk_upper_bound_mps3,touchdown_time_error_s,terminal_position_error_m,terminal_velocity_error_mps,terminal_acceleration_error_mps2\n";
    trace<<std::setprecision(12)<<"phase,state_shift_mm,delay_ms,height_m,method,time_s,x_m,z_m,vz_mps,accepted\n";
    auto terrain=std::make_shared<RevealedTerrain>();
    const auto old=curve(.10);
    SwingTrajectoryQpConfig qp;qp.safetyCorridorEnabled=true;qp.corridorConfig.contactPhase=.25;
    qp.checkReference=false;qp.verifyOutput=false;
    SwingReplanConfig config;config.enabled=true;config.maxPositionDeviation=.06;
    config.maxMarginFraction=.8;config.correctionDuration=.12;
    for (double height:{.06,.08,.10}) {
        terrain->height=height;
        auto normalized=old; normalized.setKnots(old.generateKnots(7,0,1));
        const auto corridor=buildTerrainSafetyCorridor(*terrain,normalized,qp.corridorConfig);
        const auto direct=SwingTrajectoryQpOptimizer(qp,terrain).optimizeSwingTrajectory(curve(.17),0,.4,
            old.getPosition(0),old.getPosition(.4));
        for (double phase:{.25,.35,.45,.55,.65}) for (double shift:{-.01,0.0,.01}) for (double delay:{0.0,.02,.04}) {
            const double activation=.4*phase+delay;
            const Eigen::Vector3d p=old.getPosition(activation)+shift*Eigen::Vector3d::UnitZ();
            const auto v=old.getVelocity(activation),a=old.getAcceleration(activation);
            const auto before=std::chrono::steady_clock::now();
            auto decision=SwingOnlineReplanner(config).admit(old,corridor,0,activation,.4,p,v,a);
            const double elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-before).count();
            for (int method=0;method<3;++method) {
                const bool accepted=method==0 || (method==1 ? direct.valid() : decision.admitted());
                const BSpline& candidate=method==0 ? old : method==1 && direct.valid() ? *direct.bspline :
                    method==2 && decision.admitted() ? decision.candidate : old;
                const double splice=(candidate.getPosition(activation)-p).norm();
                const double jump=(candidate.getVelocity(activation)-v).norm();
                const double minimum=clearance(candidate,activation,height);
                const bool oldValid=SwingOnlineReplanner(config).validate(old,corridor,0,.4);
                const bool safe=accepted && minimum>=-1e-7;
                const auto derivatives=swingDerivativeBounds(candidate);
                const auto knots=candidate.getKnots();
                scan<<phase<<','<<1000*shift<<','<<1000*delay<<','<<height<<','<<method<<','<<accepted<<','<<safe<<','
                    <<oldValid<<','<<1000*splice<<','<<jump<<','<<1000*minimum<<','<<(method==2 ? elapsed : 0.0)<<','
                    <<(method==2 ? static_cast<int>(decision.outcome) : -1)<<','
                    <<derivatives.row(0).maxCoeff()<<','<<derivatives.row(1).maxCoeff()<<','<<derivatives.row(2).maxCoeff()<<','
                    <<std::abs(knots[knots.size()-1]-.4)<<','<<(candidate.getPosition(.4)-old.getPosition(.4)).norm()<<','
                    <<(candidate.getVelocity(.4)-old.getVelocity(.4)).norm()<<','
                    <<(candidate.getAcceleration(.4)-old.getAcceleration(.4)).norm()<<'\n';
                if (phase==.35 && shift==.01 && delay==.02) {
                    for (double t=activation;t<=.4+1e-8;t+=.001) {
                        const auto point=candidate.getPosition(t);
                        trace<<phase<<','<<1000*shift<<','<<1000*delay<<','<<height<<','<<method<<','<<t<<','
                            <<point.x()<<','<<point.z()<<','<<candidate.getVelocity(t).z()<<','<<accepted<<'\n';
                    }
                }
            }
        }
    }
    std::cout<<"Controlled terrain-revelation scan exported (trajectory experiment, not full-body simulation).\n";
}
