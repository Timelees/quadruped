#include <cmath>
#include <iostream>
#include <stdexcept>
#include <ocs2_mpc/MRT_BASE.h>
#include <gz_quadruped_hardware/actuator.hpp>
#include "ocs2_quadruped_controller/interface/constraint/SwingTrajectoryPlanner.h"
#include "ocs2_quadruped_controller/perceptive/synchronize/PlanarTerrainReceiver.h"

using namespace ocs2;
using namespace ocs2::legged_robot;
namespace {
void require(bool value,const char* message) { if (!value) throw std::runtime_error(message); }
class Flat final : public TerrainInterface {
 public:
    double height=0.0;
    double getHeight(double,double) const override { return height; }
    std::vector<TerrainPrism> getPrisms(const Eigen::Vector2d&,const Eigen::Vector2d&) const override {
        return {{{-2,-2},{2,2},height}};
    }
};
class FakeMrt final : public MRT_BASE {
 public:
    void resetMpcNode(const TargetTrajectories&) override {}
    void setCurrentObservation(const SystemObservation&) override {}
    void publish(double marker) {
        auto command=std::make_unique<CommandData>(); command->mpcInitObservation_.time=marker;
        auto solution=std::make_unique<PrimalSolution>();solution->timeTrajectory_={marker};
        moveToBuffer(std::move(command),std::move(solution),std::make_unique<PerformanceIndex>());
    }
};
class Gate final : public MrtObserver {
 public:
    bool allow=true;
    double active=-1;
    bool acceptBufferedSolution(const CommandData&,const PrimalSolution&) override { return allow; }
    void modifyActiveSolution(const CommandData& command,PrimalSolution&) override { active=command.mpcInitObservation_.time; }
};
void mrtCheck() {
    FakeMrt mrt;auto gate=std::make_shared<Gate>();mrt.addMrtObserver(gate);
    mrt.publish(1);require(mrt.updatePolicy(),"initial MRT update");
    gate->allow=false;mrt.publish(2);
    require(!mrt.updatePolicy(),"veto did not reject package");
    require(mrt.getPolicy().timeTrajectory_.front()==1 && mrt.getCommand().mpcInitObservation_.time==1 && gate->active==1,
        "veto partially swapped policy, command or observer");
    require(!mrt.updatePolicy(),"rejected package retried without new data");
    gate->allow=true;mrt.publish(3);require(mrt.updatePolicy() && gate->active==3,"MRT did not recover after veto");
    mrt.setPolicyTransportDelay(.05); mrt.setPolicyTransportTime(10); mrt.publish(4);
    mrt.setPolicyTransportTime(10.049); mrt.publish(5);
    require(!mrt.updatePolicy() && gate->active==3,"transport released package early");
    mrt.setPolicyTransportTime(10.051);
    require(mrt.updatePolicy() && gate->active==4 && mrt.getCommand().mpcInitObservation_.time==4,
        "transport hold mixed policy and observer snapshot");
    require(mrt.policyTransportResidence()>=.05 && mrt.policyTransportDrops()==1,"transport residence/drop accounting");
    gate->allow=false; mrt.publish(6); mrt.setPolicyTransportTime(10.11);
    require(!mrt.updatePolicy() && gate->active==4,"delayed package bypassed actual admission");
    gate->allow=true; mrt.setPolicyTransportDelay(.05,20.0); mrt.publish(7);
    require(mrt.updatePolicy() && gate->active==7,"transport delay started before its configured time");
    mrt.setPolicyTransportTime(20.0); mrt.publish(8);
    require(!mrt.updatePolicy(),"transport start boundary released early");
    mrt.setPolicyTransportTime(20.051);
    require(mrt.updatePolicy() && gate->active==8,"scheduled transport injection did not start");
}
void actuatorCheck() {
    gazebo::Actuator_ motor;
    require(motor.CerrentLoopResponse(1,0,0)==1,"uninitialized first motor sample");
    for (double target : {12.0,-12.0,4.0,-3.0}) {
        const double previous=motor.current(0);
        const double actual=motor.CerrentLoopResponse(motor.GetTorque(target,0),0,0);
        require(std::abs(motor.current(0)-previous)<=3.0+1e-8,"current slew violated");
        require(std::abs(motor.getActuatorI(actual,0)-motor.current(0))<1e-5,"current/torque model mismatch");
    }
    require(motor.GetTorque(12,40)==0 && motor.GetTorque(-12,40)==-12,"TN quadrant bounds");
}
void swingCommitmentCheck() {
    for (bool terrainPlanning : {false, true}) {
        SwingTrajectoryPlanner::Config config; config.useTerrainPlanning = terrainPlanning;
        config.terrainQpConfig.checkReference = false; config.terrainQpConfig.verifyOutput = false;
        SwingTrajectoryPlanner planner(config, 4);
        planner.setTerrainInterface(std::make_shared<Flat>());
        ModeSchedule schedule; schedule.eventTimes = {1.0, 1.4}; schedule.modeSequence = {15, 0, 15};
        feet_array_t<scalar_array_t> heights, landing;
        heights.fill({0, 0, 0}); landing.fill({0, .1, .1});
        feet_array_t<std::vector<Eigen::Vector3d>> positions;
        positions.fill(std::vector<Eigen::Vector3d>(3, Eigen::Vector3d::Zero()));
        const auto update = [&](double now) {
            if (terrainPlanning) planner.updateTerrain(schedule, heights, landing, positions, positions, now, 1.4);
            else planner.update(schedule, heights, landing, now);
        };
        update(.85);
        auto accepted = std::make_shared<SwingTrajectoryPlanner::ExecutionSwings>();
        (*accepted)[0].push_back({createSwingHeightBSpline(1, 1.4, 0, config.liftOffVelocity,
            .1 + config.swingHeight, .1, config.touchDownVelocity), {}, 1, 1.4});
        planner.setActiveExecutionSwings(accepted);
        landing[0][1] = .2;
        update(.95); // an in-flight policy may cross lift-off within the delivery budget
        require(std::abs(planner.getZpositionConstraint(0, std::nextafter(1.4, 1.0)) - .1) < 1e-9,
            "In-flight policy changed the accepted terminal state inside the delivery budget.");
        const double retained = planner.getZpositionConstraint(0, 1.2);
        planner.setActiveExecutionSwings({}); // model the updated reference becoming the last policy
        landing[0][1] = .3;
        update(1.05);
        require(std::abs(planner.getZpositionConstraint(0, 1.2) - retained) < 1e-12,
            "Executing swing changed its fixed terminal state after lift-off.");
    }
}
void mapCheck() {
    auto terrain=std::make_shared<convex_plane_decomposition::PlanarTerrain>();
    auto& map=terrain->gridMap;map.setGeometry(grid_map::Length(1,1),.1);map.add("elevation",0.0);
    map.get("elevation").setConstant(.2);
    PlanarTerrainReceiver::preserveObservationLayer(map,"elevation");
    auto previous=map;previous.get("elevation").setZero();
    PlanarTerrainReceiver::filterHeightLayer(map,previous,"elevation",.5);
    GridMapTerrainInterface query(terrain,"elevation");
    require(query.getHeight(0,0)>=.2-1e-7,"filter reduced a new obstacle's certified height");
    auto frozen=query.snapshot();map.get(certifiedElevationLayer("elevation")).setConstant(.8);
    require(frozen->getHeight(0,0)<.21,"immutable certification map changed");
    map.erase(certifiedElevationLayer("elevation"));
    map.get("elevation").setConstant(std::numeric_limits<float>::quiet_NaN());
    PlanarTerrainReceiver::preserveObservationLayer(map,"elevation");
    map.get("elevation").setZero(); // dense SDF inpainting must not erase unknown evidence
    PlanarTerrainReceiver::preserveObservationLayer(map,"elevation");
    bool rejected=false;
    try { query.getPrisms({-.1,-.1},{.1,.1}); } catch(const TerrainUnavailable&) { rejected=true; }
    require(rejected,"inpainted unknown patch admitted");
}
void activationCheck() {
    SwingTrajectoryPlanner::Config config;config.useTerrainPlanning=true;config.replanConfig.enabled=true;
    config.terrainQpConfig.safetyCorridorEnabled=true;
    config.terrainQpConfig.corridorConfig.contactPhase=.25;
    config.terrainQpConfig.checkReference=false;config.terrainQpConfig.verifyOutput=false;
    auto terrain=std::make_shared<Flat>();
    SwingTrajectoryPlanner planner(config,4);
    auto reference=createSwingHeightBSpline(3,3.4,0,0,.10,0,0);
    auto corridor=planner.certifyReference(reference,3,3.4,3,{terrain,{2.9,4}});
    require(corridor.valid,"activation fixture corridor");
    SwingReplanConfig rc;rc.enabled=true;
    auto decision=SwingOnlineReplanner(rc).admit(reference,corridor,3,3.14,3.4,
        reference.getPosition(3.14),reference.getVelocity(3.14),reference.getAcceleration(3.14));
    require(decision.admitted(),decision.reason.c_str());
    SwingTrajectoryPlanner::ExecutionSwing previous{reference,corridor,3,3.4},candidate{decision.candidate,corridor,3,3.4};
    const double now=3.15;
    const auto p=candidate.reference.getPosition(now)+config.footRadius*Eigen::Vector3d::UnitZ();
    const auto v=candidate.reference.getVelocity(now);
    const auto accepted=planner.checkActivation(candidate,&previous,now,.005,p,v,{terrain,{2.9,4}});
    require(accepted.accepted,accepted.reason.c_str());
    require(!planner.checkActivation(candidate,&previous,now,.005,p+Eigen::Vector3d(.1,0,0),v,{terrain,{2.9,4}}).accepted,
        "actual state mismatch admitted");
    require(!planner.checkActivation(candidate,&previous,now,.005,p,v,{terrain,{2.9,3.1}}).accepted,"expired activation map");
    require(!planner.checkActivation(candidate,&previous,3.26,.005,p,v,{terrain,{2.9,4}}).accepted,"late activation admitted");
    terrain->height=.25;
    require(!planner.checkActivation(candidate,&previous,now,.005,p,v,{terrain,{2.9,4}}).accepted,"new obstacle admitted");
    terrain->height=0;
    SwingTrajectoryQpOptimizer optimizer(config.terrainQpConfig,terrain);
    auto result=optimizer.optimizeSwingTrajectory(reference,3,3.4,reference.getPosition(3),reference.getPosition(3.4));
    require(result.valid(),"physical derivative constrained baseline rejected");
    require(result.derivativeBounds.row(1).maxCoeff()<=config.terrainQpConfig.maxAcceleration+1e-7,"initial acceleration bound");
    config.terrainQpConfig.maxAcceleration=.1;
    require(!SwingTrajectoryQpOptimizer(config.terrainQpConfig,terrain).optimizeSwingTrajectory(reference,3,3.4,
        reference.getPosition(3),reference.getPosition(3.4)).valid(),"initial acceleration bound bypassed");
}
void remainingCorridorCheck() {
    class PastObstacle final : public TerrainInterface {
    public:
        double getHeight(double x, double) const override { return x<.10 ? .4 : 0.; }
        std::vector<TerrainPrism> getPrisms(const Eigen::Vector2d&,const Eigen::Vector2d&) const override {
            return {{{-2,-2},{2,2},0.},{{-.1,-.1},{.10,.1},.4}};
        }
    };
    SwingTrajectoryPlanner::Config config;
    config.terrainQpConfig.corridorConfig.contactPhase=.25;
    auto terrain=std::make_shared<PastObstacle>();
    SwingTrajectoryPlanner planner(config,4);
    auto reference=createSwingHeightBSpline(3,3.4,0,0,.10,0,0);
    auto controls=reference.getControlPoints();
    controls.col(0)=Eigen::VectorXd::LinSpaced(controls.rows(),0.,.3);
    reference.setControlPoints(controls.col(0),controls.col(1),controls.col(2));
    require(!planner.certifyReference(reference,3,3.4,3,{terrain,{2.9,4}}).valid,
        "fixture did not invalidate the past section");
    const auto future=planner.certifyReference(reference,3,3.4,3.26,{terrain,{2.9,4}});
    require(future.valid,future.failureReason.c_str());
    require(future.start>=.65-1e-8,"remaining certificate included a past cell");
}
void heldInputCheck() {
    SwingTrajectoryPlanner::Config planning;
    planning.terrainQpConfig.corridorConfig.contactPhase=.25;
    auto terrain=std::make_shared<Flat>();
    SwingTrajectoryPlanner planner(planning,4);
    const auto reference=createSwingHeightBSpline(3,3.4,0,0,.10,0,0);
    const auto corridor=planner.certifyReference(reference,3,3.4,3,{terrain,{2.9,4}});
    require(corridor.valid,"held input fixture corridor");
    SafetyFilterConfig config;config.enabled=true;config.tubeRate=60;config.tubeSampleRate=0;
    config.positionMeasurementError=0; config.velocityMeasurementError=0;
    SwingSafetyFilter filter(config);
    // The hold crosses the spline's internal knot; the exact reference, rather
    // than a constant-acceleration approximation, must satisfy every tube face.
    const double now=3.198,period=.005;
    SwingSafetyFilter::FootState state;
    state.reference=&reference;state.referenceTime=now;
    state.positionError={.004,0,0};state.velocityError={.06,0,0};
    const Eigen::Vector3d offset=planning.footRadius*Eigen::Vector3d::UnitZ();
    state.position=reference.getPosition(now)+offset+state.positionError;
    state.velocity=reference.getVelocity(now)+state.velocityError;
    state.referenceAcceleration=reference.getAcceleration(now);
    const auto result=filter.filter(corridor,state,state.referenceAcceleration,(now-3)/.4,period,3.4-now);
    require(result.active(),"exact reference hold rejected feasible knot crossing");
    for (int axis=0;axis<3;++axis) for (double sign:{-1.0,1.0}) {
        const Eigen::Vector3d disturbance=sign*config.disturbanceBound*Eigen::Vector3d::Unit(axis);
        const Eigen::Vector3d acceleration=result.acceleration+disturbance;
        for (int sample=0;sample<=100;++sample) {
            const double t=period*sample/100;
            const Eigen::Vector3d p=state.position+t*state.velocity+.5*t*t*acceleration;
            const Eigen::Vector3d v=state.velocity+t*acceleration;
            const Eigen::Vector3d e=p-reference.getPosition(now+t)-offset;
            const Eigen::Vector3d ev=v-reference.getVelocity(now+t);
            const Eigen::Vector3d ea=acceleration-reference.getAcceleration(now+t);
            const auto check=[&](const Eigen::Vector3d& lp,const Eigen::Vector3d& lv,double bound) {
                require(config.tubeRate*(bound-lp.dot(e)-lv.dot(ev))-lp.dot(ev)-lv.dot(ea)>=-1e-7,
                    "tube residual violated inside held input interval");
            };
            for (int i=0;i<3;++i) for (double s:{-1.0,1.0}) {
                const Eigen::Vector3d unit=s*Eigen::Vector3d::Unit(i);
                check(unit,Eigen::Vector3d::Zero(),config.positionBound);
                check(Eigen::Vector3d::Zero(),unit,config.velocityBound);
                for (double vs:{-1.0,1.0}) check(config.couplingGain*unit,vs*Eigen::Vector3d::Unit(i),config.couplingBound);
            }
            for (const auto& cell:corridor.cells) {
                const double phase=(now+t-3)/.4;
                if (phase<cell.start || phase>cell.end) continue;
                for (const auto& plane:cell.halfSpaces) {
                    const double h=plane.geometricOffset-plane.normal.dot(p),hdot=-plane.normal.dot(v);
                    require(-plane.normal.dot(acceleration)+(config.hocbfGain1+config.hocbfGain2)*hdot+
                        config.hocbfGain1*config.hocbfGain2*h>=-1e-7,"HOCBF residual violated during hold");
                }
            }
        }
    }
}
}
int main() {
    try {mrtCheck();actuatorCheck();swingCommitmentCheck();mapCheck();activationCheck();remainingCorridorCheck();heldInputCheck();}
    catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
    std::cout<<"MRT atomic admission / observed terrain / physical derivatives self-check passed\n";
}
