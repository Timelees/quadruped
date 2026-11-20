#include <cmath>
#include <iostream>
#include <stdexcept>
#include "ocs2_quadruped_controller/interface/constraint/SwingOnlineReplanner.h"
#include "ocs2_quadruped_controller/interface/constraint/SwingSafetyFilter.h"
#include "ocs2_quadruped_controller/interface/constraint/SwingTrajectoryPlanner.h"
using namespace ocs2::legged_robot;
namespace {
void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
TerrainSafetyCorridor box() {
    TerrainSafetyCorridor corridor;
    corridor.valid = true; corridor.footRadius = 0.019; corridor.numericalMargin = 1e-7;
    corridor.start=0.0; corridor.end=1.0;
    CorridorPolytope cell; cell.start=0.0; cell.end=1.0;
    for (int axis=0;axis<3;++axis) for (double sign:{-1.0,1.0})
        cell.halfSpaces.push_back({sign*Eigen::Vector3d::Unit(axis),1.0,0.98});
    corridor.cells.push_back(cell);
    return corridor;
}
BSpline reference() {
    Eigen::VectorXd x(7), y=Eigen::VectorXd::Zero(7), z(7);
    x << 0,0,0,0.1,0.2,0.2,0.2;
    z << 0,0.004,0.012,0.2,0.012,0.004,0;
    BSpline result(5); result.setControlPoints(x,y,z); result.setKnots(result.generateKnots(7,3.0,3.4));
    return result;
}
void replannerChecks() {
    auto old=reference(); auto corridor=box();
    SwingReplanConfig config; config.enabled=true;
    SwingOnlineReplanner replanner(config);
    const double t=3.14, end=3.4;
    const Eigen::Vector3d dp(0.001,-0.001,0.001), dv(0.005,0,0.002), da(0.01,-0.01,0.02);
    auto decision=replanner.admit(old,corridor,3.0,t,end,old.getPosition(t)+dp,
        old.getVelocity(t)+dv,old.getAcceleration(t)+da);
    require(decision.admitted(), decision.reason.c_str());
    require((decision.candidate.getPosition(t)-old.getPosition(t)-dp).norm()<1e-9,"position splice");
    require((decision.candidate.getVelocity(t)-old.getVelocity(t)-dv).norm()<1e-7,"velocity splice");
    require((decision.candidate.getAcceleration(t)-old.getAcceleration(t)-da).norm()<1e-5,"acceleration splice");
    require((decision.candidate.getPosition(end)-old.getPosition(end)).norm()<1e-9,"terminal position");
    require((decision.candidate.getVelocity(end)-old.getVelocity(end)).norm()<1e-7,"terminal velocity");
    require((decision.candidate.getAcceleration(end)-old.getAcceleration(end)).norm()<1e-5,"terminal acceleration");
    // Every knot retains C2, even after the QP changes internal Bezier controls.
    for (const auto& segment:extractPolynomialSegments(decision.candidate)) {
        if (segment.end>=end) continue;
        const double left=segment.end-1e-9, right=segment.end+1e-9;
        require((decision.candidate.getPosition(left)-decision.candidate.getPosition(right)).norm()<1e-6,"C0 knot");
        require((decision.candidate.getVelocity(left)-decision.candidate.getVelocity(right)).norm()<1e-5,"C1 knot");
        require((decision.candidate.getAcceleration(left)-decision.candidate.getAcceleration(right)).norm()<1e-3,"C2 knot");
    }
    // A second update still finds and certifies the reduced reference domain.
    auto next=replanner.admit(decision.candidate,corridor,3.0,3.21,end,
        decision.candidate.getPosition(3.21),decision.candidate.getVelocity(3.21),decision.candidate.getAcceleration(3.21));
    require(next.admitted(),next.reason.c_str());
    require(replanner.stats().attempts==2 && replanner.stats().admitted==2,"stats");
    const double early=3.05;
    const auto bounded=replanner.admit(old,corridor,3.0,early,end,old.getPosition(early),
        old.getVelocity(early),old.getAcceleration(early),
        [](double,const Eigen::Vector3d&){return Eigen::Vector2d(-.10,.09);});
    require(bounded.admitted(),bounded.reason.c_str());
    for (int i=0;i<=20;++i)
        require(bounded.candidate.getPosition(early+(end-early)*i/20.).z()<=.09+1e-8,
                "online QP ignored the supplied IK height section");
    const auto unreachable=replanner.admit(old,corridor,3.0,early,end,old.getPosition(early),
        old.getVelocity(early),old.getAcceleration(early),
        [](double,const Eigen::Vector3d&){return Eigen::Vector2d(-.10,-.05);});
    require(!unreachable.admitted(),"fixed activation state violated IK height section");
    auto blocked=corridor;
    blocked.cells[0].halfSpaces[0].planningOffset=-2.;
    require(!replanner.admit(old,blocked,3.0,t,end,old.getPosition(t),old.getVelocity(t),
                            old.getAcceleration(t)).admitted(),"condensation discarded a fixed XY obstacle plane");
    auto invalid=corridor; invalid.valid=false;
    auto rejected=replanner.admit(old,invalid,3.0,t,end,old.getPosition(t),old.getVelocity(t),old.getAcceleration(t));
    require(rejected.outcome==SwingReplanOutcome::RejectedCorridor,"missing certificate admitted");
    // A violation confined between old sampling points must fail hull validation.
    BSpline spike(5); Eigen::VectorXd sx=Eigen::VectorXd::Zero(6), sz=Eigen::VectorXd::Constant(6,0.1);
    sz[2]=8.0; spike.setControlPoints(sx,sx,sz); spike.setKnots(spike.generateKnots(6,3.17,3.1701));
    require(!replanner.validate(spike,corridor,3.0,end),"narrow violation admitted");
    require(SwingOnlineReplanner{}.admit(old,corridor,3.0,t,end,old.getPosition(t),old.getVelocity(t),old.getAcceleration(t)).outcome==SwingReplanOutcome::Disabled,"default enabled");
    std::cout << replanner.stats().json() << '\n';
}
void plannerChecks() {
    class Flat final : public TerrainInterface {
     public:
        bool available=true,bar=false;
        double getHeight(double x,double y) const override {
            if (!available) throw TerrainUnavailable("injected map loss");
            return bar && std::abs(x-.2)<=.015 && std::abs(y)<=.5 ? .04 : 0.0;
        }
        std::vector<TerrainPrism> getPrisms(const Eigen::Vector2d&,const Eigen::Vector2d&) const override {
            if (!available) throw TerrainUnavailable("injected map loss");
            std::vector<TerrainPrism> prisms{{{-2,-2},{2,2},0.0}};
            if (bar) prisms.push_back({{.185,-.5},{.215,.5},.04});
            return prisms;
        }
    };
    SwingTrajectoryPlanner::Config config;
    config.useTerrainPlanning=true;
    config.liftOffVelocity=.10;
    config.touchDownVelocity=-.03;
    config.swingHeight=.075;
    // Exercise the production control count and working-set budget.
    config.terrainQpConfig.numControlPoints=15;
    config.terrainQpConfig.safetyCorridorEnabled=true;
    config.terrainQpConfig.corridorConfig.contactPhase=.25;
    config.terrainQpConfig.checkReference=false;
    config.terrainQpConfig.verifyOutput=false;
    config.terrainQpConfig.verbose=true;
    config.replanConfig.enabled=true;
    config.replanConfig.activationDelay=.02;
    config.replanConfig.maxTrajectoryCorrection=.12;
    config.replanConfig.correctionDuration=.12;
    config.replanConfig.maxMarginFraction=.8;
    auto terrain=std::make_shared<Flat>();
    SwingTrajectoryPlanner planner(config,4); planner.setTerrainInterface(terrain);
    planner.setOnlineReplanningReleased(false);
    require(!planner.isOnlineReplanningEnabled(),"test release gate did not freeze pre-intervention updates");
    planner.setOnlineReplanningReleased(true);
    require(planner.isOnlineReplanningEnabled(),"test release gate did not restore configured method");
    auto frozenConfig=config;frozenConfig.replanConfig.enabled=false;
    SwingTrajectoryPlanner frozen(frozenConfig,4);frozen.setOnlineReplanningReleased(true);
    require(!frozen.isOnlineReplanningEnabled(),"release enabled frozen baseline");
    planner.setMapTiming({.9,2.0});
    ocs2::ModeSchedule schedule;
    schedule.eventTimes={1.0,1.4}; schedule.modeSequence={15,0,15};
    feet_array_t<ocs2::scalar_array_t> heights; heights.fill({0,0,0});
    feet_array_t<std::vector<Eigen::Vector3d>> positions;
    positions.fill(std::vector<Eigen::Vector3d>(3,Eigen::Vector3d::Zero()));
    auto touchdowns=positions;
    for (auto& leg:touchdowns) for (auto& point:leg) point.x()=.3;
    planner.updateTerrain(schedule,heights,heights,positions,touchdowns,.95,1.5);
    require(planner.getTerrainCorridor(0,1.1).valid,"initial corridor missing");
    const auto old=planner.executionSwings()->at(0).at(0).reference;
    SwingTrajectoryPlanner imminent(config,4);imminent.setTerrainInterface(terrain);
    imminent.setMapTiming({.9,2.0});
    imminent.updateTerrain(schedule,heights,heights,positions,touchdowns,.95,1.5);
    imminent.setActiveExecutionSwings(imminent.executionSwings());
    auto shifted=positions;
    for (auto& leg:shifted) for (auto& point:leg) point.y()+=.03;
    imminent.updateTerrain(schedule,heights,heights,shifted,touchdowns,.98,1.5);
    require((imminent.executionSwings()->at(0).at(0).reference.getPosition(1.4)-old.getPosition(1.4)).norm()<1e-10,
            "policy in transit changed an imminent swing terminal");
    SwingTrajectoryPlanner::SwingFootState p,v,a;
    a.fill(Eigen::Vector3d::Constant(NAN));
    p.fill(old.getPosition(1.1));v.fill(old.getVelocity(1.1));
    SwingTrajectoryPlanner quiet(config,4);quiet.setTerrainInterface(terrain);quiet.setMapTiming({.9,2.0});
    quiet.updateTerrain(schedule,heights,heights,positions,touchdowns,.95,1.5);
    quiet.updateTerrain(schedule,heights,heights,positions,touchdowns,1.1,1.5,p,v,a);
    require(quiet.getReplanStats().attempts==0,"certified reference with tracked state requested an unnecessary QP");
    require((quiet.getPositionConstraint(0,1.12)-old.getPosition(1.12)-Eigen::Vector3d(0,0,config.footRadius)).norm()<1e-10,
            "quiet tracking changed the reference");
    p.fill(old.getPosition(1.17)+Eigen::Vector3d(.025,0,0));v.fill(old.getVelocity(1.17));
    quiet.updateTerrain(schedule,heights,heights,positions,touchdowns,1.17,1.5,p,v,a);
    require(quiet.getReplanStats().attempts==0,"tracking error replanned a map-certified reference");
    p.fill(old.getPosition(1.1));v.fill(old.getVelocity(1.1));
    terrain->bar=true;
    planner.updateTerrain(schedule,heights,heights,positions,touchdowns,1.1,1.5,p,v,a);
    require(!planner.getTerrainCorridor(0,1.1).valid,"new obstacle did not invalidate the old reference");
    require((planner.getPositionConstraint(0,1.125)-old.getPosition(1.125)-
        Eigen::Vector3d(0,0,config.footRadius)).norm()<1e-10,"candidate activated before delay");
    require(planner.getReplanStats().attempts==0,"prediction counted as activation");
    p.fill(old.getPosition(1.125)+Eigen::Vector3d(.0005,0,.0005));v.fill(old.getVelocity(1.125));
    planner.updateTerrain(schedule,heights,heights,positions,touchdowns,1.125,1.5,p,v,a);
    require(planner.getReplanStats().admitted==4,"measured activation not admitted");
    require((planner.getPositionConstraint(0,1.125)-p[0]-
        Eigen::Vector3d(0,0,config.footRadius)).norm()<1e-8,
            "activation did not use the fresh state");
    const auto before=planner.getPositionConstraint(0,1.21);
    terrain->available=false;
    p.fill(old.getPosition(1.2));v.fill(old.getVelocity(1.2));
    planner.updateTerrain(schedule,heights,heights,positions,touchdowns,1.2,1.5,p,v,a);
    require((planner.getPositionConstraint(0,1.21)-before).norm()<1e-12,"rejection replaced the old reference");
    require(planner.getReplanStats().attempts==planner.getReplanStats().admitted+4,"activation rejection stats");
    terrain->available = true;
    const int attempts=planner.getReplanStats().attempts;
    for (size_t leg=0;leg<4;++leg) planner.rejectExecutionUpdate(leg,1.4);
    planner.updateTerrain(schedule,heights,heights,positions,touchdowns,1.27,1.5,p,v,a);
    require(planner.getReplanStats().attempts==attempts,"actual veto repeatedly proposed the rejected swing");
    require((planner.getPositionConstraint(0,1.28)-planner.executionSwings()->at(0).at(0).reference.getPosition(1.28)-
        Eigen::Vector3d(0,0,config.footRadius)).norm()<1e-10,"veto recovery policy/reference mismatch");
    SwingTrajectoryPlanner recovering(config,4);
    recovering.setTerrainInterface(terrain); recovering.setMapTiming({.9,2.0});
    terrain->bar=false;
    recovering.updateTerrain(schedule,heights,heights,positions,touchdowns,.95,1.5);
    terrain->bar=true;
    for (size_t leg=0;leg<4;++leg) recovering.rejectExecutionUpdate(leg,1.4);
    p.fill(old.getPosition(1.1));v.fill(old.getVelocity(1.1));
    recovering.updateTerrain(schedule,heights,heights,positions,touchdowns,1.1,1.5,p,v,a);
    require(recovering.getReplanStats().attempts==0,"veto recovery skipped accepted-reference MPC cycle");
    p.fill(old.getPosition(1.17));v.fill(old.getVelocity(1.17));
    recovering.updateTerrain(schedule,heights,heights,positions,touchdowns,1.17,1.5,p,v,a);
    p.fill(old.getPosition(1.195));v.fill(old.getVelocity(1.195));
    recovering.updateTerrain(schedule,heights,heights,positions,touchdowns,1.195,1.5,p,v,a);
    require(recovering.getReplanStats().attempts>0,"one activation veto froze the entire remaining swing");
    SwingTrajectoryPlanner early(config,4);
    early.setTerrainInterface(terrain);early.setMapTiming({.9,2.0});
    terrain->bar=false;
    early.updateTerrain(schedule,heights,heights,positions,touchdowns,.95,1.5);
    terrain->bar=true;
    p.fill(old.getPosition(1.04));v.fill(old.getVelocity(1.04));
    early.updateTerrain(schedule,heights,heights,positions,touchdowns,1.04,1.5,p,v,a);
    require(early.getReplanStats().attempts==0,"early prediction counted as activation");
    p.fill(old.getPosition(1.065));v.fill(old.getVelocity(1.065));
    early.updateTerrain(schedule,heights,heights,positions,touchdowns,1.065,1.5,p,v,a);
    require(early.getReplanStats().admitted==4,"waiting for noncontact phase suppressed a feasible early update");
    require((early.getPositionConstraint(0,1.065)-p[0]-Eigen::Vector3d(0,0,config.footRadius)).norm()<1e-8,
            "early activation did not use the fresh state");
    const auto earlyReference=early.executionSwings()->at(0).at(0).reference;
    require((earlyReference.getPosition(1.4)-old.getPosition(1.4)).norm()<1e-10,
            "early update changed the fixed touchdown target");
    require(early.getTerrainCorridor(0,1.065).start>=.25-1e-10,
            "early planning relaxed the noncontact certificate interval");
    const auto terminalReference = reference();
    terrain->bar=false;
    require(terminalReference.getVelocity(3.4).z() < -0.01, "terminal fixture has no descent");
    SwingTrajectoryQpOptimizer terminalBaseline(config.terrainQpConfig, terrain);
    require(terminalBaseline.optimizeSwingTrajectory(terminalReference, 3.0, 3.4,
                terminalReference.getPosition(3.0), terminalReference.getPosition(3.4)).solved,
            "terminal baseline was already infeasible");
    config.terrainQpConfig.landingMaxDownSpeed = 0.001;
    SwingTrajectoryQpOptimizer strictTerminal(config.terrainQpConfig, terrain);
    require(!strictTerminal.optimizeSwingTrajectory(terminalReference, 3.0, 3.4,
                terminalReference.getPosition(3.0), terminalReference.getPosition(3.4)).solved,
            "fixed terminal rows bypassed the touchdown velocity window");
}
void obstacleUpdateChecks() {
    class Bar final : public TerrainInterface {
     public:
        double getHeight(double x,double y) const override {
            return std::abs(x-.20)<=.015 && std::abs(y)<=.5 ? .06 : 0.;
        }
        std::vector<TerrainPrism> getPrisms(const Eigen::Vector2d&,const Eigen::Vector2d&) const override {
            return {{{-2,-2},{2,2},0.},{{.185,-.5},{.215,.5},.06}};
        }
    } terrain;
    auto old=createSwingHeightBSpline(0.,.4,0.,.10,.075,0.,-.03);
    Eigen::VectorXd x(7);x<<0,0,0,.15,.3,.3,.3;
    old.setControlPoints(x,Eigen::VectorXd::Zero(7),old.getControlPoints().col(2));
    auto normalized=old;normalized.setKnots((old.getKnots()/.4).eval());
    CorridorConfig geometry;geometry.contactPhase=.25;geometry.trackingMargin=.02;
    const auto corridor=buildTerrainSafetyCorridor(terrain,normalized,geometry,4,HullBasis::Minvo,{.1,2.},.4,.25,.75);
    SwingReplanConfig config;config.enabled=true;config.maxPositionDeviation=.04;
    config.maxTrajectoryCorrection=.12;config.correctionDuration=.12;config.minRemainingTime=.12;
    config.maxMarginFraction=.8;
    SwingOnlineReplanner replanner(config);
    bool rescued=false;
    for (double phase:{.25,.375,.5,.625}) {
        const double t=.4*phase;
        auto decision=replanner.admit(old,corridor,0.,t,.4,old.getPosition(t),old.getVelocity(t),old.getAcceleration(t));
        if (!decision.admitted()) continue;
        double oldMinimum=1.,newMinimum=1.;
        for (int i=0;i<=2000;++i) {
            const double time=t+(.3-t)*i/2000.;
            for (int which=0;which<2;++which) {
                const auto p=(which ? decision.candidate : old).getPosition(time);
                const double dx=std::max(std::abs(p.x()-.20)-.015,0.);
                const double dz=std::max(p.z()+.019-.06,0.);
                auto& minimum=which ? newMinimum : oldMinimum;
                minimum=std::min(minimum,std::hypot(dx,dz)-.019);
            }
        }
        rescued=rescued || (oldMinimum< -1e-5 && newMinimum>=0.);
        require(replanner.validate(decision.candidate,corridor,0.,.4),"rescued reference uncertified");
    }
    require(rescued,"online refinement did not repair a colliding nominal swing");
    const auto outside=replanner.admit(old,corridor,0.,.1,.4,
        old.getPosition(.1)+Eigen::Vector3d(0,0,.05),old.getVelocity(.1),old.getAcceleration(.1));
    require(outside.outcome==SwingReplanOutcome::RejectedDeviation,"trajectory budget relaxed state gate");
}
void filterChecks() {
    auto corridor=box();
    SafetyFilterConfig config; config.enabled=true;
    SwingSafetyFilter filter(config); SwingSafetyFilter::FootState state;
    auto zero=filter.filter(corridor,state,Eigen::Vector3d::Zero(),0.5,0.002);
    require(zero.active() && zero.correction.norm()<1e-8,"legal zero input changed");
    auto boundedConfig = config; boundedConfig.tubeEnabled = false; boundedConfig.maxCorrection = 1.0;
    state.correctionCentre = Eigen::Vector3d::Zero();
    const auto bounded = SwingSafetyFilter(boundedConfig).filter(
        corridor, state, Eigen::Vector3d(4, 0, 0), .5, .002);
    require(bounded.active() && std::abs(bounded.acceleration.x()-1.0)<1e-7,
        "Input budget was centred on the tracking target instead of the feasible WBC output.");
    require(bounded.accelerationB.minCoeff() >= -1e-8,
        "Input budget excluded an already safe feasible WBC output.");
    state.correctionCentre.reset();
    // Same acceleration on actual and reference has zero error acceleration.
    state.referenceAcceleration=Eigen::Vector3d(0,0,8);
    auto moving=filter.filter(corridor,state,state.referenceAcceleration,0.5,0.002);
    require(moving.active() && moving.correction.norm()<1e-8,"reference acceleration omitted from tube");
    auto pushed=filter.filter(corridor,state,Eigen::Vector3d(0,0,-10),0.5,0.002);
    require(pushed.active() && pushed.correction.z()>1.0,"tube did not resist downward input");
    require((pushed.accelerationA*pushed.acceleration-pushed.accelerationB).maxCoeff()<1e-6,"filter residual");
    state.position.x()=1.01;
    require(filter.filter(corridor,state,Eigen::Vector3d::Zero(),0.5,0.002).status==SafetyFilterStatus::BarrierEntry,"violated barrier discarded");
    state.position.setZero(); state.positionError.x()=0.02;
    const auto outsideTube = filter.filter(corridor,state,Eigen::Vector3d::Zero(),0.5,0.002);
    require(outsideTube.tubeStatus==SafetyFilterStatus::TubeEntry && !outsideTube.active(),
            "full contract silently dropped the position tube");
    const auto geometryOnly = filter.filterExecution(corridor,state,Eigen::Vector3d::Zero(),0.5,0.002);
    require(geometryOnly.status==SafetyFilterStatus::GeometryOnly &&
            geometryOnly.tubeStatus==SafetyFilterStatus::TubeEntry,
            "tube failure disabled feasible geometry or was mislabeled as full contract");
    require((geometryOnly.accelerationA*geometryOnly.acceleration-geometryOnly.accelerationB).maxCoeff()<1e-6,
            "geometry fallback did not retain hard rows");
    state.position.x()=1.01;
    require(!filter.filterExecution(corridor,state,Eigen::Vector3d::Zero(),0.5,0.002).active(),
            "geometry fallback bypassed a violated barrier entry");
    state.position.setZero();
    state.positionError.setZero(); state.velocityError.x()=0.31;
    require(filter.filter(corridor,state,Eigen::Vector3d::Zero(),0.5,0.002).tubeStatus==SafetyFilterStatus::TubeEntry,"velocity tube omitted");
    state.velocityError.setZero();
    require(filter.filter(corridor,state,Eigen::Vector3d::Zero(),0.5,0.01).status==SafetyFilterStatus::InvalidPeriod,"sampling budget ignored");
    // Disjoint future cells are not simultaneous constraints.
    corridor.cells[0].end=0.6;
    auto future=corridor.cells[0]; future.start=0.6; future.end=1.0;
    future.halfSpaces[0].geometricOffset=-2.0; future.halfSpaces[0].planningOffset=-2.02;
    corridor.cells.push_back(future);
    require(filter.filter(corridor,state,Eigen::Vector3d::Zero(),0.5,0.002).active(),"future cell applied early");
    // The terminal guard keeps the non-target lateral plane and rejects an
    // unreachable normal velocity even when the corridor itself is feasible.
    state.referenceAcceleration.setZero();
    auto landing = box();
    landing.end = 0.75;
    landing.cells[0].end = landing.end;
    landing.approachCells = {landing.cells[0]};
    landing.approachCells[0].start = landing.end;
    landing.approachCells[0].end = 1.0;
    require(filter.filter(landing,state,Eigen::Vector3d::Zero(),0.9,0.005,0.04).status ==
        SafetyFilterStatus::LandingSolved, "valid terminal entry rejected");
    state.position.x() = 1.01;
    require(filter.filter(landing,state,Eigen::Vector3d::Zero(),0.9,0.005,0.04).status ==
        SafetyFilterStatus::BarrierEntry, "terminal guard dropped a non-target obstacle");
    state.position.setZero();
    state.velocity.z() = -1.0;
    require(filter.filter(landing,state,Eigen::Vector3d::Zero(),0.99,0.005,0.004).status ==
        SafetyFilterStatus::Infeasible, "terminal guard ignored unreachable touchdown velocity");
    auto geometryConfig = config;
    geometryConfig.tubeEnabled = false;
    SwingSafetyFilter geometryFilter(geometryConfig);
    state.position = Eigen::Vector3d(.01, -.02, .08);
    state.velocity = Eigen::Vector3d(.05, -.02, -.1);
    landing.touchdownCentre = state.position + .04 * state.velocity;
    state.referenceAcceleration = Eigen::Vector3d::Constant(600.);
    const auto positive = geometryFilter.filter(landing,state,Eigen::Vector3d::Zero(),.9,.005,.04);
    state.referenceAcceleration = Eigen::Vector3d::Constant(-600.);
    const auto negative = geometryFilter.filter(landing,state,Eigen::Vector3d::Zero(),.9,.005,.04);
    require(positive.active() && negative.active(), "exact terminal target became unreachable");
    require((positive.accelerationA-negative.accelerationA).norm()<1e-12 &&
            (positive.accelerationB-negative.accelerationB).norm()<1e-12,
            "terminal guard extrapolated current quintic reference acceleration");
    require(SwingSafetyFilter{}.filter(corridor,state,Eigen::Vector3d::Zero(),0.5,0.002).status==SafetyFilterStatus::Disabled,"filter default enabled");
}
void experimentSweep() {
    using Eigen::Vector3d;
    struct UpdatedTerrain final : TerrainInterface {
        double getHeight(double, double) const override { return 0.0; }
        std::vector<TerrainPrism> getPrisms(const Eigen::Vector2d& lower, const Eigen::Vector2d& upper) const override {
            std::vector<TerrainPrism> prisms{{{-2,-2},{2,2},0.0}};
            const TerrainPrism obstacle{{0.09,-0.04},{0.11,0.04},0.08};
            if ((lower.array() <= obstacle.maxXY.array()).all() &&
                (upper.array() >= obstacle.minXY.array()).all()) prisms.push_back(obstacle);
            return prisms;
        }
    };
    auto old = reference();
    auto controls = old.getControlPoints();
    controls(3, 2) = 0.30;
    old.setControlPoints(controls.col(0), controls.col(1), controls.col(2));
    auto normalized = old;
    normalized.setKnots(normalized.generateKnots(old.getControlPoints().rows(), 0.0, 1.0));
    CorridorConfig corridorConfig;
    corridorConfig.contactPhase = 0.25;
    const auto corridor = buildTerrainSafetyCorridor(UpdatedTerrain{}, normalized, corridorConfig);
    require(corridor.valid, corridor.failureReason.c_str());
    SwingReplanConfig config;
    config.enabled = true;
    config.correctionDuration = 0.12;
    config.maxMarginFraction = 0.8;
    config.minRemainingTime = 0.12;
    std::cout << "phase,position_shift_mm,velocity_shift_mps,delay_ms,outcome,old_valid,splice_mm\n";
    for (double phase : {0.2, 0.4, 0.6, 0.8}) for (double shift : {-0.02, -0.01, 0., 0.01, 0.02})
        for (double dv : {0., 0.1}) for (double delay : {0., 0.02, 0.04}) {
            SwingOnlineReplanner replanner(config);
            const double activation = 3.0 + 0.4*phase + delay;
            const Vector3d p = old.getPosition(activation) + (shift + delay*dv)*Vector3d::UnitZ();
            const Vector3d v = old.getVelocity(activation) + dv*Vector3d::UnitZ();
            const auto result = replanner.admit(old, corridor, 3.0, activation, 3.4,
                                               p, v, old.getAcceleration(activation));
            const double splice = result.admitted() ? 1000*(result.candidate.getPosition(activation)-p).norm() : NAN;
            std::cout << phase << ',' << shift*1000 << ',' << dv << ',' << 1000*delay << ','
                      << static_cast<int>(result.outcome) << ',' << replanner.validate(old,corridor,3.0,3.4)
                      << ',' << splice << '\n';
        }
}

void disturbanceSweep() {
    using Eigen::Vector3d;
    auto corridor = box();
    corridor.cells.front().halfSpaces.push_back({-Vector3d::UnitZ(), 0., -0.02});
    const double dt = 0.005;
    std::cout << "mode,pulse_mps2,minimum_clearance_mm,peak_error_mm,corrected_samples,tube_samples,uncertified_samples\n";
    for (int mode = 0; mode <= 2; ++mode) for (double pulse : {0.5, 1., 2., 4.}) {
        SafetyFilterConfig config;
        config.enabled = (mode != 0);
        config.tubeEnabled = (mode == 2);
        config.disturbanceBound = pulse;
        SwingSafetyFilter filter(config);
        const Vector3d desired(0, 0, 0.025);
        Vector3d p = desired, v = Vector3d::Zero();
        double lowest = p.z(), peakError = 0.0;
        int corrected = 0, tube = 0, uncertified = 0;
        for (int k = 0; k < 80; ++k) {
            SwingSafetyFilter::FootState state;
            state.position = p;
            state.velocity = v;
            state.positionError = p - desired;
            state.velocityError = v;
            const Vector3d nominal = -80.0 * state.positionError - 12.0*v;
            const auto result = filter.filter(corridor, state, nominal, (k+0.5)/80.0, dt);
            Vector3d applied = result.active() ? result.acceleration : nominal;
            const double zDisturbance = (k >= 20 && k < 32) ? -pulse : 0.;
            applied.z() += zDisturbance;
            p += dt*v + 0.5*dt*dt*applied;
            v += dt*applied;
            lowest = std::min(lowest, p.z());
            peakError = std::max(peakError, (p-desired).norm());
            corrected += result.active() && result.correction.norm() > 1e-5;
            tube += result.active() && result.tubeStatus == SafetyFilterStatus::Solved;
            uncertified += config.enabled && !result.active();
        }
        std::cout << mode << ',' << pulse << ',' << 1000*lowest << ',' << 1000*peakError
                  << ',' << corrected << ',' << tube << ',' << uncertified << '\n';
    }
}

}
int main(int argc, char* argv[]) {
    if (argc == 2 && std::string(argv[1]) == "--admission-sweep") { experimentSweep(); return 0; }
    if (argc == 2 && std::string(argv[1]) == "--disturbance-sweep") { disturbanceSweep(); return 0; }
    try { replannerChecks(); plannerChecks(); obstacleUpdateChecks(); filterChecks(); }
    catch(const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
    std::cout << "swing replan / safety filter self-check passed\n";
}
