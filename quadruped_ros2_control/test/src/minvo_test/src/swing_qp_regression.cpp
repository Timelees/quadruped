// Regression checks against the production optimizer (no copied QP implementation).
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <memory>
#include <chrono>
#include <limits>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <ocs2_quadruped_controller/perceptive/synchronize/PlanarTerrainReceiver.h>
#include <ocs2_quadruped_controller/interface/constraint/SwingTrajectoryQPOptimizer.h>
#include <ocs2_quadruped_controller/interface/constraint/SwingTrajectoryPlanner.h>
#include <ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h>

using namespace ocs2::legged_robot;

void require(bool condition, const char* message) {
    if (!condition) { throw std::runtime_error(message); }
}

template <typename Function>
void requireThrows(Function function, const char* message) {
    try { function(); } catch (const std::exception&) { return; }
    throw std::runtime_error(message);
}

// A curved workspace: moving x requires raising z, even though the vertical
// section at the new x is reachable. Exercise the production section search.
class CurvedWorkspaceKinematics final : public ocs2::EndEffectorKinematics<double> {
public:
    CurvedWorkspaceKinematics* clone() const override { return new CurvedWorkspaceKinematics; }
    const std::vector<std::string>& getIds() const override {
        static const std::vector<std::string> ids{"foot"};
        return ids;
    }
    std::vector<vector3_t> getPosition(const ocs2::vector_t& state) const override {
        const vector3_t q = state.tail<3>();
        return {{q.x(), q.y(), -1.0 + q.head<2>().squaredNorm() + q.z()}};
    }
    std::vector<ocs2::VectorFunctionLinearApproximation> getPositionLinearApproximation(const ocs2::vector_t& state) const override {
        ocs2::VectorFunctionLinearApproximation result;
        result.f = getPosition(state).front();
        result.dfdx = ocs2::matrix_t::Zero(3, state.size());
        result.dfdx.rightCols<3>() << 1, 0, 0, 0, 1, 0,
            2 * state[state.size() - 3], 2 * state[state.size() - 2], 1;
        return {result};
    }
    std::vector<vector3_t> getVelocity(const ocs2::vector_t&, const ocs2::vector_t&) const override { throw std::logic_error("unused"); }
    std::vector<vector3_t> getOrientationError(const vector_t&, const std::vector<quaternion_t>&) const override { throw std::logic_error("unused"); }
    std::vector<ocs2::VectorFunctionLinearApproximation> getVelocityLinearApproximation(const ocs2::vector_t&, const ocs2::vector_t&) const override { throw std::logic_error("unused"); }
    std::vector<ocs2::VectorFunctionLinearApproximation> getOrientationErrorLinearApproximation(const ocs2::vector_t&, const std::vector<quaternion_t>&) const override { throw std::logic_error("unused"); }
};

class WorkspaceReferenceManager final : public PerceptiveLeggedReferenceManager {
public:
    using PerceptiveLeggedReferenceManager::PerceptiveLeggedReferenceManager;
    using PerceptiveLeggedReferenceManager::swingHeightBounds;
    using PerceptiveLeggedReferenceManager::swingNominalState;
};

void checkWorkspaceSeed() {
    ocs2::CentroidalModelInfo info{};
    info.actuatedDofNum = 3;
    info.stateDim = 15;
    info.numThreeDofContacts = 1;
    SwingTrajectoryPlanner::Config config;
    config.footRadius = 0.0;
    auto planner = std::make_shared<SwingTrajectoryPlanner>(config, 1);
    CurvedWorkspaceKinematics kinematics;
    WorkspaceReferenceManager manager(info, nullptr, planner, nullptr, kinematics, 0.28);
    manager.setJointLimits(Eigen::Vector3d(-1, -1, 0), Eigen::Vector3d(1, 1, 1));
    ocs2::vector_t initial = ocs2::vector_t::Zero(15);
    ocs2::vector_t target = initial;
    initial[0] = target[0] = 0.1;
    target[6] = 0.1;
    ocs2::TargetTrajectories targets({0.0, 1.0}, {initial, target}, {});
    require(std::abs(manager.swingNominalState(1.3, 0.0, initial, targets)[6] - 0.13) < 1e-12,
            "Swing workspace froze the body at the target horizon before touchdown.");
    const auto bounds = manager.swingHeightBounds(0, ocs2::vector_t::Zero(15), {0.5, 0, 0});
    require(std::abs(bounds[0] + 0.75) < 2e-5 && std::abs(bounds[1] - 0.25) < 2e-5,
            "Workspace search rejected a reachable vertical line because nominal height was infeasible.");
    requireThrows([&] { manager.swingHeightBounds(0, ocs2::vector_t::Zero(15), {1.5, 0, 0}); },
                  "Workspace search accepted an unreachable horizontal path.");
}

class TerrainCommitProbe final : public ocs2::SolverSynchronizedModule {
public:
    int revision = 0;
    void preSolverRun(double, double, const ocs2::vector_t&,
                      const ocs2::ReferenceManagerInterface&) override { ++revision; }
    void postSolverRun(const ocs2::PrimalSolution&) override {}
};

class TerrainReferenceProbe final : public PerceptiveLeggedReferenceManager {
public:
    using PerceptiveLeggedReferenceManager::PerceptiveLeggedReferenceManager;
    std::shared_ptr<TerrainCommitProbe> terrain;
    int referenceRevision = 0;
protected:
    void modifyReferences(double, double, const ocs2::vector_t&, ocs2::TargetTrajectories&,
                          ocs2::ModeSchedule&) override {
        require(terrain->revision == referenceRevision + 1,
                "References were generated before the current terrain snapshot was committed.");
        referenceRevision = terrain->revision;
    }
};

void checkTerrainCommitOrder() {
    ocs2::CentroidalModelInfo info{};
    CurvedWorkspaceKinematics kinematics;
    TerrainReferenceProbe manager(info, nullptr, nullptr, nullptr, kinematics, 0.28);
    manager.terrain = std::make_shared<TerrainCommitProbe>();
    manager.setTerrainReceiver(manager.terrain);
    for (int i = 0; i < 2; ++i) {
        manager.preSolverRun(i, i + 1.0, ocs2::vector_t::Zero(15));
        require(manager.referenceRevision == i + 1,
                "Reference update did not consume the newly committed terrain.");
    }
}


class Terrain final : public TerrainInterface {
public:
    mutable int heightQueries=0;
    double getHeight(double,double) const override { ++heightQueries;return 0; }
    std::vector<TerrainPrism> getPrisms(const Eigen::Vector2d&,const Eigen::Vector2d&) const override {
        ++heightQueries;return {{{-2,-2},{2,2},0}};
    }
};
BSpline reference(double t0, double tf) {
    BSpline spline(3);
    Eigen::VectorXd z(7);
    const double endpointOffset = 0.04 * (tf - t0) / 12.0;
    z << 0.0, endpointOffset, 0.07, 0.13, 0.07, endpointOffset, 0.0;
    spline.setControlPoints(Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7), z);
    spline.setKnots(spline.generateKnots(7, t0, tf));
    return spline;
}

void checkDerivatives() {
    const auto curve = reference(1.0, 1.4);
    require(std::abs(curve.getVelocity(1.0).z() - 0.04) < 1e-10, "Wrong initial derivative.");
    require(std::abs(curve.getVelocity(1.4).z() + 0.04) < 1e-10, "Wrong final derivative.");
    for (double t : {1.01, 1.07, 1.19, 1.31, 1.39}) {
        constexpr double dt = 1e-6;
        const Eigen::Vector3d numericalVelocity = (curve.getPosition(t + dt) - curve.getPosition(t - dt)) / (2.0 * dt);
        const Eigen::Vector3d numericalAcceleration = (curve.getVelocity(t + dt) - curve.getVelocity(t - dt)) / (2.0 * dt);
        require((curve.getVelocity(t) - numericalVelocity).norm() < 1e-7, "Spline derivative differs from position derivative.");
        require((curve.getAcceleration(t) - numericalAcceleration).norm() < 1e-5, "Spline second derivative differs from velocity derivative.");
    }
}

void checkPlannerSwitch() {
    SwingTrajectoryPlanner::Config config;
    config.useTerrainPlanning = false;
    config.footRadius = 0.019;
    SwingTrajectoryPlanner planner(config, 4);
    ocs2::ModeSchedule schedule;
    schedule.modeSequence = {15}; // full stance
    feet_array_t<ocs2::scalar_array_t> heights;
    heights.fill({0.0});
    feet_array_t<std::vector<Eigen::Vector3d>> positions;
    planner.updateTerrain(schedule, heights, heights, positions, positions);
    require(std::abs(planner.getZpositionConstraint(0, 0.0)) < 1e-12,
            "terrain planning-off legacy height changed due to the terrain planning foot radius.");
    require(std::abs(planner.getPositionConstraint(0, 0.0).z() - config.footRadius) < 1e-12,
            "terrain planning 3D reference lost its foot-center offset.");

    // Disabling swing terrain planning must bypass terrain/workspace optimization on steps too.
    auto terrain = std::make_shared<Terrain>();
    planner.setTerrainInterface(terrain);
    planner.setWorkspace([](size_t, double, const Eigen::Vector3d&) -> Eigen::Vector2d {
        throw std::runtime_error("terrain planning-off swing queried the joint workspace.");
    });
    SwingTrajectoryPlanner nominal(config, 4);
    schedule.eventTimes = {1.0, 1.3};
    schedule.modeSequence = {15, 0, 15};
    for (double stepHeight : {0.1, -0.1, 0.0}) {
        feet_array_t<ocs2::scalar_array_t> landing;
        heights.fill({0.0, 0.0, stepHeight});
        landing.fill({0.0, stepHeight, stepHeight});
        nominal.update(schedule, heights, landing);
        // Empty XYZ arrays are deliberate: the legacy path only needs heights.
        planner.updateTerrain(schedule, heights, landing, positions, positions);
        for (size_t leg = 0; leg < 4; ++leg) {
            for (int i = 0; i <= 100; ++i) {
                const double time = 1.0 + 0.3 * i / 100.0;
                require(planner.getZpositionConstraint(leg, time) == nominal.getZpositionConstraint(leg, time) &&
                        planner.getZvelocityConstraint(leg, time) == nominal.getZvelocityConstraint(leg, time),
                        "terrain planning-off step trajectory differs from the nominal height planner.");
            }
        }
    }
    require(terrain->heightQueries == 0, "terrain planning-off step trajectory queried terrain optimization.");
    config.useTerrainPlanning = true;
    SwingTrajectoryPlanner enabled(config, 4);
    require(enabled.isTerrainPlanningEnabled(), "Missing terrain silently disabled requested terrain planning.");
    requireThrows([&] { enabled.updateTerrain(schedule, heights, heights, positions, positions); },
                  "Enabled terrain planning without terrain silently fell back.");
}

void checkNominalExecutingSwing() {
    SwingTrajectoryPlanner::Config config;
    SwingTrajectoryPlanner planner(config, 4);
    ocs2::ModeSchedule schedule;
    schedule.eventTimes = {1.0, 1.3, 1.6, 1.9};
    schedule.modeSequence = {15, 0, 15, 0, 15};
    feet_array_t<ocs2::scalar_array_t> heights, landing;
    heights.fill({0, 0, .1, .1, .2});
    landing.fill({0, .1, .1, .2, .2});
    planner.update(schedule, heights, landing, .95);
    const double before = planner.getZpositionConstraint(0, 1.15);
    // Future swing remains replannable before lift-off.
    landing[0][1] = .12;
    planner.update(schedule, heights, landing, .98);
    const double committed = planner.getZpositionConstraint(0, 1.15);
    require(committed != before, "Future nominal swing was frozen too early.");
    const double velocity = planner.getZvelocityConstraint(0, 1.15);
    for (double now : {1.02, 1.08, 1.20}) {
        landing[0][1] += .03;
        landing[0][3] += .02;
        planner.update(schedule, heights, landing, now);
        require(planner.getZpositionConstraint(0, 1.15) == committed &&
                planner.getZvelocityConstraint(0, 1.15) == velocity,
                "A later horizon swing or changing touchdown replaced the executing nominal swing.");
    }
    // New swing after touchdown must not inherit the old frozen trajectory.
    planner.update(schedule, heights, landing, 1.65);
    const double next = planner.getZpositionConstraint(0, 1.75);
    landing[0][3] += .1;
    planner.update(schedule, heights, landing, 1.70);
    require(planner.getZpositionConstraint(0, 1.75) == next,
            "Second executing swing did not retain its own trajectory.");
}

void checkMovingTerrainFilter() {
    grid_map::GridMap previous({"height"});
    previous.setGeometry(grid_map::Length(2, 2), .1);
    previous.setFrameId("odom");
    auto fill = [](grid_map::GridMap& map) {
        for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
            grid_map::Position p; map.getPosition(*it, p);
            map.at("height", *it) = p.x() >= 0 ? .1f : 0.f;
        }
    };
    fill(previous);
    auto current = previous;
    current.move(grid_map::Position(.2, .1));
    fill(current);
    const auto expected = current;
    PlanarTerrainReceiver::filterHeightLayer(current, previous, "height", .5);
    require((current.getPosition() - expected.getPosition()).norm() == 0 &&
            (current.getStartIndex() == expected.getStartIndex()).all(),
            "Terrain filter changed the map's world geometry.");
    require((current.get("height") - expected.get("height")).cwiseAbs().maxCoeff() < 1e-7,
            "Rolling-map filter moved or blurred a stationary stair edge.");
}

void checkExecutingSwing() {
    SwingTrajectoryPlanner::Config config;
    config.useTerrainPlanning = true;
    config.terrainQpConfig.numCurveSamples = 30;
    config.terrainQpConfig.verbose = false;
    SwingTrajectoryPlanner planner(config, 4);
    auto terrain = std::make_shared<Terrain>();
    planner.setTerrainInterface(terrain);
    ocs2::ModeSchedule schedule;
    schedule.eventTimes = {1.0, 1.3};
    schedule.modeSequence = {15, 0, 15};
    feet_array_t<ocs2::scalar_array_t> heights;
    heights.fill({0.0, 0.0, 0.0});
    feet_array_t<std::vector<Eigen::Vector3d>> positions;
    positions.fill(std::vector<Eigen::Vector3d>(3, Eigen::Vector3d::Zero()));
    planner.updateTerrain(schedule, heights, heights, positions, positions, 0.0, 0.9);
    require(terrain->heightQueries == 0, "Swing outside the MPC horizon triggered terrain optimization.");
    planner.updateTerrain(schedule, heights, heights, positions, positions, 0.95, 1.2);
    schedule.eventTimes = {1.0, 1.3, 1.6, 1.9};
    schedule.modeSequence = {15, 0, 15, 0, 15};
    heights.fill({0, 0, 0, 0, 0});
    positions.fill(std::vector<Eigen::Vector3d>(5, Eigen::Vector3d::Zero()));
    planner.updateTerrain(schedule, heights, heights, positions, positions, 1.05, 1.95);
    const double height = planner.getZpositionConstraint(0, 1.15);
    const int queries = terrain->heightQueries;
    planner.updateTerrain(schedule, heights, heights, positions, positions, 1.10, 1.3);
    require(terrain->heightQueries == queries, "Executing swing was replanned against a changing map.");
    require(planner.getZpositionConstraint(0, 1.15) == height, "Executing reference changed between MPC updates.");
    planner.updateTerrain(schedule, heights, heights, positions, positions, 1.4, 1.5);
    require(terrain->heightQueries == queries, "Historical swing triggered terrain optimization.");
}

// Reproduces the original QP infeasibility: flat terrain, MINVO geometry mode,
// workspace bounds that produce B.row(i) constraints with zero free-variable
// coefficients at the endpoints. The reference has a tiny (~1mm) endpoint
// offset that matches the real simulation scenario.
void checkWorkspaceBoundaryQpFeasibility() {
    SwingTrajectoryQpConfig qpConfig;
    qpConfig.numControlPoints = 15;
    qpConfig.numCurveSamples = 60;
    qpConfig.geometryMode = SwingGeometryMode::Minvo;
    qpConfig.subdivisionsPerSpan = 4;
    qpConfig.groundClearance = 0.03;
    qpConfig.footRadius = 0.019;
    qpConfig.verbose = false;
    qpConfig.checkReference = false;
    qpConfig.verifyOutput = false;

    auto terrain = std::make_shared<Terrain>();
    SwingTrajectoryQpOptimizer optimizer(qpConfig, terrain);

    // Reference spline: typical swing on flat ground
    BSpline refSpline(5);
    const int nRef = 7;
    Eigen::VectorXd z(nRef);
    z << 0.0, 0.002, 0.0425, 0.085, 0.0425, 0.002, 0.0;
    refSpline.setControlPoints(Eigen::VectorXd::Zero(nRef), Eigen::VectorXd::Zero(nRef), z);
    refSpline.setKnots(refSpline.generateKnots(nRef, 0.0, 1.0));

    const Eigen::Vector3d liftOff(0.1, 0.0, -0.001);  // ~1mm below ground (sim settling)
    const Eigen::Vector3d touchDown(0.2, 0.0, 0.0);

    // Workspace that constrains z at each control point phase — the endpoint
    // phases map B.row to fixed boundary control points, reproducing free_norm=0.
    auto workspace = SwingTrajectoryQpOptimizer::HeightBounds(
        [](double /*t*/, const Eigen::Vector3d& /*pos*/) -> Eigen::Vector2d {
            return {-0.005, 0.3};  // lower bound slightly above the -0.001 lift-off z
        });

    auto result = optimizer.optimizeSwingTrajectory(refSpline, 0.0, 0.3, liftOff, touchDown, workspace);
    require(result.solved,
            "MINVO QP with workspace boundary constraints should be feasible "
            "(free_norm=0 rows must be filtered).");
}

int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--nominal-only") {
            checkTerrainCommitOrder();
            checkDerivatives();
            checkPlannerSwitch();
            checkNominalExecutingSwing();
            checkMovingTerrainFilter();
            std::cout << "Nominal swing and terrain commit checks passed.\n";
            return 0;
        }
        if (argc == 2 && std::string(argv[1]) == "--continuity-only") {
            checkNominalExecutingSwing();checkMovingTerrainFilter();checkExecutingSwing();
            std::cout << "Swing continuity checks passed.\n";
            return 0;
        }
        checkWorkspaceSeed();checkTerrainCommitOrder();checkDerivatives();checkPlannerSwitch();checkExecutingSwing();
        checkWorkspaceBoundaryQpFeasibility();
        std::cout << "Swing integration checks passed.\n";
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1; }
}
