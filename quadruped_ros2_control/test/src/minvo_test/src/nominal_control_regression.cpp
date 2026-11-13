// Exercise the production problem assembly, not just the swing planner switch.
#include <ocs2_core/cost/StateInputCostCollection.h>
#include <ocs2_core/cost/StateCostCollection.h>
#include <ocs2_core/constraint/StateInputConstraintCollection.h>
#include <ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedInterface.h>
#include <ocs2_core/misc/LoadData.h>
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "Usage: nominal_control_regression TASK URDF REFERENCE\n";
        return 2;
    }
    try {
        ocs2::legged_robot::PerceptiveLeggedInterface interface(argv[1], argv[2], argv[3]);
        const std::vector<std::string> feet{"FL_foot", "FR_foot", "RL_foot", "RR_foot"};
        interface.setupJointNames({"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                                  "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                                  "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
                                  "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"}, feet);
        interface.setupOptimalControlProblem(argv[1], argv[2], argv[3], false);
        const bool terrain = interface.getSwitchedModelReferenceManagerPtr()
            ->getSwingTrajectoryPlanner()->isTerrainPlanningEnabled();
        const auto& problem = interface.getOptimalControlProblem();
        for (const auto& foot : feet) {
            size_t index;
            auto require = [&](bool ok, const std::string& message) {
                if (!ok) { throw std::runtime_error(foot + ": " + message); }
            };
            require(problem.softConstraintPtr->getTermIndex(foot + "_footPlacementCbf", index) == !terrain,
                    "legacy foothold CBF registration does not match the nominal path");
            require(problem.equalityConstraintPtr->getTermIndex(foot + "_normalVelocity", index) == !terrain,
                    "nominal Z velocity equality registration changed");
            require(problem.softConstraintPtr->getTermIndex(foot + "_swingTracking", index) == terrain,
                    "3D tracking leaked into the nominal path");
            for (const auto& suffix : {"_footPlacement", "_footCollision"}) {
                require(problem.stateSoftConstraintPtr->getTermIndex(foot + suffix, index),
                        std::string("missing shared terrain cost ") + suffix);
            }
        }
        auto manager = std::dynamic_pointer_cast<ocs2::legged_robot::PerceptiveLeggedReferenceManager>(
            interface.getSwitchedModelReferenceManagerPtr());
        auto selector = manager->getConvexRegionSelectorPtr();
        ocs2::vector_t state = ocs2::vector_t::Zero(interface.getCentroidalModelInfo().stateDim);
        ocs2::loadData::loadEigenMatrix(argv[1], "initialState", state);
        ocs2::TargetTrajectories targets({0.0, 3.0}, {state, state},
            {ocs2::vector_t::Zero(interface.getCentroidalModelInfo().inputDim),
             ocs2::vector_t::Zero(interface.getCentroidalModelInfo().inputDim)});
        ocs2::ModeSchedule schedule;
        schedule.eventTimes = {.5, 1.0, 1.3, 1.6, 1.9, 2.2};
        schedule.modeSequence = {15, 15, 0, 15, 0, 15, 15};
        selector->update(schedule, .95, state, targets);
        const auto original = selector->getProjection(0, 1.15).positionInWorld;
        auto map = selector->getPlanarTerrainPtr();
        for (double now : {1.02, 1.1, 1.2, 1.4}) {
            map->planarRegions.front().transformPlaneToWorld.translation().z() += .03;
            selector->update(schedule, now, state, targets);
            const auto retained = selector->getProjection(0, 1.31);
            if ((retained.positionInWorld - original).norm() > 1e-12 ||
                std::abs(retained.regionPtr->transformPlaneToWorld.translation().z()) > 1e-12) {
                throw std::runtime_error("Committed touchdown changed with map replacement during swing/stance.");
            }
            if (selector->getProjection(0, 2.0).positionInWorld.z() < .02) {
                throw std::runtime_error("Future touchdown was frozen before its swing.");
            }
        }
        selector->update(schedule, 1.65, state, targets);
        if (selector->getProjection(0, 2.0).positionInWorld.z() < .1) {
            throw std::runtime_error("Next swing inherited the previous committed foothold.");
        }
        std::cout << "Production control problem regression passed: terrain=" << terrain << '\n';
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
