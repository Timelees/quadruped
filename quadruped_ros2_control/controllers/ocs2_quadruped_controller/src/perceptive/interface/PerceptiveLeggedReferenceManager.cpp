//
// Created by biao on 3/21/25.
//

#include <utility>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <array>
#include <Eigen/QR>
#include <ocs2_core/misc/Lookup.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h"

#include "ocs2_quadruped_controller/perceptive/synchronize/PlanarTerrainReceiver.h"
#include "log_save.h"

#include <rclcpp/rclcpp.hpp>

namespace ocs2::legged_robot
{
    namespace
    {
        scalar_t computeSwingPhaseProgressFromModeSchedule(const ModeSchedule& modeSchedule,
                                                           const feet_array_t<std::vector<bool>>& contactFlagStocks,
                                                           size_t leg, scalar_t time)
        {
            if (modeSchedule.eventTimes.empty() || modeSchedule.modeSequence.empty())
            {
                return scalar_t(-1.0);
            }

            if (leg >= contactFlagStocks.size())
            {
                return scalar_t(-1.0);
            }
            const auto& contactFlags = contactFlagStocks[leg];
            if (contactFlags.empty())
            {
                return scalar_t(-1.0);
            }

            const size_t phaseIndex = lookup::findIndexInTimeArray(modeSchedule.eventTimes, time);
            const size_t clampedIndex = std::min(phaseIndex, contactFlags.size() - 1);
            if (contactFlags[clampedIndex])
            {
                return scalar_t(-1.0);
            }

            // find swing start index (previous stance)
            int startIdx = -1;
            for (int i = static_cast<int>(clampedIndex) - 1; i >= 0; --i)
            {
                if (contactFlags[static_cast<size_t>(i)])
                {
                    startIdx = i;
                    break;
                }
            }
            // find swing final index (next stance)
            int finalIdx = static_cast<int>(contactFlags.size()) - 1;
            for (size_t i = clampedIndex + 1; i < contactFlags.size(); ++i)
            {
                if (contactFlags[i])
                {
                    finalIdx = static_cast<int>(i) - 1;
                    break;
                }
            }
            if (startIdx < 0)
            {
                return scalar_t(-1.0);
            }
            const int lastEvent = static_cast<int>(modeSchedule.eventTimes.size()) - 1;
            const int safeStart = std::min(std::max(startIdx, 0), lastEvent);
            const int safeFinal = std::min(std::max(finalIdx, 0), lastEvent);

            const scalar_t t0 = modeSchedule.eventTimes[static_cast<size_t>(safeStart)];
            const scalar_t t1_raw = modeSchedule.eventTimes[static_cast<size_t>(safeFinal)];
            const scalar_t t1 = (t1_raw > t0) ? t1_raw : (t0 + scalar_t(1e-3));

            const scalar_t alpha = (time - t0) / (t1 - t0);
            return std::min<scalar_t>(1.0, std::max<scalar_t>(0.0, alpha));
        }
    }

    
    
    
    namespace { constexpr double ikTolerance = 1e-3; constexpr int maxIterations = 32; }
    PerceptiveLeggedReferenceManager::PerceptiveLeggedReferenceManager(CentroidalModelInfo info,
                                                                       std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                                       std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                                                       std::shared_ptr<ConvexRegionSelector> convexRegionSelectorPtr,
                                                                       const EndEffectorKinematics<scalar_t>&
                                                                       endEffectorKinematics,
                                                                       scalar_t comHeight,
                                                                       FootPlacementRelaxationConfig footPlacementRelaxationConfig)
        : info_(std::move(info)),
          SwitchedModelReferenceManager(std::move(gaitSchedulePtr), std::move(swingTrajectoryPtr)),
          convexRegionSelectorPtr_(std::move(convexRegionSelectorPtr)),
          endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
          comHeight_(comHeight),
          footPlacementRelaxationConfig_(std::move(footPlacementRelaxationConfig))
    {
    }

    void PerceptiveLeggedReferenceManager::preSolverRun(scalar_t initTime, scalar_t finalTime,
                                                       const vector_t& initState)
    {
        // ReferenceManager runs before solver synchronized modules. Commit terrain
        // here so footholds, swing references and collision costs use one snapshot.
        if (terrainReceiver_) {
            terrainReceiver_->preSolverRun(initTime, finalTime, initState, *this);
            swingTrajectoryPtr_->setMapTiming(terrainReceiver_->mapTiming(initTime));
        }
        SwitchedModelReferenceManager::preSolverRun(initTime, finalTime, initState);
    }

    void PerceptiveLeggedReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime,
                                                            const vector_t& initState,
                                                            TargetTrajectories& targetTrajectories,
                                                            ModeSchedule& modeSchedule)
    {
        // 更新模式调度
        const auto timeHorizon = finalTime - initTime;
        modeSchedule = getGaitSchedule()->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);



        // 生成新目标轨迹
        TargetTrajectories newTargetTrajectories;
        int nodeNum = 11;
        // TargetManager blends measured pitch toward the operator command. Testing
        // each intermediate pitch against zero disabled terrain alignment whenever
        // the measured body had any tilt. Use the terminal operator target instead.
        const bool followTerrainPitch = std::abs(
            centroidal_model::getBasePose(targetTrajectories.stateTrajectory.back(), info_)(4)) < 1e-6;
        const scalar_t initialPitch = centroidal_model::getBasePose(initState, info_)(4);
        for (size_t i = 0; i < nodeNum; ++i)
        {
            scalar_t time = initTime + static_cast<double>(i) * timeHorizon / (nodeNum - 1);
            vector_t state = targetTrajectories.getDesiredState(time);
            vector_t input = targetTrajectories.getDesiredInput(time);

            const auto& map = convexRegionSelectorPtr_->getPlanarTerrainPtr()->gridMap;
            vector_t pos = centroidal_model::getBasePose(state, info_).head(3); // (x,y,z)

            // grid_map 的 atPosition() 越界会 throw std::out_of_range。
            const grid_map::Position pos2d(pos.x(), pos.y());
            if (!map.isInside(pos2d))
            {

                newTargetTrajectories.timeTrajectory.push_back(time);
                newTargetTrajectories.stateTrajectory.push_back(state);
                newTargetTrajectories.inputTrajectory.push_back(input);
                continue;
            }

            // Base Orientation 根据地形调整基座姿态和高度
            scalar_t step = 0.3;
            grid_map::Vector3 normalVector;

            // 做差分取梯度时也可能越界（靠近边缘），这里用中心点退化处理。
            auto safeAtPosition = [&](const grid_map::Position& p) -> scalar_t {
                if (!map.isInside(p))
                {
                    return map.atPosition("smooth_planar", pos2d);
                }
                return map.atPosition("smooth_planar", p);
            };
            
            // 计算高度梯度，normalVector为当前基座位置所处地形的向上法向量，用于后续调整基座的俯仰角
            normalVector(0) = (safeAtPosition(pos2d + grid_map::Position(-step, 0)) -
                               safeAtPosition(pos2d + grid_map::Position(step, 0))) /
                              (2 * step);
            normalVector(1) = (safeAtPosition(pos2d + grid_map::Position(0, -step)) -
                               safeAtPosition(pos2d + grid_map::Position(0, step))) /
                              (2 * step);
            normalVector(2) = 1;
            normalVector.normalize();
            matrix3_t R;
            scalar_t yaw = centroidal_model::getBasePose(state, info_)(3);  // 偏航角
            R << cos(yaw), -sin(yaw), 0, // clang-format off
             sin(yaw), cos(yaw), 0,
             0, 0, 1;  // clang-format on
            vector_t v = R.transpose() * normalVector;
            auto basePose = centroidal_model::getBasePose(state, info_);
            // Stair gradients can request >0.3 rad while the two support
            // feet lie on different treads. Cap tilt to retain leg workspace.
            const scalar_t terrainPitch = std::clamp(std::atan2(v.x(), v.z()), scalar_t(-0.20), scalar_t(0.20));
            if (followTerrainPitch)
            {
                const scalar_t blend = std::min(scalar_t(1.0), (time - initTime) / scalar_t(0.3));
                basePose(4) = initialPitch + blend * (terrainPitch - initialPitch);
            }

            // Base Z Position
            basePose(2) = map.atPosition("smooth_planar", pos2d) + comHeight_ / cos(basePose(4));

            newTargetTrajectories.timeTrajectory.push_back(time);
            newTargetTrajectories.stateTrajectory.push_back(state);
            newTargetTrajectories.inputTrajectory.push_back(input);
        }
        targetTrajectories = newTargetTrajectories;

        // Footstep — 使用 ConvexRegionSelector（凸多边形生长）
        convexRegionSelectorPtr_->update(modeSchedule, initTime, initState, targetTrajectories);

        // Swing trajectory
        updateSwingTrajectoryPlanner(initTime, initState, modeSchedule, targetTrajectories);
    }

    void PerceptiveLeggedReferenceManager::updateSwingTrajectoryPlanner(scalar_t initTime, const vector_t& initState,
                                                                        ModeSchedule& modeSchedule,
                                                                        const TargetTrajectories& targets)
    {
        const auto contactFlagStocks = convexRegionSelectorPtr_->extractContactFlags(modeSchedule.modeSequence);
        feet_array_t<scalar_array_t> liftOffHeightSequence, touchDownHeightSequence;
        feet_array_t<std::vector<convex_plane_decomposition::PlanarTerrainProjection>> allProjections;

        for (size_t leg = 0; leg < info_.numThreeDofContacts; leg++)
        {
            size_t initIndex = lookup::findIndexInTimeArray(modeSchedule.eventTimes, initTime);

            auto projections = convexRegionSelectorPtr_->getProjections(leg);
            modifyProjections(initTime, initState, leg, initIndex, contactFlagStocks[leg], projections);

            scalar_array_t liftOffHeights, touchDownHeights;
            std::tie(liftOffHeights, touchDownHeights) = getHeights(contactFlagStocks[leg], projections);
            liftOffHeightSequence[leg] = liftOffHeights;
            touchDownHeightSequence[leg] = touchDownHeights;
            allProjections[leg] = projections;
            // std::cout << "\033[34m[ReferenceManager] Leg " << leg << " liftOffHeightSequence: ";
            // for (const auto& height : liftOffHeightSequence[leg]) {
            //     std::cout << height << " ";
            // }
            // std::cout << " touchDownHeightSequence: ";
            // for (const auto& height : touchDownHeightSequence[leg]) {
            //     std::cout << height << " ";
            // }
            // std::cout << " updated based on projections\033[0m\n";
        }

        if (swingTrajectoryPtr_->isTerrainPlanningEnabled())
        {
            // Workspace logging for diagnostics
            static int workspaceLogCounter = 0;
            const bool shouldLog = (workspaceLogCounter++ % 10 == 0);  // Log every 10th call

            swingTrajectoryPtr_->setWorkspace([this, targets, initState, initTime, shouldLog](size_t leg, double time, const Eigen::Vector3d& position) {
                try {
                    auto bounds = swingHeightBounds(leg, swingNominalState(time, initTime, initState, targets), position);
                    if (shouldLog && leg == 0) {
                        std::cerr << "[Workspace] leg=" << leg << " t=" << time
                                  << " pos=(" << position.x() << "," << position.y() << "," << position.z() << ")"
                                  << " bounds=[" << bounds[0] << "," << bounds[1] << "]"
                                  << " range=" << (bounds[1] - bounds[0]) << '\n';
                    }
                    return bounds;
                } catch (const std::runtime_error& e) {
                    if (shouldLog && leg == 0) {
                        std::cerr << "[Workspace] IK failed: " << e.what() << '\n';
                    }
                    // IK failed for this query — return a wide default bound so the
                    // QP can still solve without workspace constraints. The terrain
                    // clearance constraints still provide safety.
                    return Eigen::Vector2d(-1.0, 1.0);
                }
            });
            const auto [liftOff3d, touchDown3d] = getSwingPositions3d(contactFlagStocks, allProjections);
            SwingTrajectoryPlanner::SwingFootState footPosition, footVelocity, footAcceleration;
            const auto measuredPositions = endEffectorKinematicsPtr_->getPosition(initState);
            const auto feedback = swingTrajectoryPtr_->footFeedback();
            for (size_t leg = 0; leg < info_.numThreeDofContacts; ++leg) {
                footPosition[leg] = measuredPositions[leg];
                footPosition[leg].z() -= swingTrajectoryPtr_->getConfig().footRadius;
                footVelocity[leg] = Eigen::Vector3d::Constant(std::numeric_limits<scalar_t>::quiet_NaN());
                footAcceleration[leg] = footVelocity[leg];
                if (feedback && initTime >= feedback->time && initTime - feedback->time <= 0.03) {
                    footVelocity[leg] = feedback->velocity[leg];
                }
            }
            swingTrajectoryPtr_->updateTerrain(modeSchedule, liftOffHeightSequence, touchDownHeightSequence,
                                           liftOff3d, touchDown3d, initTime, targets.timeTrajectory.back(),
                                           footPosition, footVelocity, footAcceleration);
            swingTrajectoryPtr_->setWorkspace({});
        }
        else
        {
            swingTrajectoryPtr_->update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence, initTime);
        }
    }
    
    vector_t PerceptiveLeggedReferenceManager::swingNominalState(
        scalar_t time, scalar_t initTime, const vector_t& initState, const TargetTrajectories& targets) const
    {
        vector_t state = targets.getDesiredState(time);
        // Swing segments can end beyond the target horizon. getDesiredState()
        // clamps there; continue the commanded translation instead of checking
        // the landing foot against a body frozen before touchdown.
        state.segment<3>(6) += std::max(0.0, time - targets.timeTrajectory.back()) * state.head<3>();
        // Start the workspace at the measured body pose and approach the target
        // over the prediction horizon instead of jumping to its commanded height.
        const double progress = std::clamp((time - initTime) / (targets.timeTrajectory.back() - initTime), 0.0, 1.0);
        state.segment<6>(6) += (1.0 - progress) *
            (initState.segment<6>(6) - targets.getDesiredState(initTime).segment<6>(6));
        return state;
    }

    bool PerceptiveLeggedReferenceManager::solveSwingIk(size_t leg, const vector3_t& desired, vector_t& state) const
    {
        const int firstJoint = state.size() - info_.actuatedDofNum + 3 * leg;
        const vector3_t lower = jointLower_.segment<3>(3 * leg);
        const vector3_t upper = jointUpper_.segment<3>(3 * leg);
        for (int iteration = 0; iteration < maxIterations; ++iteration) {
            const auto approximation = endEffectorKinematicsPtr_->getPositionLinearApproximation(state)[leg];
            const vector3_t error = desired - approximation.f;
            if (error.norm() < ikTolerance) { return true; }
            const matrix3_t jacobian = approximation.dfdx.block<3, 3>(0, firstJoint);
            const vector3_t delta = jacobian.colPivHouseholderQr().solve(error);
            const vector3_t joints = state.segment<3>(firstJoint);
            double step = 1.0;
            while (step * delta.norm() > ikTolerance) {
                state.segment<3>(firstJoint) = (joints + step * delta).cwiseMax(lower).cwiseMin(upper);
                if ((desired - endEffectorKinematicsPtr_->getPosition(state)[leg]).squaredNorm() < error.squaredNorm()) { break; }
                step *= 0.5;
            }
            if (step * delta.norm() <= ikTolerance) {
                state.segment<3>(firstJoint) = joints;
                break;
            }
        }
        return (desired - endEffectorKinematicsPtr_->getPosition(state)[leg]).norm() < ikTolerance;
    }

    // Vertical section of the IK branch reached from the nominal leg posture.
    // Bounds are in contact-surface coordinates, matching the height QP (foot center minus radius).
    Eigen::Vector2d PerceptiveLeggedReferenceManager::swingHeightBounds(
        size_t leg, const vector_t& nominalState, const Eigen::Vector3d& position) const
    {
        const Eigen::Vector2d xy = position.head<2>();
        const int firstJoint = nominalState.size() - info_.actuatedDofNum + 3 * leg;
        const vector3_t lower = jointLower_.segment<3>(3 * leg);
        const vector3_t upper = jointUpper_.segment<3>(3 * leg);
        const double radius = swingTrajectoryPtr_->getConfig().footRadius;
        const auto nominalPosition = endEffectorKinematicsPtr_->getPosition(nominalState)[leg];
        // Anchor the section to a reachable reference height before searching its
        // boundaries. If that height is unreachable, solve XY with height free.
        vector_t seedState = nominalState;
        seedState.segment<3>(firstJoint) = seedState.segment<3>(firstJoint).cwiseMax(lower).cwiseMin(upper);
        vector3_t desired = position;
        desired.z() += radius;
        const bool referenceReachable = solveSwingIk(leg, desired, seedState);
        if (!referenceReachable) { seedState = nominalState; }
        for (int iteration = 0; !referenceReachable && iteration < maxIterations; ++iteration) {
            const auto approximation = endEffectorKinematicsPtr_->getPositionLinearApproximation(seedState)[leg];
            const Eigen::Vector2d error = xy - approximation.f.head<2>();
            if (error.norm() < ikTolerance) { break; }
            const Eigen::Matrix<double, 2, 3> jacobian = approximation.dfdx.block<2, 3>(0, firstJoint);
            const vector3_t delta = jacobian.completeOrthogonalDecomposition().solve(error);
            seedState.segment<3>(firstJoint) = (seedState.segment<3>(firstJoint) + delta).cwiseMax(lower).cwiseMin(upper);
        }
        const vector3_t seedPosition = endEffectorKinematicsPtr_->getPosition(seedState)[leg];
        if ((seedPosition.head<2>() - xy).norm() >= ikTolerance) {
            throw std::runtime_error("Swing horizontal path IK failed: leg=" + std::to_string(leg) +
                                     " xy=" + std::to_string(xy.x()) + "," + std::to_string(xy.y()) +
                                     " residual=" + std::to_string((seedPosition.head<2>() - xy).norm()));
        }
        auto reachable = [&](double height, vector_t& state) {
            const vector3_t desired(xy.x(), xy.y(), height + radius);
            return solveSwingIk(leg, desired, state);
        };
        const double seed = seedPosition.z() - radius;
        // A leg cannot reach farther vertically than its full displacement from the body.
        const double span = 2.0 * (nominalPosition -
            centroidal_model::getBasePose(nominalState, info_).head<3>()).norm();
        Eigen::Vector2d bounds;
        for (int side = 0; side < 2; ++side) {
            double inside = seed;
            double outside = seed + (side == 0 ? -span : span);
            vector_t insideState = seedState;
            // Resolve the boundary to the IK residual tolerance. Eight bisections
            // left millimetres of artificial workspace loss at ground contact.
            while (std::abs(outside - inside) > ikTolerance) {
                const double midpoint = 0.5 * (inside + outside);
                vector_t candidate = insideState;
                if (reachable(midpoint, candidate)) { inside = midpoint; insideState = std::move(candidate); }
                else { outside = midpoint; }
            }
            bounds[side] = inside;
        }
        return bounds;
    }

    // 根据当前状态和接触标志修正投影的高度
    void PerceptiveLeggedReferenceManager::modifyProjections(scalar_t initTime, const vector_t& initState, size_t leg,
                                                             size_t initIndex,
                                                             const std::vector<bool>& contactFlagStocks,
                                                             std::vector< convex_plane_decomposition::PlanarTerrainProjection>& projections)
    {
        if (contactFlagStocks[initIndex])
        {
            lastLiftoffPos_[leg] = endEffectorKinematicsPtr_->getPosition(initState)[leg];
            lastLiftoffPos_[leg].z() -= swingTrajectoryPtr_->getConfig().footRadius;
            for (int i = initIndex; i < projections.size(); ++i)
            {
                if (!contactFlagStocks[i])
                {
                    break;
                }
                projections[i].positionInWorld = lastLiftoffPos_[leg];
            }
            for (int i = initIndex; i >= 0; --i)
            {
                if (!contactFlagStocks[i])
                {
                    break;
                }
                projections[i].positionInWorld = lastLiftoffPos_[leg];
            }
        }
        if (initTime > convexRegionSelectorPtr_->getInitStandFinalTimes()[leg])
        {
            for (int i = initIndex; i >= 0; --i)
            {
                if (contactFlagStocks[i])
                {
                    projections[i].positionInWorld = lastLiftoffPos_[leg];
                }
                if (!contactFlagStocks[i] && !contactFlagStocks[i + 1])
                {
                    break;
                }
            }
        }
        //    for (int i = 0; i < numPhases; ++i) {
        //      if (leg == 1) std::cerr << std::setprecision(3) << projections[i].positionInWorld.z() << "\t";
        //    }
        //    std::cerr << std::endl;
    }

    std::pair<scalar_array_t, scalar_array_t> PerceptiveLeggedReferenceManager::getHeights(
        const std::vector<bool>& contactFlagStocks,
        const std::vector<convex_plane_decomposition::PlanarTerrainProjection>& projections)
    {
        scalar_array_t liftOffHeights, touchDownHeights;
        const size_t numPhases = projections.size();

        liftOffHeights.clear();
        liftOffHeights.resize(numPhases);
        touchDownHeights.clear();
        touchDownHeights.resize(numPhases);

        for (size_t i = 1; i < numPhases; ++i)
        {
            if (!contactFlagStocks[i])
            {
                liftOffHeights[i] = contactFlagStocks[i - 1]
                                        ? projections[i - 1].positionInWorld.z()
                                        : liftOffHeights[i - 1];
            }
        }
        for (int i = numPhases - 2; i >= 0; --i)
        {
            if (!contactFlagStocks[i])
            {
                touchDownHeights[i] = contactFlagStocks[i + 1]
                                          ? projections[i + 1].positionInWorld.z()
                                          : touchDownHeights[i + 1];
            }
        }

        return {liftOffHeights, touchDownHeights};
    }

    contact_flag_t PerceptiveLeggedReferenceManager::getFootPlacementFlags(scalar_t time) const
    {
        contact_flag_t flag;
        const auto contactFlagStocks = convexRegionSelectorPtr_->extractContactFlags(getModeSchedule().modeSequence);
        const auto finalTime = convexRegionSelectorPtr_->getInitStandFinalTimes();
        for (int i = 0; i < flag.size(); ++i)
        {
            const bool hasPolygon = convexRegionSelectorPtr_->getConvexPolygon(i, time).size() >= 3;    // 是否有有效的支撑多边形
            const bool isContact = getContactFlags(time)[i];                                            // 当前是否在接触相
            const scalar_t swingProgress = computeSwingPhaseProgressFromModeSchedule(
                getModeSchedule(), contactFlagStocks, static_cast<size_t>(i), time);
            const bool isLateSwing = swingProgress >= footPlacementRelaxationConfig_.swingActivationProgress;
            flag[i] = hasPolygon && time >= finalTime[i] && (isContact || isLateSwing);
        }
        return flag;
    }

    // 根据摆动相进度计算足端位置约束的松弛量，摆动初期松弛较大以增加可行域，接近着陆时逐渐收紧约束
    // hiw​(x,t)=Ai​⋅pWi​​(x)+bi​+s(t)⋅1≥0中的s()
    scalar_t PerceptiveLeggedReferenceManager::getFootPlacementRelaxation(size_t leg, scalar_t time) const
    {
        const auto contactFlagStocks = convexRegionSelectorPtr_->extractContactFlags(getModeSchedule().modeSequence);
        const scalar_t swingProgress = computeSwingPhaseProgressFromModeSchedule(
            getModeSchedule(), contactFlagStocks, leg, time);
        if (swingProgress < 0.0)
        {
            return 0.0;
        }

        const scalar_t remaining = std::max<scalar_t>(0.0, 1.0 - swingProgress);
         return footPlacementRelaxationConfig_.maxRelaxation *
             std::pow(remaining, footPlacementRelaxationConfig_.relaxationPower);
    }

    
    
    
    std::pair<feet_array_t<std::vector<Eigen::Vector3d>>, feet_array_t<std::vector<Eigen::Vector3d>>>
        PerceptiveLeggedReferenceManager::getSwingPositions3d(
            const feet_array_t<std::vector<bool>>& contactFlagStocks,
            const feet_array_t<std::vector<convex_plane_decomposition::PlanarTerrainProjection>>& allProjections) const
    {
        feet_array_t<std::vector<Eigen::Vector3d>> liftOff3d, touchDown3d;
        for (size_t leg = 0; leg < info_.numThreeDofContacts; leg++) {
            const auto& contacts = contactFlagStocks[leg];
            const auto& proj = allProjections[leg];
            const size_t numPhases = contacts.size();
            liftOff3d[leg].resize(numPhases, Eigen::Vector3d::Zero());
            touchDown3d[leg].resize(numPhases, Eigen::Vector3d::Zero());

            for (size_t i = 0; i < numPhases; ++i) {
                if (contacts[i]) continue;  // 仅处理摆动相

                bool foundLift = false, foundTouch = false;

                // 抬脚位置：向前搜索最近的接触相投影
                for (int k = static_cast<int>(i) - 1; k >= 0; --k) {
                    if (contacts[k]) {
                        liftOff3d[leg][i] = proj[k].positionInWorld;
                        foundLift = true;
                        break;
                    }
                }
                // 落脚位置：向后搜索最近的接触相投影
                for (size_t k = i + 1; k < numPhases; ++k) {
                    if (contacts[k]) {
                        touchDown3d[leg][i] = proj[k].positionInWorld;
                        foundTouch = true;
                        break;
                    }
                }

                // 边界情况：找不到抬脚/落脚位置时使用对端位置或上次已知位置
                if (!foundLift && foundTouch) {
                    // 没有之前的接触相：用上次已知的抬脚位置
                    liftOff3d[leg][i] = lastLiftoffPos_[leg];
                }
                if (!foundTouch && foundLift) {
                    // 没有之后的接触相：使用抬脚位置（原地踏步退化）
                    touchDown3d[leg][i] = liftOff3d[leg][i];
                }
                if (!foundLift && !foundTouch) {
                    // 两端均无接触相：使用上次已知位置
                    liftOff3d[leg][i] = lastLiftoffPos_[leg];
                    touchDown3d[leg][i] = lastLiftoffPos_[leg];
                }
            }
        }
        return {liftOff3d, touchDown3d};
    }

} // namespace legged
