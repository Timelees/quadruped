//
// Created by biao on 3/21/25.
//

#include <utility>
#include <algorithm>
#include <iomanip>
#include <array>
#include <ocs2_core/misc/Lookup.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h"

#include "log_save.h"

#include <rclcpp/rclcpp.hpp>

namespace ocs2::legged_robot
{
    
    
    
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
        for (size_t i = 0; i < nodeNum; ++i)
        {
            scalar_t time = initTime + static_cast<double>(i) * timeHorizon / (nodeNum - 1);
            vector_t state = targetTrajectories.getDesiredState(time);
            vector_t input = targetTrajectories.getDesiredState(time);

            const auto& map = convexRegionSelectorPtr_->getPlanarTerrainPtr()->gridMap;
            vector_t pos = centroidal_model::getBasePose(state, info_).head(3); // (x,y,z)

            // grid_map 的 atPosition() 越界会 throw std::out_of_range。
            // ground_truth 模式下 base 的世界坐标更容易落在 map 外，必须做保护避免整个进程崩溃。
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
            centroidal_model::getBasePose(state, info_)(4) = atan(v.x() / v.z());   // 修改俯仰角

            // Base Z Position
            centroidal_model::getBasePose(state, info_)(2) =
                map.atPosition("smooth_planar", pos2d) + comHeight_ / cos(centroidal_model::getBasePose(state, info_)(4));

            newTargetTrajectories.timeTrajectory.push_back(time);
            newTargetTrajectories.stateTrajectory.push_back(state);
            newTargetTrajectories.inputTrajectory.push_back(input);
        }
        targetTrajectories = newTargetTrajectories;

        // Footstep — 使用 ConvexRegionSelector（凸多边形生长 + CBF 边缘惩罚）
        convexRegionSelectorPtr_->update(modeSchedule, initTime, initState, targetTrajectories);

        // Swing trajectory
        updateSwingTrajectoryPlanner(initTime, initState, modeSchedule);
    }

    void PerceptiveLeggedReferenceManager::setEdgeSpheres(std::shared_ptr<EdgeSphereArray> edgeSpheresPtr) {
        edgeSpheresPtr_ = edgeSpheresPtr;

        if (!edgeSpheresPtr_)
        {
            footholdScorer_.reset();
            convexRegionSelectorPtr_->setFootholdScorer(nullptr);
            return;
        }

        EdgeAwareFootholdScorer::Config scorerConfig;
        scorerConfig.safeMargin = 0.08;
        scorerConfig.penaltyWeight = 3.0;
        scorerConfig.sphereRadiusInflation = 0.01;
        scorerConfig.smoothingDistance = 0.02;
        scorerConfig.maxPenalty = 0.08;
        scorerConfig.verticalWeightScale = 0.06;
        scorerConfig.maxConsideredSpheres = 4;
        scorerConfig.useXYDistanceOnly = true;

        if (footholdScorer_ == nullptr)
        {
            footholdScorer_ = std::make_unique<EdgeAwareFootholdScorer>(scorerConfig, edgeSpheresPtr_);
        }
        else
        {
            footholdScorer_->updateEdgeSpheres(edgeSpheresPtr_);
        }

        convexRegionSelectorPtr_->setFootholdScorer(footholdScorer_.get());
    }

    void PerceptiveLeggedReferenceManager::updateSwingTrajectoryPlanner(scalar_t initTime, const vector_t& initState,
                                                                        ModeSchedule& modeSchedule)
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

        // // 如果 QP 优化器已启用，使用带避碰优化的更新
        // if (swingTrajectoryPtr_->isQpOptimizerEnabled()) {
        //     auto [liftOff3d, touchDown3d] = getSwingPositions3d(contactFlagStocks, allProjections);
        //     // std::cout << "modeSchedule: " << std::endl;
        //     // for(int i = 0; i < modeSchedule.modeSequence.size(); ++i) {
        //     //     std::cout << "  Time: " << modeSchedule.eventTimes[i] << ", Mode: " << modeSchedule.modeSequence[i] << std::endl;
        //     // }
        //     swingTrajectoryPtr_->updateWithQpOptimization(modeSchedule, initTime,
        //                                                liftOffHeightSequence, touchDownHeightSequence,
        //                                                liftOff3d, touchDown3d);
        // } else {
            swingTrajectoryPtr_->update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence);
        // }
    }
    
    // 根据当前状态和接触标志修正投影的高度
    void PerceptiveLeggedReferenceManager::modifyProjections(scalar_t initTime, const vector_t& initState, size_t leg,
                                                             size_t initIndex,
                                                             const std::vector<bool>& contactFlagStocks,
                                                             std::vector<
                                                                 convex_plane_decomposition::PlanarTerrainProjection>&
                                                             projections)
    {
        if (contactFlagStocks[initIndex])
        {
            lastLiftoffPos_[leg] = endEffectorKinematicsPtr_->getPosition(initState)[leg];
            lastLiftoffPos_[leg].z() -= 0.019;
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
        const auto finalTime = convexRegionSelectorPtr_->getInitStandFinalTimes();
        for (int i = 0; i < flag.size(); ++i)
        {
            const bool hasPolygon = convexRegionSelectorPtr_->getConvexPolygon(i, time).size() >= 3;    // 是否有有效的支撑多边形
            const bool isContact = getContactFlags(time)[i];                                            // 当前是否在接触相
            const scalar_t swingProgress = getSwingTrajectoryPlanner()->getSwingPhaseProgress(i, time);
            const bool isLateSwing = swingProgress >= footPlacementRelaxationConfig_.swingActivationProgress;
            flag[i] = hasPolygon && time >= finalTime[i] && (isContact || isLateSwing);
        }
        return flag;
    }

    // 根据摆动相进度计算足端位置约束的松弛量，摆动初期松弛较大以增加可行域，接近着陆时逐渐收紧约束
    // hiw​(x,t)=Ai​⋅pWi​​(x)+bi​+s(t)⋅1≥0中的s()
    scalar_t PerceptiveLeggedReferenceManager::getFootPlacementRelaxation(size_t leg, scalar_t time) const
    {
        const scalar_t swingProgress = getSwingTrajectoryPlanner()->getSwingPhaseProgress(leg, time);
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