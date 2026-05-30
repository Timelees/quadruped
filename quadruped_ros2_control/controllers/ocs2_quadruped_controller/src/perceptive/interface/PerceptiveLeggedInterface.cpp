//
// Created by biao on 3/21/25.
//
#include <memory>
#include <ocs2_core/misc/LoadData.h>
#include <boost/property_tree/ptree.hpp>
#include "ocs2_quadruped_controller/perceptive/constraint/FootCollisionConstraint.h"
#include "ocs2_quadruped_controller/perceptive/constraint/FootPlacementCBFConstraint.h"
#include "ocs2_quadruped_controller/perceptive/constraint/SphereSdfConstraint.h"

#include "ocs2_quadruped_controller/perceptive/interface/ConvexRegionSelector.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedInterface.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedPrecomputation.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h"
#include "ocs2_quadruped_controller/interface/constraint/SwingTrajectoryQPOptimizer.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>



namespace ocs2::legged_robot
{
    namespace
    {
        template <typename ValueType>
        void loadPtreeValueWithFallback(const boost::property_tree::ptree& pt,
                                        ValueType& value,
                                        const std::string& preferredKey,
                                        const std::string& fallbackKey,
                                        bool verbose)
        {
            if (const auto preferred = pt.get_optional<ValueType>(preferredKey))
            {
                value = *preferred;
                if (verbose)
                {
                    std::cerr << preferredKey << ": " << value << std::endl;
                }
                return;
            }

            if (const auto fallback = pt.get_optional<ValueType>(fallbackKey))
            {
                value = *fallback;
                if (verbose)
                {
                    std::cerr << fallbackKey << ": " << value << " (fallback)" << std::endl;
                }
            }
        }
    }

    void PerceptiveLeggedInterface::setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile,
                                                               const std::string& referenceFile, bool verbose)
    {
        // 初始化地形模型
        planarTerrainPtr_ = std::make_shared<convex_plane_decomposition::PlanarTerrain>();

        double width{5.0}, height{5.0};
        convex_plane_decomposition::PlanarRegion plannerRegion;
        plannerRegion.transformPlaneToWorld.setIdentity();
        plannerRegion.bbox2d = convex_plane_decomposition::CgalBbox2d(-height / 2, -width / 2, +height / 2, width / 2);
        convex_plane_decomposition::CgalPolygonWithHoles2d boundary;
        boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(+height / 2, +width / 2));
        boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(-height / 2, +width / 2));
        boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(-height / 2, -width / 2));
        boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(+height / 2, -width / 2));
        plannerRegion.boundaryWithInset.boundary = boundary;
        convex_plane_decomposition::CgalPolygonWithHoles2d insets;
        insets.outer_boundary().push_back(
            convex_plane_decomposition::CgalPoint2d(+height / 2 - 0.01, +width / 2 - 0.01));
        insets.outer_boundary().push_back(
            convex_plane_decomposition::CgalPoint2d(-height / 2 + 0.01, +width / 2 - 0.01));
        insets.outer_boundary().push_back(
            convex_plane_decomposition::CgalPoint2d(-height / 2 + 0.01, -width / 2 + 0.01));
        insets.outer_boundary().push_back(
            convex_plane_decomposition::CgalPoint2d(+height / 2 - 0.01, -width / 2 + 0.01));
        plannerRegion.boundaryWithInset.insets.push_back(insets);
        planarTerrainPtr_->planarRegions.push_back(plannerRegion);

        // 初始化 GridMap 和 SDF(符号距离场)
        std::string layer = "elevation_before_postprocess";
        planarTerrainPtr_->gridMap.setGeometry(grid_map::Length(5.0, 5.0), 0.03);
        planarTerrainPtr_->gridMap.add(layer, 0);               // 添加高度层
        planarTerrainPtr_->gridMap.add("smooth_planar", 0);     // 添加平滑层


        signedDistanceFieldPtr_ = std::make_shared<grid_map::SignedDistanceField>();
        // signedDistanceFieldPtr_->calculateSignedDistanceField(planarTerrainPtr_->gridMap, layer, 0.1);
        

        LeggedInterface::setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
        
        // 构建足端约束
        for (size_t i = 0; i < centroidal_model_info_.numThreeDofContacts; i++)
        {
            const std::string& footName = modelSettings().contactNames3DoF[i];

            std::unique_ptr<PenaltyBase> collisionPenalty(
                new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(1e-2, 1e-3)));
            // std::unique_ptr<PenaltyBase> edgePenalty(
            //     new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(1e-2, 1e-3)));

            // 落足点规划到平面区域内
            problem_ptr_->stateSoftConstraintPtr->add(
                footName + "_footPlacement",
                getFootPlacementConstraint(i, taskFile, "footPlacement.", verbose));
            // CBF约束
            problem_ptr_->softConstraintPtr->add(
                footName + "_footPlacementCbf",
                getFootPlacementCbfConstraint(i, taskFile, "footPlacementCbf.", verbose));

            std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr = getEeKinematicsPtr({footName}, footName);
            
            // For foot Collision
            std::unique_ptr<FootCollisionConstraint> footCollisionConstraint(
                new FootCollisionConstraint(*reference_manager_ptr_, *eeKinematicsPtr, signedDistanceFieldPtr_, i,
                                            0.03));
            problem_ptr_->stateSoftConstraintPtr->add(
                footName + "_footCollision",
                std::make_unique<StateSoftConstraint>(std::move(footCollisionConstraint), std::move(collisionPenalty)));

            
           
        }
        
        // For collision avoidance
        scalar_t thighExcess = 0.025;
        scalar_t calfExcess = 0.02;

        std::vector<std::string> collisionLinks = {"FL_knee", "FR_knee", "RL_knee", "RR_knee"};
        const std::vector<scalar_t>& maxExcesses = {calfExcess, calfExcess, calfExcess, calfExcess};

        pinocchioSphereInterfacePtr_ = std::make_shared<PinocchioSphereInterface>(
            *pinocchio_interface_ptr_, collisionLinks, maxExcesses, 0.6, urdfFile);

        CentroidalModelPinocchioMapping pinocchioMapping(centroidal_model_info_);
        auto sphereKinematicsPtr = std::make_unique<PinocchioSphereKinematics>(
            *pinocchioSphereInterfacePtr_, pinocchioMapping);

        // std::unique_ptr<SphereSdfConstraint> sphereSdfConstraint(
        //     new SphereSdfConstraint(*sphereKinematicsPtr, signedDistanceFieldPtr_));


        //  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(1e-3, 1e-3)));
        //  problem_ptr_->stateSoftConstraintPtr->add(
        //      "sdfConstraint", std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(sphereSdfConstraint), std::move(penalty))));
    }

    void PerceptiveLeggedInterface::setupReferenceManager(const std::string& taskFile, const std::string& /*urdfFile*/,
                                                          const std::string& referenceFile, bool verbose)
    {
        
        auto swingTrajectoryPlanner =
            std::make_unique<SwingTrajectoryPlanner>(
                loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), 4);
        swingTrajectoryPlanner->setTerrainInterface(std::make_shared<GridMapTerrainInterface>(
            planarTerrainPtr_,
            swingTrajectoryPlanner->getConfig().cbfQpConfig.elevationLayer,
            swingTrajectoryPlanner->getConfig().cbfQpConfig.epsilon));


        std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr = getEeKinematicsPtr(
            {model_settings_.contactNames3DoF}, "ALL_FOOT");

        auto convexRegionSelector =
            std::make_unique<ConvexRegionSelector>(centroidal_model_info_, planarTerrainPtr_, *eeKinematicsPtr,
                                                   numVertices_);

        scalar_t comHeight = 0;
        loadData::loadCppDataType(referenceFile, "comHeight", comHeight);
        const auto footPlacementRelaxationConfig = loadFootPlacementRelaxationConfig(taskFile, "footPlacement.", verbose);
        reference_manager_ptr_.reset(new PerceptiveLeggedReferenceManager(
            centroidal_model_info_, loadGaitSchedule(referenceFile, verbose),
            std::move(swingTrajectoryPlanner),
            std::move(convexRegionSelector),
            *eeKinematicsPtr, comHeight, footPlacementRelaxationConfig));

        // NOTE: SwingTrajectoryPlanner 已回退到基线版本（无 QP/避碰接口），感知层仅保留参考管理与落足区域选择。
    }

    void PerceptiveLeggedInterface::setupPreComputation(const std::string& /*taskFile*/,
                                                        const std::string& /*urdfFile*/,
                                                        const std::string& /*referenceFile*/, bool /*verbose*/)
    {
        auto& perceptiveRefMgr = dynamic_cast<PerceptiveLeggedReferenceManager&>(*reference_manager_ptr_);
        const auto& selector = *perceptiveRefMgr.getConvexRegionSelectorPtr();
        problem_ptr_->preComputationPtr = std::make_unique<PerceptiveLeggedPrecomputation>(
            *pinocchio_interface_ptr_, centroidal_model_info_, *reference_manager_ptr_->getSwingTrajectoryPlanner(),
            model_settings_, selector);
    }

    PerceptiveLeggedReferenceManager::FootPlacementRelaxationConfig
    PerceptiveLeggedInterface::loadFootPlacementRelaxationConfig(const std::string& taskFile,
                                                                 const std::string& prefix,
                                                                 bool verbose) const
    {
        boost::property_tree::ptree pt;
        read_info(taskFile, pt);

        PerceptiveLeggedReferenceManager::FootPlacementRelaxationConfig config;
        loadPtreeValueWithFallback(pt, config.maxRelaxation,
                                   prefix + "footPlacementRelaxationMax",
                                   prefix + "footPlacementRelaxtionMax", verbose);
        loadPtreeValueWithFallback(pt, config.relaxationPower,
                                   prefix + "footPlacementRelaxationPower",
                                   prefix + "footPlacementRelaxtionPower", verbose);
        loadPtreeValueWithFallback(pt, config.swingActivationProgress,
                                   prefix + "swingActivationProgress",
                                   prefix + "swingActivationProgress", verbose);
        return config;
    }

    std::unique_ptr<StateCost> PerceptiveLeggedInterface::getFootPlacementConstraint(size_t contactPointIndex,
                                                                                      const std::string& taskFile,
                                                                                      const std::string& prefix,
                                                                                      bool verbose)
    {
        const std::string& footName = modelSettings().contactNames3DoF[contactPointIndex];
        std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr = getEeKinematicsPtr({footName}, footName);

        std::unique_ptr<FootPlacementConstraint> footPlacementConstraint(
            new FootPlacementConstraint(*reference_manager_ptr_, *eeKinematicsPtr, contactPointIndex));

        RelaxedBarrierPenalty::Config barrierConfig{1e-2, 1e-4};
        boost::property_tree::ptree pt;
        read_info(taskFile, pt);
        if (const auto mu = pt.get_optional<scalar_t>(prefix + "mu"))
        {
            barrierConfig.mu = *mu;
        }
        if (const auto delta = pt.get_optional<scalar_t>(prefix + "delta"))
        {
            barrierConfig.delta = *delta;
        }
        if (verbose)
        {
            std::cerr << prefix << "mu: " << barrierConfig.mu << std::endl;
            std::cerr << prefix << "delta: " << barrierConfig.delta << std::endl;
        }
        std::unique_ptr<PenaltyBase> placementPenalty(
            new RelaxedBarrierPenalty(barrierConfig));

        return std::make_unique<StateSoftConstraint>(std::move(footPlacementConstraint), std::move(placementPenalty));
    }

    std::unique_ptr<StateInputCost> PerceptiveLeggedInterface::getFootPlacementCbfConstraint(
        size_t contactPointIndex, const std::string& taskFile,
        const std::string& prefix, bool verbose)
    {
        const std::string& footName = modelSettings().contactNames3DoF[contactPointIndex];
        std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr = getEeKinematicsPtr({footName}, footName);

        FootPlacementCBFConstraint::Config cbfConfig;
        RelaxedBarrierPenalty::Config barrierConfig{1e-2, 1e-4};
        boost::property_tree::ptree pt;
        read_info(taskFile, pt);

        if (const auto gamma = pt.get_optional<scalar_t>(prefix + "gamma"))
        {
            cbfConfig.gamma = *gamma;
        }
        if (const auto activationProgress = pt.get_optional<scalar_t>(prefix + "swingActivationProgress"))
        {
            cbfConfig.swingActivationProgress = *activationProgress;
        }
        if (const auto relaxationDerivativeDt = pt.get_optional<scalar_t>(prefix + "relaxationDerivativeDt"))
        {
            cbfConfig.relaxationDerivativeDt = *relaxationDerivativeDt;
        }
        if (const auto mu = pt.get_optional<scalar_t>(prefix + "mu"))
        {
            barrierConfig.mu = *mu;
        }
        if (const auto delta = pt.get_optional<scalar_t>(prefix + "delta"))
        {
            barrierConfig.delta = *delta;
        }
        if (verbose)
        {
            std::cerr << prefix << "gamma: " << cbfConfig.gamma << std::endl;
            std::cerr << prefix << "swingActivationProgress: " << cbfConfig.swingActivationProgress << std::endl;
            std::cerr << prefix << "relaxationDerivativeDt: " << cbfConfig.relaxationDerivativeDt << std::endl;
            std::cerr << prefix << "mu: " << barrierConfig.mu << std::endl;
            std::cerr << prefix << "delta: " << barrierConfig.delta << std::endl;
        }

        std::unique_ptr<FootPlacementCBFConstraint> footPlacementCbfConstraint(
            new FootPlacementCBFConstraint(*reference_manager_ptr_, *eeKinematicsPtr, contactPointIndex, cbfConfig));
        std::unique_ptr<PenaltyBase> placementCbfPenalty(
            new RelaxedBarrierPenalty(barrierConfig));
        return std::make_unique<StateInputSoftConstraint>(std::move(footPlacementCbfConstraint),
                                                          std::move(placementCbfPenalty));
    }


} // namespace legged
