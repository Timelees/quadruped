//
// Created by biao on 3/21/25.
//

#include "ocs2_quadruped_controller/perceptive/synchronize/PlanarTerrainReceiver.h"

#include <convex_plane_decomposition_ros/MessageConversion.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

namespace ocs2::legged_robot
{
    PlanarTerrainReceiver::PlanarTerrainReceiver(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                                 const std::shared_ptr<convex_plane_decomposition::PlanarTerrain>
                                                 & planarTerrainPtr,
                                                 const std::shared_ptr<grid_map::SignedDistanceField>&
                                                 signedDistanceFieldPtr,
                                                 const std::string& mapTopic, const std::string& sdfElevationLayer)
        : node_(node),
          sdfElevationLayer_(sdfElevationLayer),
          planarTerrainPtr_(planarTerrainPtr),
          sdfPtr_(signedDistanceFieldPtr)
    {
        maxMapAge_ = getOrDeclareDoubleParameter("planar_terrain_max_age", maxMapAge_);
        futureTimestampTolerance_ = getOrDeclareDoubleParameter(
            "planar_terrain_future_tolerance", futureTimestampTolerance_);
        maxCenterJump_ = getOrDeclareDoubleParameter("planar_terrain_center_jump_threshold", maxCenterJump_);
        centerFilterThreshold_ = getOrDeclareDoubleParameter(
            "planar_terrain_center_filter_threshold", centerFilterThreshold_);
        heightFilterAlpha_ = getOrDeclareDoubleParameter("planar_terrain_height_filter_alpha", heightFilterAlpha_);

        maxMapAge_ = std::max(0.0, maxMapAge_);
        futureTimestampTolerance_ = std::max(0.0, futureTimestampTolerance_);
        maxCenterJump_ = std::max(0.0, maxCenterJump_);
        centerFilterThreshold_ = std::max(0.0, centerFilterThreshold_);
        heightFilterAlpha_ = std::max(0.0, std::min(1.0, heightFilterAlpha_));

        subscription_ = node_->create_subscription<convex_plane_decomposition_msgs::msg::PlanarTerrain>(
            mapTopic, 10, [this](const convex_plane_decomposition_msgs::msg::PlanarTerrain msg)
            {
                auto candidate = convex_plane_decomposition::PlanarTerrain(
                    convex_plane_decomposition::fromMessage(msg));

                if (!sanitizeElevationLayer(candidate))
                {
                    return;
                }

                std::lock_guard lock(mutex_);
                const auto timestamp = candidate.gridMap.getTimestamp();
                if (!acceptTimestamp(timestamp))
                {
                    return;
                }

                double centerJump = 0.0;
                if (!acceptMapCenter(candidate, centerJump))
                {
                    return;
                }

                filteredTerrain_ = filterTerrain(candidate, centerJump);
                updateSignedDistanceField(filteredTerrain_);
                pendingTerrain_ = filteredTerrain_;
                lastAcceptedTimestamp_ = timestamp;
                hasFilteredTerrain_ = true;
                updated_ = true;
            });
    }

    void PlanarTerrainReceiver::preSolverRun(scalar_t /*initTime*/, scalar_t /*finalTime*/,
                                             const vector_t& /*currentState*/,
                                             const ReferenceManagerInterface& /*referenceManager*/)
    {
        if (updated_)
        {
            std::lock_guard lock(mutex_);
            updated_ = false;
            *planarTerrainPtr_ = pendingTerrain_;
        }
    }

    bool PlanarTerrainReceiver::sanitizeElevationLayer(
        convex_plane_decomposition::PlanarTerrain& planarTerrain) const
    {
        if (!planarTerrain.gridMap.exists(sdfElevationLayer_))
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "[PlanarTerrainReceiver] Dropping terrain map without layer '%s'.",
                                 sdfElevationLayer_.c_str());
            return false;
        }

        auto& elevationData = planarTerrain.gridMap.get(sdfElevationLayer_);
        if (elevationData.hasNaN())
        {
            if (!elevationData.array().isFinite().any())
            {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                     "[PlanarTerrainReceiver] Dropping terrain map because layer '%s' has no finite values.",
                                     sdfElevationLayer_.c_str());
                return false;
            }

            const float inpaint{elevationData.minCoeffOfFinites()};
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "[PlanarTerrainReceiver] Map contains NaN values. Applying min-value inpainting.");
            elevationData = elevationData.unaryExpr([=](float v) { return std::isfinite(v) ? v : inpaint; });
        }
        return true;
    }

    bool PlanarTerrainReceiver::acceptTimestamp(grid_map::Time timestamp) const
    {
        if (timestamp == 0)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "[PlanarTerrainReceiver] Received terrain map with zero timestamp; age check skipped.");
            return true;
        }

        if (lastAcceptedTimestamp_ > 0 && timestamp < lastAcceptedTimestamp_)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "[PlanarTerrainReceiver] Dropping older terrain map timestamp.");
            return false;
        }

        const double mapTime = static_cast<double>(timestamp) * 1e-9;
        const double now = node_->now().seconds();
        if (now <= 0.0)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "[PlanarTerrainReceiver] ROS time is zero; terrain age check skipped.");
            return true;
        }

        const double age = now - mapTime;
        if (maxMapAge_ > 0.0 && age > maxMapAge_)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "[PlanarTerrainReceiver] Dropping stale terrain map. age=%.3f s, max=%.3f s.",
                                 age, maxMapAge_);
            return false;
        }
        if (futureTimestampTolerance_ > 0.0 && age < -futureTimestampTolerance_)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "[PlanarTerrainReceiver] Dropping future-dated terrain map. lead=%.3f s, max=%.3f s.",
                                 -age, futureTimestampTolerance_);
            return false;
        }
        return true;
    }

    bool PlanarTerrainReceiver::acceptMapCenter(
        const convex_plane_decomposition::PlanarTerrain& planarTerrain,
        double& centerJump) const
    {
        centerJump = 0.0;
        if (!hasFilteredTerrain_)
        {
            return true;
        }

        const auto& previousMap = filteredTerrain_.gridMap;
        const auto& candidateMap = planarTerrain.gridMap;
        if (!previousMap.getFrameId().empty() && !candidateMap.getFrameId().empty() &&
            previousMap.getFrameId() != candidateMap.getFrameId())
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "[PlanarTerrainReceiver] Dropping terrain map with changed frame_id '%s' -> '%s'.",
                                 previousMap.getFrameId().c_str(), candidateMap.getFrameId().c_str());
            return false;
        }

        centerJump = (candidateMap.getPosition() - previousMap.getPosition()).norm();
        if (maxCenterJump_ > 0.0 && centerJump > maxCenterJump_)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "[PlanarTerrainReceiver] Large terrain map center jump %.3f m > %.3f m; accepting map and resetting filter.",
                                 centerJump, maxCenterJump_);
        }
        return true;
    }

    bool PlanarTerrainReceiver::hasFilterCompatibleGeometry(
        const grid_map::GridMap& lhs, const grid_map::GridMap& rhs) const
    {
        constexpr double geometryTolerance{1e-6};
        return lhs.getFrameId() == rhs.getFrameId() &&
               (lhs.getSize() == rhs.getSize()).all() &&
               (lhs.getLength() - rhs.getLength()).matrix().norm() < geometryTolerance &&
               std::abs(lhs.getResolution() - rhs.getResolution()) < geometryTolerance &&
               lhs.exists(sdfElevationLayer_) &&
               rhs.exists(sdfElevationLayer_);
    }

    convex_plane_decomposition::PlanarTerrain PlanarTerrainReceiver::filterTerrain(
        const convex_plane_decomposition::PlanarTerrain& candidate,
        double centerJump) const
    {
        if (!hasFilteredTerrain_ ||
            !hasFilterCompatibleGeometry(filteredTerrain_.gridMap, candidate.gridMap) ||
            centerJump > centerFilterThreshold_)
        {
            return candidate;
        }

        auto filtered = candidate;
        auto& filteredElevation = filtered.gridMap.get(sdfElevationLayer_);
        const auto& previousElevation = filteredTerrain_.gridMap.get(sdfElevationLayer_);
        const auto alpha = static_cast<float>(heightFilterAlpha_);
        const auto beta = static_cast<float>(1.0 - heightFilterAlpha_);
        filteredElevation = filteredElevation.binaryExpr(
            previousElevation,
            [alpha, beta](float current, float previous)
            {
                if (std::isfinite(current) && std::isfinite(previous))
                {
                    return alpha * current + beta * previous;
                }
                return std::isfinite(current) ? current : previous;
            });

        filtered.gridMap.setPosition(filteredTerrain_.gridMap.getPosition());
        return filtered;
    }

    void PlanarTerrainReceiver::updateSignedDistanceField(
        const convex_plane_decomposition::PlanarTerrain& planarTerrain) const
    {
        constexpr float heightMargin{0.1};
        const auto& elevationData = planarTerrain.gridMap.get(sdfElevationLayer_);
        const float maxValue{elevationData.maxCoeffOfFinites() + 3 * heightMargin};
        sdfPtr_->calculateSignedDistanceField(planarTerrain.gridMap, sdfElevationLayer_, maxValue);
    }

    double PlanarTerrainReceiver::getOrDeclareDoubleParameter(
        const std::string& name, double defaultValue) const
    {
        if (!node_->has_parameter(name))
        {
            node_->declare_parameter<double>(name, defaultValue);
        }
        return node_->get_parameter(name).as_double();
    }
} // namespace legged
