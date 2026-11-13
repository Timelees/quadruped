/*
 * SignedDistanceField.hpp
 *
 *  Created on: Aug 16, 2017
 *     Authors: Takahiro Miki, Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <grid_map_core/GridMap.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>
#include <mutex>
namespace grid_map
{
    class SignedDistanceField
    {
    public:
        SignedDistanceField();
        virtual ~SignedDistanceField();

        // Exchange a completed field without rebuilding it on the solver thread.
        void swap(SignedDistanceField& other);

        void calculateSignedDistanceField(
            const GridMap& gridMap, const std::string& layer,
            double heightClearance);
        double getDistanceAt(const Position3& position) const;
        Vector3 getDistanceGradientAt(const Position3& position) const;
        double getInterpolatedDistanceAt(const Position3& position) const;
        void convertToPointCloud(pcl::PointCloud<pcl::PointXYZI>& points) const;
        std::vector<Matrix>& getData()  {
            std::lock_guard<std::mutex> lock(sdfMutex_);
            return data_;
        }

    private:
        Matrix getPlanarSignedDistanceField(
            Eigen::Matrix<bool, Eigen::Dynamic,
                          Eigen::Dynamic>& data) const;

        Size size_;
        Position position_;
        std::vector<Matrix> data_;
        float maxDistance_;
        float zIndexStartHeight_;
        double resolution_;
        const float lowestHeight_;
        mutable std::mutex sdfMutex_; // Add mutex for thread safety
    };
} // namespace grid_map
