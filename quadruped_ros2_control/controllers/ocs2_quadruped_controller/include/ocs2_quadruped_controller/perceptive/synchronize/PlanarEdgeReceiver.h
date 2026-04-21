#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include "ocs2_quadruped_controller/perceptive/synchronize/EdgeSphere.h"

namespace ocs2::legged_robot {

class PlanarEdgeReceiver : public SolverSynchronizedModule {
 public:
  struct Config {
    Config() : sphereRadius(0.025) {}

    double sphereRadius;  // 2.5 cm
  };

  PlanarEdgeReceiver(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                     const std::shared_ptr<EdgeSphereArray>& edgeSpheresPtr,
                     const std::string& edgeTopic,
                     Config config = Config());

  void updateEdgeSpheres();

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const PrimalSolution& primalSolution) override {}

 private:
  using PointCloud = std::vector<Eigen::Vector3d>;

  static PointCloud parsePointCloud(const sensor_msgs::msg::PointCloud2& message);
  EdgeSphereArray buildSpheres(const PointCloud& points) const;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  Config config_;
  std::mutex mutex_;
  bool updated_ = false;

  EdgeSphereArray edgeSpheres_;
  std::shared_ptr<EdgeSphereArray> edgeSpheresPtr_;
};

}  // namespace ocs2::legged_robot