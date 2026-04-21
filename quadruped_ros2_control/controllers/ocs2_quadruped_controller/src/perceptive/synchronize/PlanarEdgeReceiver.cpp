//
// Created by GitHub Copilot on 3/24/26.
//

#include "ocs2_quadruped_controller/perceptive/synchronize/PlanarEdgeReceiver.h"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cmath>

namespace ocs2::legged_robot {

PlanarEdgeReceiver::PlanarEdgeReceiver(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
															 const std::shared_ptr<EdgeSphereArray>& edgeSpheresPtr,
															 const std::string& edgeTopic,
															 Config config)
		: node_(node), config_(config), edgeSpheresPtr_(edgeSpheresPtr) {
	subscription_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
			edgeTopic, rclcpp::QoS(1).best_effort(), [this](const sensor_msgs::msg::PointCloud2& msg) {
				const auto points = parsePointCloud(msg);
				auto spheres = buildSpheres(points);
				{
					std::lock_guard<std::mutex> lock(mutex_);
					edgeSpheres_ = std::move(spheres);
					updated_ = true;
				}
			});
}

void PlanarEdgeReceiver::updateEdgeSpheres() {
	std::lock_guard<std::mutex> lock(mutex_);
	if (!updated_) {
		return;
	}
	updated_ = false;
	*edgeSpheresPtr_ = edgeSpheres_;
}

void PlanarEdgeReceiver::preSolverRun(scalar_t /*initTime*/, scalar_t /*finalTime*/, const vector_t& /*currentState*/,
																			const ReferenceManagerInterface& /*referenceManager*/) {
	updateEdgeSpheres();
}

PlanarEdgeReceiver::PointCloud PlanarEdgeReceiver::parsePointCloud(const sensor_msgs::msg::PointCloud2& message) {
	PointCloud points;
	if (message.width == 0 || message.height == 0) {
		return points;
	}

	points.reserve(static_cast<size_t>(message.width) * static_cast<size_t>(message.height));
	sensor_msgs::PointCloud2ConstIterator<float> iterX(message, "x");
	sensor_msgs::PointCloud2ConstIterator<float> iterY(message, "y");
	sensor_msgs::PointCloud2ConstIterator<float> iterZ(message, "z");
	for (; iterX != iterX.end(); ++iterX, ++iterY, ++iterZ) {
		if (std::isfinite(*iterX) && std::isfinite(*iterY) && std::isfinite(*iterZ)) {
			points.emplace_back(*iterX, *iterY, *iterZ);
		}
	}
	return points;
}

EdgeSphereArray PlanarEdgeReceiver::buildSpheres(const PointCloud& points) const {
	EdgeSphereArray spheres;
	spheres.reserve(points.size());
	for (const auto& pt : points) {
		EdgeSphere s;
		s.center = pt;
		s.radius = config_.sphereRadius;
		spheres.push_back(s);
	}
	return spheres;
}

}  // namespace ocs2::legged_robot
