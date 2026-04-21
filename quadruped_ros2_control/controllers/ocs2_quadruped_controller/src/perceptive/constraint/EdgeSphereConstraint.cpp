
#include "ocs2_quadruped_controller/perceptive/constraint/EdgeSphereConstraint.h"

namespace ocs2::legged_robot {

EdgeSphereConstraint::EdgeSphereConstraint(const SwitchedModelReferenceManager& referenceManager,
                                     const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                     std::shared_ptr<EdgeSphereArray> edgeSpheresPtr,
                                     size_t contactPointIndex,
                                     scalar_t inflation)
    : StateConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      edgeSpheresPtr_(std::move(edgeSpheresPtr)),
      contactPointIndex_(contactPointIndex),
      inflation_(inflation) {}

EdgeSphereConstraint::EdgeSphereConstraint(const EdgeSphereConstraint& rhs)
    : StateConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      edgeSpheresPtr_(rhs.edgeSpheresPtr_),
      contactPointIndex_(rhs.contactPointIndex_),
      inflation_(rhs.inflation_) {}

bool EdgeSphereConstraint::isActive(scalar_t time) const {
  const scalar_t offset = 0.05;
  const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
  const auto contactFlagsFuture = referenceManagerPtr_->getContactFlags(time + 0.5 * offset);
  const auto contactFlagsPast = referenceManagerPtr_->getContactFlags(time - offset);
  return !contactFlags[contactPointIndex_] && !contactFlagsFuture[contactPointIndex_] &&
         !contactFlagsPast[contactPointIndex_];
}

vector_t EdgeSphereConstraint::getValue(scalar_t /*time*/, const vector_t& state,
                                     const PreComputation& /*preComp*/) const {
  vector_t value(1);
  if (edgeSpheresPtr_->empty()) {
    value(0) = 1.0;
    return value;
  }

  const Eigen::Vector3d position = endEffectorKinematicsPtr_->getPosition(state).front();
  const size_t closest = findClosestSphere(*edgeSpheresPtr_, position, inflation_);
  value(0) = edgeSpheresPtr_->at(closest).levelSetValue(position, inflation_);
  return value;
}

VectorFunctionLinearApproximation EdgeSphereConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                            const PreComputation& preComp) const {
  VectorFunctionLinearApproximation approx = VectorFunctionLinearApproximation::Zero(1, state.size(), 0);
  approx.f = getValue(time, state, preComp);

  if (edgeSpheresPtr_->empty()) {
    return approx;
  }

  const Eigen::Vector3d position = endEffectorKinematicsPtr_->getPosition(state).front();
  const size_t closest = findClosestSphere(*edgeSpheresPtr_, position, inflation_);
  const Eigen::Vector3d gradient = edgeSpheresPtr_->at(closest).gradient(position, inflation_);
  approx.dfdx = gradient.transpose() * endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front().dfdx;
  return approx;
}

}  // namespace ocs2::legged_robot
