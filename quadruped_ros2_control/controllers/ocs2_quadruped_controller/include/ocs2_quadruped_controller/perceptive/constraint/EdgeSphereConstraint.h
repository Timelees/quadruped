#pragma once

#include <memory>

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include "ocs2_quadruped_controller/interface/SwitchedModelReferenceManager.h"
#include "ocs2_quadruped_controller/perceptive/synchronize/EdgeSphere.h"

namespace ocs2::legged_robot {

class EdgeSphereConstraint final : public StateConstraint {
 public:
  EdgeSphereConstraint(const SwitchedModelReferenceManager& referenceManager,
                    const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                    std::shared_ptr<EdgeSphereArray> edgeSpheresPtr,
                    size_t contactPointIndex,
                    scalar_t inflation);

  ~EdgeSphereConstraint() override = default;

  EdgeSphereConstraint* clone() const override { return new EdgeSphereConstraint(*this); }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t /*time*/) const override { return 1; }
  vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComp) const override;

 private:
  EdgeSphereConstraint(const EdgeSphereConstraint& rhs);

  const SwitchedModelReferenceManager* referenceManagerPtr_;
  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
  std::shared_ptr<EdgeSphereArray> edgeSpheresPtr_;
  size_t contactPointIndex_;
  scalar_t inflation_;
};

}  // namespace ocs2::legged_robot
