#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include "ocs2_legged_robot/common/Types.h"
#include "ocs2_legged_robot/synchronized_module/SwitchedModelReferenceManager.h"

namespace ocs2 {
namespace legged_robot {

class FootPlacementConstraint final : public StateInputConstraint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FootPlacementConstraint(
      const SwitchedModelReferenceManager& referenceManager,
      const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
      size_t contactPointIndex);

  ~FootPlacementConstraint() override = default;
  FootPlacementConstraint* clone() const override {
    return new FootPlacementConstraint(*this);
  }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override { return 4; }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input,
                    const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(
      scalar_t time, const vector_t& state, const vector_t& input,
      const PreComputation& preComp) const override;

 private:
  FootPlacementConstraint(const FootPlacementConstraint& rhs);

  const SwitchedModelReferenceManager* referenceManagerPtr_;
  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
  const size_t contactPointIndex_;

  scalar_t terrainGap_;    // defalut: 0.3m
  scalar_t terrainSizeX_;  // defalut: 0.2m
  scalar_t terrainSizeY_;  // defalut: 1.0m
};
}  // namespace legged_robot
}  // namespace ocs2