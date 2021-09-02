#include "ocs2_legged_robot/constraint/FootPlacementConstraint.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace ocs2 {
namespace legged_robot {

FootPlacementConstraint::FootPlacementConstraint(
    const SwitchedModelReferenceManager& referenceManager,
    const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
    size_t contactPointIndex)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      contactPointIndex_(contactPointIndex),
      terrainGap_(0.3),
      terrainSizeX_(0.2),
      terrainSizeY_(1.0) {
  if (endEffectorKinematicsPtr_->getIds().size() != 1) {
    throw std::runtime_error(
        "[FootPlacementConstraint] this class only accepts a single "
        "end-effector!");
  }
}

bool FootPlacementConstraint::isActive(scalar_t time) const {
  const scalar_t trotGaitWholeCycle = 0.7;
  const scalar_t currentTime = referenceManagerPtr_->getCurrentTime();
  assert(time >= currentTime);
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_] &&
         time <= currentTime + trotGaitWholeCycle;  // only consider one cycle
}

FootPlacementConstraint::FootPlacementConstraint(
    const FootPlacementConstraint& rhs)
    : StateInputConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      contactPointIndex_(rhs.contactPointIndex_) {}

vector_t FootPlacementConstraint::getValue(
    scalar_t time, const vector_t& state, const vector_t& input,
    const PreComputation& preComp) const {
  // Step 0 get start (current) time
  const scalar_t trotGaitWholeCycle = 0.7;
  const scalar_t currentTime = referenceManagerPtr_->getCurrentTime();
  assert(time >= currentTime);
  assert(time <= currentTime + trotGaitWholeCycle);

  // Step 1 get middle of the last stance phase
  scalar_t middle =
      referenceManagerPtr_->getGaitSchedule()->getMiddleOfLastStance(time);

  // Step 2 get target state
  const TargetTrajectories& targetTrajectories =
      referenceManagerPtr_->getTargetTrajectories();
  const vector_t stateReference = targetTrajectories.getDesiredState(middle);

  // Step 3 get target end-effector position
  assert(endEffectorKinematicsPtr_->getIds().size() == 1);
  vector3_t pEEReference =
      endEffectorKinematicsPtr_->getPosition(stateReference).front();

  // Step 4 find a closest region (A,b)
  scalar_t xEE = pEEReference(0);
  scalar_t yEE = pEEReference(1);
  scalar_t terrainIndex = std::round(xEE / terrainGap_);
  scalar_t xMin = terrainIndex * terrainGap_ - terrainSizeX_ / 2;
  scalar_t XMax = terrainIndex * terrainGap_ + terrainSizeX_ / 2;
  scalar_t yMin = 0 - terrainSizeY_ / 2;
  scalar_t yMax = 0 + terrainSizeY_ / 2;
  Eigen::Matrix<scalar_t, 4, 3> A;
  A << 1, 0, 0, -1, 0, 0, 0, 1, 0, 0, -1, 0;
  Eigen::Matrix<scalar_t, 4, 1> b;
  b << -xMin, XMax, -yMin, yMax;

  // Step 5 s = get swing time left (0.35 -> 0)
  scalar_t swingTimeLeft =
      referenceManagerPtr_->getGaitSchedule()->getSwingTimeLeft();
  assert(swingTimeLeft >= 0);
  assert(swingTimeLeft <= trotGaitWholeCycle / 2);
  Eigen::Matrix<scalar_t, 4, 1> s;
  s << swingTimeLeft, swingTimeLeft, swingTimeLeft, swingTimeLeft;

  // Step 6 add constraint Ax + b + s >=0
  vector3_t pEE = endEffectorKinematicsPtr_->getPosition(state).front();
  return A * pEE + b + s;
}

VectorFunctionLinearApproximation
FootPlacementConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const vector_t& input,
    const PreComputation& preComp) const {
  VectorFunctionLinearApproximation approx;
  approx.f = getValue(time, state, input, preComp);

  Eigen::Matrix<scalar_t, 4, 3> A;
  A << 1, 0, 0, -1, 0, 0, 0, 1, 0, 0, -1, 0;
  approx.dfdx = A;
  return approx;
}
}  // namespace legged_robot
}  // namespace ocs2