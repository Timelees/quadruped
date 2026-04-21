#include "ocs2_quadruped_controller/interface/constraint/QuinticSplineCpg.h"

#include <algorithm>
#include <stdexcept>

namespace ocs2::legged_robot {

QuinticSplineCpg::QuinticSplineCpg(QuinticSpline::Node liftOff, scalar_t midHeight, QuinticSpline::Node touchDown)
    : midTime_((liftOff.time + touchDown.time) / 2.0),
      leftSpline_(std::make_unique<QuinticSpline>(liftOff, QuinticSpline::Node{midTime_, midHeight, 0.0, 0.0})),
      rightSpline_(std::make_unique<QuinticSpline>(QuinticSpline::Node{midTime_, midHeight, 0.0, 0.0}, touchDown)) {
}

QuinticSplineCpg::QuinticSplineCpg(std::vector<scalar_t> times,
                                   std::vector<scalar_t> positions,
                                   std::vector<scalar_t> velocities,
                                   std::vector<scalar_t> accelerations)
    : useSampledTrajectory_(true),
      sampleTimes_(std::move(times)),
      samplePositions_(std::move(positions)),
      sampleVelocities_(std::move(velocities)),
      sampleAccelerations_(std::move(accelerations)) {
    const bool validSizes = sampleTimes_.size() >= 2 &&
                            sampleTimes_.size() == samplePositions_.size() &&
                            sampleTimes_.size() == sampleVelocities_.size() &&
                            sampleTimes_.size() == sampleAccelerations_.size();
    if (!validSizes) {
        throw std::invalid_argument("Sampled QuinticSplineCpg data has inconsistent sizes");
    }

    if (!std::is_sorted(sampleTimes_.begin(), sampleTimes_.end())) {
        throw std::invalid_argument("Sampled QuinticSplineCpg times must be sorted");
    }

    midTime_ = sampleTimes_[sampleTimes_.size() / 2];
}

QuinticSplineCpg::QuinticSplineCpg(const QuinticSplineCpg& other)
    : useSampledTrajectory_(other.useSampledTrajectory_),
      midTime_(other.midTime_),
      leftSpline_(other.leftSpline_ ? std::make_unique<QuinticSpline>(*other.leftSpline_) : nullptr),
      rightSpline_(other.rightSpline_ ? std::make_unique<QuinticSpline>(*other.rightSpline_) : nullptr),
      sampleTimes_(other.sampleTimes_),
      samplePositions_(other.samplePositions_),
      sampleVelocities_(other.sampleVelocities_),
      sampleAccelerations_(other.sampleAccelerations_) {
}

QuinticSplineCpg& QuinticSplineCpg::operator=(const QuinticSplineCpg& other) {
    if (this == &other) {
        return *this;
    }

    useSampledTrajectory_ = other.useSampledTrajectory_;
    midTime_ = other.midTime_;
    leftSpline_ = other.leftSpline_ ? std::make_unique<QuinticSpline>(*other.leftSpline_) : nullptr;
    rightSpline_ = other.rightSpline_ ? std::make_unique<QuinticSpline>(*other.rightSpline_) : nullptr;
    sampleTimes_ = other.sampleTimes_;
    samplePositions_ = other.samplePositions_;
    sampleVelocities_ = other.sampleVelocities_;
    sampleAccelerations_ = other.sampleAccelerations_;

    return *this;
}

size_t QuinticSplineCpg::findSegmentIndex(scalar_t time) const {
    if (time <= sampleTimes_.front()) {
        return 0;
    }
    if (time >= sampleTimes_.back()) {
        return sampleTimes_.size() - 2;
    }

    const auto upper = std::upper_bound(sampleTimes_.begin(), sampleTimes_.end(), time);
    return static_cast<size_t>(std::distance(sampleTimes_.begin(), upper) - 1);
}

scalar_t QuinticSplineCpg::evaluateSampledSignal(const std::vector<scalar_t>& signal, scalar_t time) const {
    if (time <= sampleTimes_.front()) {
        return signal.front();
    }
    if (time >= sampleTimes_.back()) {
        return signal.back();
    }

    const size_t index = findSegmentIndex(time);
    const scalar_t t0 = sampleTimes_[index];
    const scalar_t t1 = sampleTimes_[index + 1];
    const scalar_t alpha = (t1 > t0) ? (time - t0) / (t1 - t0) : 0.0;
    return signal[index] + alpha * (signal[index + 1] - signal[index]);
}

scalar_t QuinticSplineCpg::position(scalar_t time) const {
    if (useSampledTrajectory_) {
        return evaluateSampledSignal(samplePositions_, time);
    }
    return (time < midTime_) ? leftSpline_->position(time) : rightSpline_->position(time);
}

scalar_t QuinticSplineCpg::velocity(scalar_t time) const {
    if (useSampledTrajectory_) {
        return evaluateSampledSignal(sampleVelocities_, time);
    }
    return (time < midTime_) ? leftSpline_->velocity(time) : rightSpline_->velocity(time);
}

scalar_t QuinticSplineCpg::acceleration(scalar_t time) const {
    if (useSampledTrajectory_) {
        return evaluateSampledSignal(sampleAccelerations_, time);
    }
    return (time < midTime_) ? leftSpline_->acceleration(time) : rightSpline_->acceleration(time);
}

scalar_t QuinticSplineCpg::startTimeDerivative(scalar_t time) const {
    if (useSampledTrajectory_) {
        return 0.0;
    }
    if (time <= midTime_) {
        return leftSpline_->startTimeDerivative(time) + 0.5 * leftSpline_->finalTimeDerivative(time);
    }
    return 0.5 * rightSpline_->startTimeDerivative(time);
}

scalar_t QuinticSplineCpg::finalTimeDerivative(scalar_t time) const {
    if (useSampledTrajectory_) {
        return 0.0;
    }
    if (time <= midTime_) {
        return 0.5 * leftSpline_->finalTimeDerivative(time);
    }
    return rightSpline_->finalTimeDerivative(time) + 0.5 * rightSpline_->startTimeDerivative(time);
}

} // namespace ocs2::legged_robot
