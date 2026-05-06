#include "ocs2_quadruped_controller/interface/constraint/QuinticSplineCpg.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace ocs2::legged_robot {

namespace {

struct ConvexSegment {
    scalar_t startTime = 0.0;
    scalar_t duration = 1.0;
    scalar_t position0 = 0.0;
    scalar_t c1 = 0.0;
    scalar_t c2 = 0.0;
    scalar_t c3 = 0.0;
};

scalar_t convexEndpointSlope(scalar_t requestedSlope, scalar_t delta) {
    const scalar_t absDelta = std::abs(delta);
    if (absDelta <= 1e-12) {
        return 0.0;
    }

    const scalar_t sign = (delta >= 0.0) ? 1.0 : -1.0;
    const scalar_t directionalRequested = std::max<scalar_t>(0.0, sign * requestedSlope);
    const scalar_t parabolicSlope = 2.0 * absDelta;
    const scalar_t blendedSlope = 0.75 * parabolicSlope + 0.25 * directionalRequested;
    return sign * std::min(std::max(blendedSlope, 1.75 * absDelta), 2.25 * absDelta);
}

ConvexSegment makeConvexSegment(scalar_t time,
                                const QuinticSpline::Node& liftOff,
                                scalar_t midTime,
                                scalar_t midHeight,
                                const QuinticSpline::Node& touchDown) {
    ConvexSegment segment;
    segment.duration = midTime - liftOff.time;
    if (segment.duration <= 1e-12) {
        segment.startTime = liftOff.time;
        segment.duration = 1.0;
        segment.position0 = midHeight;
        return segment;
    }

    const bool left = time < midTime;
    segment.startTime = left ? liftOff.time : midTime;
    segment.position0 = left ? liftOff.position : midHeight;
    const scalar_t position1 = left ? midHeight : touchDown.position;
    const scalar_t delta = position1 - segment.position0;
    const scalar_t requestedSlope = (left ? liftOff.velocity : touchDown.velocity) * segment.duration;
    const scalar_t endpointSlope = convexEndpointSlope(requestedSlope, delta);

    if (left) {
        segment.c1 = endpointSlope;
        segment.c2 = 3.0 * delta - 2.0 * segment.c1;
        segment.c3 = segment.c1 - 2.0 * delta;
    } else {
        segment.c1 = 0.0;
        segment.c2 = 3.0 * delta - endpointSlope;
        segment.c3 = endpointSlope - 2.0 * delta;
    }

    return segment;
}

scalar_t normalizedSegmentTime(scalar_t time, const ConvexSegment& segment) {
    return (time - segment.startTime) / segment.duration;
}

scalar_t evaluatePosition(scalar_t time, const ConvexSegment& segment) {
    const scalar_t s = normalizedSegmentTime(time, segment);
    return segment.position0 + s * (segment.c1 + s * (segment.c2 + s * segment.c3));
}

scalar_t evaluateVelocity(scalar_t time, const ConvexSegment& segment) {
    const scalar_t s = normalizedSegmentTime(time, segment);
    return (segment.c1 + s * (2.0 * segment.c2 + s * 3.0 * segment.c3)) / segment.duration;
}

scalar_t evaluateAcceleration(scalar_t time, const ConvexSegment& segment) {
    const scalar_t s = normalizedSegmentTime(time, segment);
    return (2.0 * segment.c2 + 6.0 * segment.c3 * s) / (segment.duration * segment.duration);
}

} // namespace

QuinticSplineCpg::QuinticSplineCpg(QuinticSpline::Node liftOff, scalar_t midHeight, QuinticSpline::Node touchDown)
    : midTime_((liftOff.time + touchDown.time) / 2.0),
      midHeight_(midHeight),
      liftOff_(liftOff),
      touchDown_(touchDown) {
    if (!(liftOff_.time < touchDown_.time)) {
        throw std::invalid_argument("Convex QuinticSplineCpg requires liftOff.time < touchDown.time");
    }
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
      midHeight_(other.midHeight_),
      liftOff_(other.liftOff_),
      touchDown_(other.touchDown_),
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
    midHeight_ = other.midHeight_;
    liftOff_ = other.liftOff_;
    touchDown_ = other.touchDown_;
    sampleTimes_ = other.sampleTimes_;
    samplePositions_ = other.samplePositions_;
    sampleVelocities_ = other.sampleVelocities_;
    sampleAccelerations_ = other.sampleAccelerations_;

    return *this;
}

scalar_t QuinticSplineCpg::evaluateConvexPosition(scalar_t time,
                                                  const QuinticSpline::Node& liftOff,
                                                  scalar_t midHeight,
                                                  const QuinticSpline::Node& touchDown) {
    const scalar_t midTime = (liftOff.time + touchDown.time) / 2.0;
    return evaluatePosition(time, makeConvexSegment(time, liftOff, midTime, midHeight, touchDown));
}

scalar_t QuinticSplineCpg::evaluateConvexVelocity(scalar_t time) const {
    return evaluateVelocity(time, makeConvexSegment(time, liftOff_, midTime_, midHeight_, touchDown_));
}

scalar_t QuinticSplineCpg::evaluateConvexAcceleration(scalar_t time) const {
    return evaluateAcceleration(time, makeConvexSegment(time, liftOff_, midTime_, midHeight_, touchDown_));
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
    return evaluateConvexPosition(time, liftOff_, midHeight_, touchDown_);
}

scalar_t QuinticSplineCpg::velocity(scalar_t time) const {
    if (useSampledTrajectory_) {
        return evaluateSampledSignal(sampleVelocities_, time);
    }
    return evaluateConvexVelocity(time);
}

scalar_t QuinticSplineCpg::acceleration(scalar_t time) const {
    if (useSampledTrajectory_) {
        return evaluateSampledSignal(sampleAccelerations_, time);
    }
    return evaluateConvexAcceleration(time);
}

scalar_t QuinticSplineCpg::startTimeDerivative(scalar_t time) const {
    if (useSampledTrajectory_) {
        return 0.0;
    }
    const scalar_t duration = touchDown_.time - liftOff_.time;
    const scalar_t eps = std::max<scalar_t>(1e-7, 1e-6 * duration);
    QuinticSpline::Node liftOffPlus = liftOff_;
    QuinticSpline::Node liftOffMinus = liftOff_;
    liftOffPlus.time += eps;
    liftOffMinus.time -= eps;
    return (evaluateConvexPosition(time, liftOffPlus, midHeight_, touchDown_) -
            evaluateConvexPosition(time, liftOffMinus, midHeight_, touchDown_)) /
           (2.0 * eps);
}

scalar_t QuinticSplineCpg::finalTimeDerivative(scalar_t time) const {
    if (useSampledTrajectory_) {
        return 0.0;
    }
    const scalar_t duration = touchDown_.time - liftOff_.time;
    const scalar_t eps = std::max<scalar_t>(1e-7, 1e-6 * duration);
    QuinticSpline::Node touchDownPlus = touchDown_;
    QuinticSpline::Node touchDownMinus = touchDown_;
    touchDownPlus.time += eps;
    touchDownMinus.time -= eps;
    return (evaluateConvexPosition(time, liftOff_, midHeight_, touchDownPlus) -
            evaluateConvexPosition(time, liftOff_, midHeight_, touchDownMinus)) /
           (2.0 * eps);
}

} // namespace ocs2::legged_robot
