#pragma once

#include <vector>

#include "ocs2_quadruped_controller/interface/constraint/QuinticSpline.h"

namespace ocs2::legged_robot {

/**
 * 两段凸形样条拼接的摆动足高度轨迹生成器 (Central Pattern Generator)。
 *
 * 左段: liftOff  → midPoint (midTime_, midHeight)
 * 右段: midPoint → touchDown
 *
 * 默认轨迹使用两段无拐点的三次多项式，避免五次 Hermite smootherstep
 * 在每半段内部形成 S 形曲线。保留类名以兼容已有接口。
 */
class QuinticSplineCpg {
public:
    QuinticSplineCpg(QuinticSpline::Node liftOff, scalar_t midHeight, QuinticSpline::Node touchDown);
    QuinticSplineCpg(std::vector<scalar_t> times,
                     std::vector<scalar_t> positions,
                     std::vector<scalar_t> velocities,
                     std::vector<scalar_t> accelerations);
    QuinticSplineCpg(const QuinticSplineCpg& other);
    QuinticSplineCpg& operator=(const QuinticSplineCpg& other);
    QuinticSplineCpg(QuinticSplineCpg&& other) noexcept = default;
    QuinticSplineCpg& operator=(QuinticSplineCpg&& other) noexcept = default;
    ~QuinticSplineCpg() = default;

    scalar_t position(scalar_t time) const;
    scalar_t velocity(scalar_t time) const;
    scalar_t acceleration(scalar_t time) const;

    scalar_t startTimeDerivative(scalar_t time) const;
    scalar_t finalTimeDerivative(scalar_t time) const;

private:
    static scalar_t evaluateConvexPosition(scalar_t time,
                                           const QuinticSpline::Node& liftOff,
                                           scalar_t midHeight,
                                           const QuinticSpline::Node& touchDown);
    scalar_t evaluateConvexVelocity(scalar_t time) const;
    scalar_t evaluateConvexAcceleration(scalar_t time) const;

    scalar_t evaluateSampledSignal(const std::vector<scalar_t>& signal, scalar_t time) const;
    size_t findSegmentIndex(scalar_t time) const;

    bool useSampledTrajectory_ = false;
    scalar_t midTime_ = 0.0;
    scalar_t midHeight_ = 0.0;
    QuinticSpline::Node liftOff_{0.0, 0.0, 0.0, 0.0};
    QuinticSpline::Node touchDown_{1.0, 0.0, 0.0, 0.0};
    std::vector<scalar_t> sampleTimes_;
    std::vector<scalar_t> samplePositions_;
    std::vector<scalar_t> sampleVelocities_;
    std::vector<scalar_t> sampleAccelerations_;
};

} // namespace ocs2::legged_robot
