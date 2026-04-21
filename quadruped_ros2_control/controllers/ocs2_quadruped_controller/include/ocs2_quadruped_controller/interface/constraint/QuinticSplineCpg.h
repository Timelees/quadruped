#pragma once

#include <memory>
#include <vector>

#include "ocs2_quadruped_controller/interface/constraint/QuinticSpline.h"

namespace ocs2::legged_robot {

/**
 * 两段五次样条拼接的摆动足高度轨迹生成器 (Central Pattern Generator)。
 *
 * 左段: liftOff  → midPoint (midTime_, midHeight, v=0, a=0)
 * 右段: midPoint → touchDown
 *
 * 中点处 velocity=0, acceleration=0 保证拼接点 C² 连续，
 * 相比原始两段三次样条（仅 C¹ 连续）加速度更平滑。
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
    scalar_t evaluateSampledSignal(const std::vector<scalar_t>& signal, scalar_t time) const;
    size_t findSegmentIndex(scalar_t time) const;

    bool useSampledTrajectory_ = false;
    scalar_t midTime_ = 0.0;
    std::unique_ptr<QuinticSpline> leftSpline_;
    std::unique_ptr<QuinticSpline> rightSpline_;
    std::vector<scalar_t> sampleTimes_;
    std::vector<scalar_t> samplePositions_;
    std::vector<scalar_t> sampleVelocities_;
    std::vector<scalar_t> sampleAccelerations_;
};

} // namespace ocs2::legged_robot
