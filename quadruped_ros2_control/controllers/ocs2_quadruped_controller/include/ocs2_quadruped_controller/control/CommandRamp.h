#pragma once

#include <algorithm>
#include <cmath>

namespace ocs2::legged_robot {
struct CommandRampSample {
    double velocity;
    double displacement;
};

// Exact integral of an acceleration-limited velocity command, including stopping.
inline CommandRampSample sampleCommandRamp(double initial, double target, double acceleration, double time) {
    const double duration = std::abs(target - initial) / acceleration;
    const double rampTime = std::min(std::max(time, 0.0), duration);
    const double slope = target >= initial ? acceleration : -acceleration;
    return {initial + slope * rampTime,
            initial * rampTime + 0.5 * slope * rampTime * rampTime + target * std::max(0.0, time - duration)};
}
}  // namespace ocs2::legged_robot
