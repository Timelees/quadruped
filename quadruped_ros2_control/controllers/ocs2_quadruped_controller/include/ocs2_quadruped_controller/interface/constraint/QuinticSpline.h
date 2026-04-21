#pragma once

#include <ocs2_core/Types.h>

namespace ocs2::legged_robot {

/**
 * 五次 Hermite 样条：在两个端点分别指定 position / velocity / acceleration，
 * 使用归一化时间 s = (t - t0) / Δt ∈ [0, 1]，多项式形式
 *   p(s) = c0 + c1·s + c2·s² + c3·s³ + c4·s⁴ + c5·s⁵
 */
class QuinticSpline {
public:
    struct Node {
        scalar_t time;
        scalar_t position;
        scalar_t velocity;
        scalar_t acceleration;
    };

    QuinticSpline(Node start, Node end);

    scalar_t position(scalar_t time) const;
    scalar_t velocity(scalar_t time) const;
    scalar_t acceleration(scalar_t time) const;

    scalar_t startTimeDerivative(scalar_t t) const;
    scalar_t finalTimeDerivative(scalar_t t) const;

private:
    scalar_t normalizedTime(scalar_t t) const;

    scalar_t t0_, t1_, dt_, inv_dt_;

    // 多项式系数 c0..c5
    scalar_t c0_, c1_, c2_, c3_, c4_, c5_;

    // dc_i / d(Δt)，在构造时预计算（因为 Δt 固定），dc0 恒为 0 故省略
    scalar_t dc1_, dc2_, dc3_, dc4_, dc5_;
};

} // namespace ocs2::legged_robot
