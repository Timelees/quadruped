#include "ocs2_quadruped_controller/interface/constraint/QuinticSpline.h"

#include <cassert>

namespace ocs2::legged_robot {

/**
 * 五次 Hermite 插值。
 * 归一化时间 s = (t - t0) / Δt, Δt = t1 - t0.
 *
 * 边界条件 → 6 个方程确定 6 个系数 c0..c5:
 *   p(0)   = p0,   p'(0)/Δt = v0,   p''(0)/Δt² = a0
 *   p(1)   = p1,   p'(1)/Δt = v1,   p''(1)/Δt² = a1
 *
 * 令 h0 = v0·Δt,  h1 = v1·Δt,  g0 = a0·Δt²,  g1 = a1·Δt²
 *   Δp = p1 - p0
 *   A  = Δp - h0 - g0/2
 *   B  = h1 - h0 - g0
 *   C  = g1 - g0
 *
 * 则 c0..c5 的封闭解：
 *   c0 = p0
 *   c1 = h0
 *   c2 = g0 / 2
 *   c3 = 10A - 4B + C/2
 *   c4 = -15A + 7B - C
 *   c5 = 6A - 3B + C/2
 */
QuinticSpline::QuinticSpline(Node start, Node end) {
    assert(start.time < end.time);

    t0_ = start.time;
    t1_ = end.time;
    dt_ = t1_ - t0_;
    inv_dt_ = 1.0 / dt_;

    const scalar_t dp = end.position - start.position;
    const scalar_t v0 = start.velocity;
    const scalar_t v1 = end.velocity;
    const scalar_t a0 = start.acceleration;
    const scalar_t a1 = end.acceleration;

    const scalar_t h0 = v0 * dt_;
    const scalar_t h1 = v1 * dt_;
    const scalar_t g0 = a0 * dt_ * dt_;
    const scalar_t g1 = a1 * dt_ * dt_;

    const scalar_t A = dp - h0 - 0.5 * g0;
    const scalar_t B = h1 - h0 - g0;
    const scalar_t C = g1 - g0;

    c0_ = start.position;
    c1_ = h0;
    c2_ = 0.5 * g0;
    c3_ = 10.0 * A - 4.0 * B + 0.5 * C;
    c4_ = -15.0 * A + 7.0 * B - C;
    c5_ = 6.0 * A - 3.0 * B + 0.5 * C;

    // ---- dc_i / d(Δt)，dc0 恒为 0 故省略 ----
    dc1_ = v0;
    dc2_ = a0 * dt_;
    dc3_ = (-6.0 * v0 - 4.0 * v1) + (a1 - 3.0 * a0) * dt_;
    dc4_ = (8.0 * v0 + 7.0 * v1) + (3.0 * a0 - 2.0 * a1) * dt_;
    dc5_ = (-3.0 * v0 - 3.0 * v1) + (a1 - a0) * dt_;
}

scalar_t QuinticSpline::normalizedTime(scalar_t t) const {
    return (t - t0_) * inv_dt_;
}

scalar_t QuinticSpline::position(scalar_t time) const {
    const scalar_t s = normalizedTime(time);
    return c0_ + s * (c1_ + s * (c2_ + s * (c3_ + s * (c4_ + s * c5_))));
}

scalar_t QuinticSpline::velocity(scalar_t time) const {
    const scalar_t s = normalizedTime(time);
    return (c1_ + s * (2.0 * c2_ + s * (3.0 * c3_ + s * (4.0 * c4_ + s * 5.0 * c5_)))) * inv_dt_;
}

scalar_t QuinticSpline::acceleration(scalar_t time) const {
    const scalar_t s = normalizedTime(time);
    return (2.0 * c2_ + s * (6.0 * c3_ + s * (12.0 * c4_ + s * 20.0 * c5_))) * (inv_dt_ * inv_dt_);
}

scalar_t QuinticSpline::startTimeDerivative(scalar_t t) const {
    const scalar_t s = normalizedTime(t);
    // dp/ds — 内联求值，避免重复调用 velocity()
    const scalar_t dp_ds = c1_ + s * (2.0 * c2_ + s * (3.0 * c3_ + s * (4.0 * c4_ + s * 5.0 * c5_)));
    // Σ dc_i · s^i（dc0=0 已省略）
    const scalar_t dCoff = s * (dc1_ + s * (dc2_ + s * (dc3_ + s * (dc4_ + s * dc5_))));
    // ds/dt0 = -(1-s)/Δt
    return -dp_ds * (1.0 - s) * inv_dt_ - dCoff;
}

scalar_t QuinticSpline::finalTimeDerivative(scalar_t t) const {
    const scalar_t s = normalizedTime(t);
    const scalar_t dp_ds = c1_ + s * (2.0 * c2_ + s * (3.0 * c3_ + s * (4.0 * c4_ + s * 5.0 * c5_)));
    const scalar_t dCoff = s * (dc1_ + s * (dc2_ + s * (dc3_ + s * (dc4_ + s * dc5_))));
    // ds/dt1 = -s/Δt
    return -dp_ds * s * inv_dt_ + dCoff;
}

} // namespace ocs2::legged_robot
