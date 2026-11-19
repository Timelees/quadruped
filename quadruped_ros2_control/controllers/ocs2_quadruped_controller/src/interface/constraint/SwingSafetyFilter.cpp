#include "ocs2_quadruped_controller/interface/constraint/SwingSafetyFilter.h"

#include "ocs2_quadruped_controller/interface/constraint/TerrainSafetyCorridor.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <qpOASES.hpp>

namespace ocs2::legged_robot {
namespace {
struct TubeRow { Eigen::Vector3d position, velocity; double bound; };
std::vector<TubeRow> tubeRows(const SafetyFilterConfig& config) {
    std::vector<TubeRow> rows;
    for (int axis = 0; axis < 3; ++axis) {
        const Eigen::Vector3d unit = Eigen::Vector3d::Unit(axis);
        for (double sign : {-1.0, 1.0}) {
            rows.push_back({sign * unit, Eigen::Vector3d::Zero(), config.positionBound});
            rows.push_back({Eigen::Vector3d::Zero(), sign * unit, config.velocityBound});
            // All four signs are needed for omega*|e_i| + |edot_i| <= B.
            for (double velocitySign : {-1.0, 1.0})
                rows.push_back({sign * config.couplingGain * unit, velocitySign * unit, config.couplingBound});
        }
    }
    return rows;
}
}  // namespace

SwingSafetyFilter::SwingSafetyFilter(SafetyFilterConfig config) : config_(config) {
    const double positive[] = {config.hocbfGain1, config.hocbfGain2, config.positionBound,
        config.velocityBound, config.couplingGain, config.couplingBound, config.tubeRate,
        config.maxCorrection, config.maxAcceleration, config.maxPeriod, config.landingPositionWindow,
        config.landingMaxDownSpeed, config.landingMaxUpSpeed};
    for (double value : positive)
        if (!std::isfinite(value) || value <= 0.0) throw std::invalid_argument("Invalid safety filter bound.");
    const double nonnegative[] = {config.disturbanceBound, config.hocbfSampleRate, config.tubeSampleRate};
    for (double value : nonnegative)
        if (!std::isfinite(value) || value < 0.0) throw std::invalid_argument("Invalid safety sampling budget.");
    // A per-axis box must fit inside the Euclidean corridor tracking ball.
    // This is checked against the actual corridor's shrink in filter().
}

SafetyFilterResult SwingSafetyFilter::filter(const TerrainSafetyCorridor& corridor, const FootState& state,
        const Eigen::Vector3d& nominal, double phase, double period, double timeToTouchdown) const {
    SafetyFilterResult result;
    if (!config_.enabled) return result;
    if (!std::isfinite(phase) || !state.position.allFinite() || !state.velocity.allFinite() ||
        !state.positionError.allFinite() || !state.velocityError.allFinite() ||
        !state.referenceAcceleration.allFinite() || !nominal.allFinite()) {
        result.status = SafetyFilterStatus::InvalidState; return result;
    }
    if (!corridor.valid || corridor.cells.empty()) {
        result.status = SafetyFilterStatus::NoCorridor; return result;
    }
    const bool landing = phase > corridor.end && phase < 1.0 && timeToTouchdown > 0.0;
    if (landing && corridor.approachCells.empty()) {
        result.status = SafetyFilterStatus::NoLandingCorridor; return result;
    }
    if (!landing && (phase < corridor.start - 1e-10 || phase > corridor.end + 1e-10)) {
        result.status = SafetyFilterStatus::NoActiveSwing; return result;
    }
    if (!std::isfinite(period) || period <= 0.0 || period > config_.maxPeriod + 1e-9) {
        result.status = SafetyFilterStatus::InvalidPeriod; return result;
    }
    double clearance = INFINITY, psi1 = INFINITY, tubeSlack = INFINITY;
    bool marginValid = true;
    std::vector<Eigen::Vector3d> rows;
    std::vector<double> bounds;
    constexpr double tolerance = 1e-8;
    for (const auto& cell : landing ? corridor.approachCells : corridor.cells) {
        if (phase < cell.start - 1e-10 || phase > cell.end + 1e-10) continue;
        for (const auto& plane : cell.halfSpaces) {
            const double supportE = config_.positionBound * plane.normal.lpNorm<1>();
            marginValid &= supportE <= plane.geometricOffset - plane.planningOffset + tolerance;
            const double h = plane.geometricOffset - plane.normal.dot(state.position);
            const double hdot = -plane.normal.dot(state.velocity);
            clearance = std::min(clearance, h);
            psi1 = std::min(psi1, hdot + config_.hocbfGain1 * h);
            rows.push_back(plane.normal);
            bounds.push_back((config_.hocbfGain1 + config_.hocbfGain2) * hdot +
                config_.hocbfGain1 * config_.hocbfGain2 * h -
                config_.disturbanceBound * plane.normal.norm() - config_.hocbfSampleRate * period);
        }
    }
    if (rows.empty()) { result.status = SafetyFilterStatus::NoActiveSwing; return result; }
    const int nHocbf = rows.size();
    const auto tube = tubeRows(config_);
    for (const auto& row : tube) {
        tubeSlack = std::min(tubeSlack, row.bound - row.position.dot(state.positionError) -
                                        row.velocity.dot(state.velocityError));
    }
    result.clearance = clearance;
    result.psi1 = psi1;
    result.tubeSlack = tubeSlack;
    if (clearance < -tolerance || psi1 < -tolerance) {
        result.status = SafetyFilterStatus::BarrierEntry; return result;
    }
    if (config_.tubeEnabled) {
        result.tubeStatus = !marginValid ? SafetyFilterStatus::MarginMismatch :
            (tubeSlack < -tolerance ? SafetyFilterStatus::TubeEntry : SafetyFilterStatus::Solved);
        if (result.tubeStatus != SafetyFilterStatus::Solved) {
            result.status = result.tubeStatus;
            return result;
        }
    }
    for (const auto& row : tube) {
        if (result.tubeStatus != SafetyFilterStatus::Solved) break;
        const double slack = row.bound - row.position.dot(state.positionError) -
                             row.velocity.dot(state.velocityError);
        // e'' = u - p_d'' + w. The reference acceleration MUST be subtracted.
        const double outwardRate = row.position.dot(state.velocityError);
        const double periodReserve = config_.tubeSampleRate * period;
        if (row.velocity.isZero()) {
            // The position face has no instantaneous acceleration authority.
            // Check its entry derivative, then enforce the full sample hold:
            // each of the quadratic error's three Bernstein coefficients stays
            // inside the face under the bounded acceleration residual.
            const double midpointSlack = slack - 0.5 * period * outwardRate;
            if (config_.tubeRate * slack - outwardRate < -tolerance || midpointSlack < -tolerance) {
                result.tubeStatus = SafetyFilterStatus::TubeRate;
                continue;
            }
            rows.push_back(row.position);
            bounds.push_back(2.0 * (slack - period*outwardRate) / (period*period) +
                row.position.dot(state.referenceAcceleration) - config_.disturbanceBound - periodReserve);
        } else {
            rows.push_back(row.velocity);
            bounds.push_back(config_.tubeRate * slack - outwardRate +
                row.velocity.dot(state.referenceAcceleration) - config_.disturbanceBound * row.velocity.norm() -
                periodReserve);
        }
    }
    if (config_.tubeEnabled && result.tubeStatus != SafetyFilterStatus::Solved) {
        result.status = result.tubeStatus;
        return result;
    }
    const int nSafety = rows.size();
    if (landing) {
        // Predict terminal error with constant error acceleration. The target
        // patch permits finite-time contact, while the other cells' HOCBF rows
        // above still enforce geometric clearance.
        const double t = timeToTouchdown;
        const double gain = 0.5 * t * t;
        const Eigen::Vector3d error = state.positionError + t * state.velocityError -
                                     gain * state.referenceAcceleration;
        for (int axis = 0; axis < 3; ++axis) {
            const double uncertainty = gain * config_.disturbanceBound;
            rows.push_back(Eigen::Vector3d::Unit(axis));
            bounds.push_back((config_.landingPositionWindow - error[axis] - uncertainty) / gain);
            rows.push_back(-Eigen::Vector3d::Unit(axis));
            bounds.push_back((config_.landingPositionWindow + error[axis] - uncertainty) / gain);
        }
        const double normalVelocity = corridor.touchdownVelocity.z() + state.velocityError.z() -
                                      t * state.referenceAcceleration.z();
        rows.push_back(Eigen::Vector3d::UnitZ());
        bounds.push_back((config_.landingMaxUpSpeed - normalVelocity) / t - config_.disturbanceBound);
        rows.push_back(-Eigen::Vector3d::UnitZ());
        bounds.push_back((config_.landingMaxDownSpeed + normalVelocity) / t - config_.disturbanceBound);
    }
    // Acceleration box Uhat and Delta-u box, used both here and in the final WBC.
    for (int axis = 0; axis < 3; ++axis) {
        rows.push_back(Eigen::Vector3d::Unit(axis));
        bounds.push_back(std::min(config_.maxAcceleration, nominal[axis] + config_.maxCorrection));
        rows.push_back(-Eigen::Vector3d::Unit(axis));
        bounds.push_back(std::min(config_.maxAcceleration, -nominal[axis] + config_.maxCorrection));
    }
    const auto solve = [&]() {
        const int nRows = rows.size();
        using Matrix = Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
        using Vector = Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, 1>;
        Matrix A(nRows, 3), H = Matrix::Identity(3, 3);
        Vector upper(nRows), g = Vector::Zero(3), solution = Vector::Zero(3);
        result.accelerationA.resize(nRows, 3);
        result.accelerationB.resize(nRows);
        for (int i = 0; i < nRows; ++i) {
            result.accelerationA.row(i) = rows[i].transpose();
            result.accelerationB[i] = bounds[i];
            A.row(i) = rows[i].cast<qpOASES::real_t>().transpose();
            upper[i] = bounds[i] - rows[i].dot(nominal);
        }
        result.activeRows = nRows;
        if (upper.minCoeff() < -1e-8) {
            qpOASES::QProblem problem(3, nRows);
            qpOASES::Options options;
            options.setToReliable(); options.printLevel = qpOASES::PL_NONE;
            problem.setOptions(options);
            qpOASES::int_t budget = 100;
            const auto status = problem.init(H.data(), g.data(), A.data(), nullptr, nullptr, nullptr, upper.data(), budget);
            result.iterations += budget;
            if (status != qpOASES::SUCCESSFUL_RETURN ||
                problem.getPrimalSolution(solution.data()) != qpOASES::SUCCESSFUL_RETURN || !solution.allFinite()) return false;
        }
        const Vector residual = upper - A * solution;
        if (residual.minCoeff() < -1e-6) return false;
        result.correction = solution.cast<double>();
        result.acceleration = nominal + result.correction;
        result.worstHocbfResidual = residual.head(nHocbf).minCoeff();
        if (config_.tubeEnabled) result.worstTubeResidual = residual.segment(nHocbf, nSafety - nHocbf).minCoeff();
        return true;
    };
    const bool solved = solve();
    if (!solved && config_.tubeEnabled) {
        result.tubeStatus = SafetyFilterStatus::Infeasible;
    }
    result.status = solved ? (landing ? SafetyFilterStatus::LandingSolved : SafetyFilterStatus::Solved) :
        SafetyFilterStatus::Infeasible;
    return result;
}
}  // namespace ocs2::legged_robot
