#include "ocs2_quadruped_controller/interface/constraint/SwingOnlineReplanner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <qpOASES.hpp>

namespace ocs2::legged_robot {
namespace {
PolynomialSegment restrictSegment(PolynomialSegment segment, double lo, double hi) {
    if (lo > segment.start) segment = segment.split(lo).second;
    if (hi < segment.end) segment = segment.split(hi).first;
    return segment;
}

// Eq. (37) exactly, in the union of old knots, activation, and correction end.
// Independent Bezier pieces retain the original physical domain. Their shared
// endpoint value/velocity/acceleration agree by construction (C2 after activation).
BSpline correctedReference(const BSpline& old, double activation, double end,
                           const Eigen::Matrix<double, 6, 3>& correction, double delta) {
    const auto oldControls = old.getControlPoints();
    const double rejoin = activation + delta;
    std::vector<PolynomialSegment> pieces;
    for (auto segment : extractPolynomialSegments(old)) {
        if (segment.end <= activation) continue;
        segment = restrictSegment(segment, std::max(activation, segment.start), std::min(end, segment.end));
        if (segment.start < rejoin && segment.end > rejoin) {
            auto halves = segment.split(rejoin);
            pieces.push_back(std::move(halves.first));
            pieces.push_back(std::move(halves.second));
        } else {
            pieces.push_back(std::move(segment));
        }
    }
    const int n = 5 * pieces.size() + 1;
    Eigen::MatrixXd controls(n, 3);
    Eigen::VectorXd knots(n + 6);
    knots.head<6>().setConstant(activation);
    for (size_t i = 0; i < pieces.size(); ++i) {
        const auto& piece = pieces[i];
        Eigen::Matrix<double, 6, 3> points = piece.bezierMap * oldControls;
        if (piece.start < rejoin) {
            PolynomialSegment d{activation, rejoin, Eigen::Matrix<double, 6, 6>::Identity()};
            d = restrictSegment(d, piece.start, piece.end);
            points += d.bezierMap * correction;
        }
        controls.middleRows(5 * i, 6) = points;
        if (i + 1 < pieces.size()) knots.segment(6 + 5 * i, 5).setConstant(piece.end);
    }
    knots.tail<6>().setConstant(end);
    BSpline result(5);
    result.setControlPoints(controls.col(0), controls.col(1), controls.col(2));
    result.setKnots(knots);
    return result;
}
// Z-only refinement in the EXACT candidate knot space. Fixed XYZ continuity
// and touchdown are preserved; every position/derivative row is checked again.
// The objective pulls the free controls toward the old reference, rather than
// resampling the candidate into a different spline space.
bool refineCandidate(BSpline& candidate, const BSpline& old, const TerrainSafetyCorridor& corridor,
                     const SwingReplanConfig& config) {
    const auto controls = candidate.getControlPoints();
    const int n = controls.rows();
    const auto segments = extractPolynomialSegments(candidate);
    std::vector<Eigen::RowVectorXd> rows;
    std::vector<double> lower, upper;
    auto add = [&](Eigen::RowVectorXd row, double lo, double hi) {
        rows.push_back(std::move(row)); lower.push_back(lo); upper.push_back(hi);
    };
    const auto constraints = corridorControlPointConstraints(corridor);
    Eigen::VectorXd q(3*n); q << controls.col(0), controls.col(1), controls.col(2);
    const Eigen::VectorXd slack = constraints.b - constraints.A * q;
    for (int i = 0; i < constraints.A.rows(); ++i)
        add(constraints.A.row(i).tail(n), -qpOASES::INFTY, slack[i]);
    Eigen::MatrixXd previousVelocity, previousAcceleration;
    for (const auto& segment : segments) {
        Eigen::MatrixXd derivative = segment.bezierMap;
        Eigen::MatrixXd velocity, acceleration;
        for (int order = 1; order <= 3; ++order) {
            derivative = ((6-order) / (segment.end-segment.start) *
                (derivative.bottomRows(derivative.rows()-1)-derivative.topRows(derivative.rows()-1))).eval();
            const double limit = order == 1 ? config.maxVelocity : order == 2 ? config.maxAcceleration : config.maxJerk;
            for (int i = 0; i < derivative.rows(); ++i) {
                const double value = derivative.row(i).dot(controls.col(2));
                // Scale rows before qpOASES: short spans make jerk coefficients large.
                const double scale = std::max(1.0, derivative.row(i).norm());
                add(derivative.row(i)/scale, (-limit-value)/scale, (limit-value)/scale);
            }
            if (order == 1) velocity = derivative;
            if (order == 2) acceleration = derivative;
        }
        if (previousVelocity.size()) {
            Eigen::RowVectorXd row = previousVelocity.bottomRows(1)-velocity.topRows(1);
            add(row / std::max(1.0, row.norm()), 0.0, 0.0);
            row = previousAcceleration.bottomRows(1)-acceleration.topRows(1);
            add(row / std::max(1.0, row.norm()), 0.0, 0.0);
        }
        previousVelocity = velocity; previousAcceleration = acceleration;
    }
    // Terminal inequalities in physical time. X/Y are fixed in this QP and
    // checked with the other endpoint invariants after solution.
    const double end = segments.back().end;
    const Eigen::RowVectorXd pEnd = segments.back().bezierMap.bottomRows(1);
    const Eigen::RowVectorXd vEnd = 5.0 / (end - segments.back().start) *
        (pEnd - segments.back().bezierMap.row(4));
    const double targetZ = old.getPosition(end).z();
    add(pEnd, targetZ - config.landingPositionWindow - pEnd.dot(controls.col(2)),
              targetZ + config.landingPositionWindow - pEnd.dot(controls.col(2)));
    add(vEnd, -config.landingMaxDownSpeed - vEnd.dot(controls.col(2)),
               config.landingMaxUpSpeed - vEnd.dot(controls.col(2)));
    using Matrix = Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    using Vector = Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, 1>;
    Matrix H = (config.nominalTrackingWeight + config.previousTrackingWeight) *
        Matrix::Identity(n,n);
    Matrix A(rows.size(),n);
    Vector g = Vector::Zero(n), lb(rows.size()), ub(rows.size());
    // Evaluate old polynomial controls on the same segments, exactly.
    const auto oldSegments = extractPolynomialSegments(old);
    Eigen::VectorXd oldZ = Eigen::VectorXd::Constant(n, std::numeric_limits<double>::quiet_NaN());
    int index = 0;
    for (const auto& segment : segments) {
        for (const auto& oldSegment : oldSegments) {
            if (oldSegment.start <= segment.start + 1e-10 && oldSegment.end >= segment.end - 1e-10) {
                const auto restricted = restrictSegment(oldSegment, segment.start, segment.end);
                oldZ.segment(index,6) = restricted.bezierMap * old.getControlPoints().col(2);
                break;
            }
        }
        index += 5;
    }
    if (!oldZ.allFinite()) return false;
    // In correction coordinates the nominal target is zero. The previous
    // target uses the same knot space, so each term has a distinct gradient.
    g = config.previousTrackingWeight * (controls.col(2)-oldZ).cast<qpOASES::real_t>();
    for (size_t i=0; i<rows.size(); ++i) { A.row(i)=rows[i].cast<qpOASES::real_t>(); lb[i]=lower[i]; ub[i]=upper[i]; }
    Vector variableLower = Vector::Constant(n,-config.maxPositionDeviation);
    Vector variableUpper = Vector::Constant(n,config.maxPositionDeviation);
    variableLower.head(3).setZero(); variableLower.tail(3).setZero();
    variableUpper.head(3).setZero(); variableUpper.tail(3).setZero();
    qpOASES::QProblem problem(n,rows.size());
    qpOASES::Options options; options.setToReliable(); options.printLevel=qpOASES::PL_NONE;
    options.enableEqualities=qpOASES::BT_TRUE; problem.setOptions(options);
    qpOASES::int_t budget=200;
    const auto status=problem.init(H.data(),g.data(),A.data(),variableLower.data(),variableUpper.data(),lb.data(),ub.data(),budget);
    Vector solution=Vector::Zero(n);
    if (status!=qpOASES::SUCCESSFUL_RETURN || problem.getPrimalSolution(solution.data())!=qpOASES::SUCCESSFUL_RETURN ||
        !solution.allFinite() || (A*solution-ub).maxCoeff()>1e-6 || (lb-A*solution).maxCoeff()>1e-6) return false;
    candidate.setControlPoints(controls.col(0),controls.col(1),controls.col(2)+solution.cast<double>());
    return true;
}
}  // namespace

std::string SwingReplanStats::json() const {
    std::ostringstream out;
    out << "{\"nAttempt\":" << attempts << ",\"nAdmit\":" << admitted
        << ",\"nReject\":" << attempts - admitted
        << ",\"noReference\":" << noReference
        << ",\"deviation\":" << rejectedDeviation << ",\"margin\":" << rejectedMargin
        << ",\"candidate\":" << rejectedCandidate << ",\"corridor\":" << rejectedCorridor
        << ",\"terminal\":" << rejectedTerminal << ",\"activation\":" << rejectedActivation
        << ",\"control\":" << rejectedControl << '}';
    return out.str();
}

SwingOnlineReplanner::SwingOnlineReplanner(SwingReplanConfig config) : config_(std::move(config)) {
    const double positive[] = {config_.correctionDuration, config_.minRemainingTime, config_.updateInterval,
                              config_.maxVelocity, config_.maxAcceleration, config_.maxJerk, config_.maxMarginFraction,
                              config_.landingPositionWindow, config_.landingMaxDownSpeed, config_.landingMaxUpSpeed};
    for (double value : positive) {
        if (!std::isfinite(value) || value <= 0.0) throw std::invalid_argument("Invalid swing replan bound.");
    }
    const double nonnegative[] = {config_.activationDelay, config_.maxPositionDeviation,
        config_.maxVelocityDeviation, config_.positionUncertainty, config_.velocityUncertainty, config_.disturbanceBound,
        config_.nominalTrackingWeight, config_.previousTrackingWeight};
    for (double value : nonnegative) {
        if (!std::isfinite(value) || value < 0.0) throw std::invalid_argument("Invalid swing prediction budget.");
    }
    if (config_.maxMarginFraction > 1.0 || config_.nominalTrackingWeight + config_.previousTrackingWeight <= 0.0)
        throw std::invalid_argument("Invalid replan margin or objective.");
}

void SwingOnlineReplanner::reject(SwingReplanOutcome outcome) {
    switch (outcome) {
        case SwingReplanOutcome::NoExecutingReference: ++stats_.noReference; break;
        case SwingReplanOutcome::RejectedDeviation: ++stats_.rejectedDeviation; break;
        case SwingReplanOutcome::RejectedMargin: ++stats_.rejectedMargin; break;
        case SwingReplanOutcome::RejectedCorridor: ++stats_.rejectedCorridor; break;
        case SwingReplanOutcome::RejectedTerminal: ++stats_.rejectedTerminal; break;
        case SwingReplanOutcome::RejectedActivation: ++stats_.rejectedActivation; break;
        case SwingReplanOutcome::RejectedControl: ++stats_.rejectedControl; break;
        default: ++stats_.rejectedCandidate; break;
    }
}

void SwingOnlineReplanner::correctionControlPoints(double delta, const Eigen::Vector3d& dp,
        const Eigen::Vector3d& dv, const Eigen::Vector3d& da, std::vector<Eigen::Vector3d>& controls) {
    controls = {dp, dp + delta / 5.0 * dv, dp + 2.0 * delta / 5.0 * dv + delta * delta / 20.0 * da,
                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
}

TerrainSafetyCorridor SwingOnlineReplanner::remapCorridor(const BSpline& reference,
        const TerrainSafetyCorridor& corridor, double swingStart, double swingEnd) {
    TerrainSafetyCorridor result = corridor;
    result.cells.clear();
    if (!corridor.valid || !(swingEnd > swingStart)) { result.valid = false; return result; }
    const auto segments = extractPolynomialSegments(reference);
    for (const auto& cell : corridor.cells) {
        const double start = swingStart + (swingEnd - swingStart) * cell.start;
        const double end = swingStart + (swingEnd - swingStart) * cell.end;
        for (const auto& segment : segments) {
            const double lo = std::max(start, segment.start), hi = std::min(end, segment.end);
            if (hi <= lo) continue;
            auto mapped = cell;
            mapped.start = (lo - swingStart) / (swingEnd - swingStart);
            mapped.end = (hi - swingStart) / (swingEnd - swingStart);
            mapped.hullMapping = hullMap(restrictSegment(segment, lo, hi), HullBasis::Minvo);
            result.cells.push_back(std::move(mapped));
        }
    }
    result.valid = !result.cells.empty();
    return result;
}

bool SwingOnlineReplanner::validate(const BSpline& candidate, const TerrainSafetyCorridor& corridor,
                                   double swingStart, double swingEnd) const {
    const auto mapped = remapCorridor(candidate, corridor, swingStart, swingEnd);
    if (!mapped.valid) return false;
    const auto controls = candidate.getControlPoints();
    const auto constraints = corridorControlPointConstraints(mapped);
    Eigen::VectorXd q(controls.size());
    q << controls.col(0), controls.col(1), controls.col(2);
    if (!q.allFinite() || (constraints.A * q - constraints.b).maxCoeff() > 1e-8) return false;
    for (const auto& segment : extractPolynomialSegments(candidate)) {
        Eigen::MatrixXd derivative = segment.bezierMap * controls;
        for (int order = 1; order <= 3; ++order) {
            derivative = ((6 - order) / (segment.end - segment.start) *
                (derivative.bottomRows(derivative.rows() - 1) - derivative.topRows(derivative.rows() - 1))).eval();
            const double limit = order == 1 ? config_.maxVelocity :
                                 order == 2 ? config_.maxAcceleration : config_.maxJerk;
            if (!derivative.allFinite() || derivative.cwiseAbs().maxCoeff() > limit + 1e-7) return false;
        }
    }
    return true;
}

SwingReplanDecision SwingOnlineReplanner::admit(const BSpline& old, const TerrainSafetyCorridor& corridor,
        double swingStart, double activation, double end, const Eigen::Vector3d& position,
        const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration) {
    SwingReplanDecision result;
    result.activation = activation;
    result.end = end;
    if (!config_.enabled) return result;
    ++stats_.attempts;
    auto rejected = [&](SwingReplanOutcome outcome, const char* reason) {
        result.outcome = outcome; result.reason = reason; reject(outcome); return result;
    };
    if (!std::isfinite(activation) || !std::isfinite(end) || end - activation < config_.minRemainingTime ||
        !position.allFinite() || !velocity.allFinite() || !acceleration.allFinite())
        return rejected(SwingReplanOutcome::RejectedActivation, "invalid activation state or insufficient time");
    const auto& knots = old.getKnots();
    if (knots.size() == 0 || activation < knots[0] || std::abs(knots[knots.size()-1] - end) > 1e-8)
        return rejected(SwingReplanOutcome::NoExecutingReference, "reference does not cover activation and touchdown");
    if (!corridor.valid || corridor.cells.empty())
        return rejected(SwingReplanOutcome::RejectedCorridor, "no current terrain corridor certificate");
    const double terminalSpeed = old.getVelocity(end).z();
    if (terminalSpeed < -config_.landingMaxDownSpeed || terminalSpeed > config_.landingMaxUpSpeed)
        return rejected(SwingReplanOutcome::RejectedTerminal, "fixed touchdown velocity outside terminal window");
    const Eigen::Vector3d dp = position - old.getPosition(activation);
    const Eigen::Vector3d dv = velocity - old.getVelocity(activation);
    const Eigen::Vector3d da = acceleration - old.getAcceleration(activation);
    if (dp.norm() > config_.maxPositionDeviation || dv.norm() > config_.maxVelocityDeviation)
        return rejected(SwingReplanOutcome::RejectedDeviation, "state outside correction gate");
    const double delta = std::min(config_.correctionDuration, end - activation);
    std::vector<Eigen::Vector3d> d;
    correctionControlPoints(delta, dp, dv, da, d);
    Eigen::Matrix<double, 6, 3> correction;
    for (int i = 0; i < 6; ++i) correction.row(i) = d[i].transpose();
    // Eq. (39): exact old hull bound, including the propagated uncertainty
    // support. This is a sufficient quick gate, followed by the full candidate check.
    const auto oldControls = old.getControlPoints();
    const auto oldSegments = extractPolynomialSegments(old);
    const double rejoin = activation + delta;
    result.availableMargin = std::numeric_limits<double>::infinity();
    auto admissionCorridor = corridor;
    admissionCorridor.cells.clear();
    for (const auto& cell : corridor.cells) {
        const double cellStart = swingStart + (end - swingStart) * cell.start;
        const double cellEnd = swingStart + (end - swingStart) * cell.end;
        for (const auto& segment : oldSegments) {
            const double lo = std::max({cellStart, segment.start, activation});
            const double hi = std::min({cellEnd, segment.end, rejoin});
            if (hi <= lo) continue;
            const auto oldPiece = restrictSegment(segment, lo, hi);
            const Eigen::MatrixXd hull = hullMap(oldPiece, HullBasis::Minvo) * oldControls;
            const PolynomialSegment correctionSegment{activation, rejoin, Eigen::Matrix<double,6,6>::Identity()};
            const Eigen::Matrix<double,6,3> localCorrection =
                restrictSegment(correctionSegment, lo, hi).bezierMap * correction;
            // Reserve the propagated uncertainty on the exact overlap, rather
            // than comparing separate maxima attained at different times.
            auto overlap = cell;
            overlap.start = (lo - swingStart) / (end - swingStart);
            overlap.end = (hi - swingStart) / (end - swingStart);
            const double elapsed = hi - activation;
            const double uncertainty = config_.positionUncertainty + elapsed * config_.velocityUncertainty +
                0.5 * config_.disturbanceBound * elapsed * elapsed;
            for (auto& plane : overlap.halfSpaces) {
                const double margin = plane.planningOffset - plane.basisMargin - corridor.numericalMargin -
                    plane.normal.z() * corridor.footRadius - (hull * plane.normal).maxCoeff();
                const double correctionSupport = (localCorrection * plane.normal).maxCoeff();
                result.correctionBound = std::max(result.correctionBound, correctionSupport);
                result.availableMargin = std::min(result.availableMargin, margin);
                // Tracking shrink already reserves space for execution error.
                // Reserve whichever support is larger, avoiding double counting
                // uncertainty already inside the geometry/plan offset gap.
                const double extra = std::max(0.0, plane.normal.norm()*uncertainty -
                                               (plane.geometricOffset - plane.planningOffset));
                plane.planningOffset -= std::max(0.0, margin) * (1.0 - config_.maxMarginFraction) + extra;
            }
            admissionCorridor.cells.push_back(std::move(overlap));
        }
    }
    result.candidate = correctedReference(old, activation, end, correction, delta);
    if (admissionCorridor.cells.empty())
        return rejected(SwingReplanOutcome::RejectedCorridor, "correction has no certified interval");
    admissionCorridor.cells.insert(admissionCorridor.cells.end(), corridor.cells.begin(), corridor.cells.end());
    const auto candidateCorridor = remapCorridor(result.candidate, admissionCorridor, swingStart, end);
    if (!refineCandidate(result.candidate, old, candidateCorridor, config_))
        return rejected(SwingReplanOutcome::RejectedMargin, "candidate QP infeasible within reserved corridor");
    if (!validate(result.candidate, admissionCorridor, swingStart, end))
        return rejected(SwingReplanOutcome::RejectedCandidate, "candidate hull or derivative bound failed");
    if ((result.candidate.getPosition(end) - old.getPosition(end)).norm() > 1e-8 ||
        (result.candidate.getVelocity(end) - old.getVelocity(end)).norm() > 1e-7 ||
        (result.candidate.getAcceleration(end) - old.getAcceleration(end)).norm() > 1e-6)
        return rejected(SwingReplanOutcome::RejectedTerminal, "touchdown state changed");
    result.outcome = SwingReplanOutcome::Admitted;
    ++stats_.admitted;
    return result;
}
}  // namespace ocs2::legged_robot
