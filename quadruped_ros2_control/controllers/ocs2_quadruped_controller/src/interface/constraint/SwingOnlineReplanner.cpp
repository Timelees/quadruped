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
                           const Eigen::Matrix<double, 6, 3>& correction, double delta,
                           const TerrainSafetyCorridor& corridor, double swingStart) {
    const auto oldControls = old.getControlPoints();
    const double rejoin = activation + delta;
    const auto& oldKnots = old.getKnots();
    bool alreadyPiecewise=false;
    for (int i=6;i<oldControls.rows();++i)
        alreadyPiecewise=alreadyPiecewise || std::abs(oldKnots[i]-oldKnots[i-1])<1e-10;
    std::vector<PolynomialSegment> pieces;
    for (auto segment : extractPolynomialSegments(old)) {
        if (segment.end <= activation) continue;
        segment = restrictSegment(segment, std::max(activation, segment.start), std::min(end, segment.end));
        std::vector<double> breaks{segment.start,segment.end};
        if (rejoin>segment.start && rejoin<segment.end) breaks.push_back(rejoin);
        // Exact subdivision adds local adjustment freedom at terrain events.
        // It does not alter the executing reference or relax any certificate.
        // guardrail: enrich once; repeated online updates retain this knot
        // space rather than multiplying the decision size at every map update.
        if (!alreadyPiecewise) {
            for (const auto& cell:corridor.cells) {
                for (double phase:{cell.start,cell.end}) {
                    const double time=swingStart+(end-swingStart)*phase;
                    if (time>segment.start && time<segment.end) breaks.push_back(time);
                }
            }
        }
        std::sort(breaks.begin(),breaks.end());
        breaks.erase(std::unique(breaks.begin(),breaks.end(),[](double a,double b) {return std::abs(a-b)<1e-10;}),breaks.end());
        for (size_t i=1;i<breaks.size();++i)
            pieces.push_back(restrictSegment(segment,breaks[i-1],breaks[i]));
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
                     const SwingReplanConfig& config, std::string& failure,
                     const std::function<Eigen::Vector2d(double,const Eigen::Vector3d&)>& workspace) {
    failure = "candidate QP construction failed";
    const auto controls = candidate.getControlPoints();
    const int n = controls.rows();
    const int freeVariables = n-6;
    const auto segments = extractPolynomialSegments(candidate);
    // Express the old reference in exactly the candidate's piecewise knot space.
    const auto oldSegments = extractPolynomialSegments(old);
    Eigen::MatrixXd oldPoints=Eigen::MatrixXd::Constant(n,3,std::numeric_limits<double>::quiet_NaN());
    int index=0;
    for (const auto& segment:segments) {
        for (const auto& oldSegment:oldSegments) {
            if (oldSegment.start<=segment.start+1e-10 && oldSegment.end>=segment.end-1e-10) {
                oldPoints.middleRows(index,6)=restrictSegment(oldSegment,segment.start,segment.end).bezierMap*
                    old.getControlPoints();
                break;
            }
        }
        index+=5;
    }
    if (!oldPoints.allFinite()) return false;
    constexpr double solverTolerance=1e-9;
    std::vector<Eigen::RowVectorXd> rows;
    std::vector<double> lower, upper;
    bool fixedRowsFeasible = true;
    auto add = [&](Eigen::RowVectorXd row, double lo, double hi) {
        // Endpoint corrections are exactly zero. As in the terrain QP, solve
        // only for free controls and check every constant row before omitting it.
        Eigen::RowVectorXd freeRow = row.segment(3,freeVariables);
        if (freeRow.isZero(0.0)) {
            fixedRowsFeasible = fixedRowsFeasible && lo<=1e-6 && hi>=-1e-6;
            return;
        }
        rows.push_back(std::move(freeRow)); lower.push_back(lo); upper.push_back(hi);
    };
    const auto constraints = corridorControlPointConstraints(corridor);
    Eigen::VectorXd q(3*n); q << controls.col(0), controls.col(1), controls.col(2);
    const Eigen::VectorXd slack = constraints.b - constraints.A * q;
    for (int i = 0; i < constraints.A.rows(); ++i)
        add(constraints.A.row(i).tail(n), -qpOASES::INFTY, slack[i]);
    if (workspace) {
        // Use the same sampled IK sections as the planner's post-solve check.
        // XY is fixed, so each section is a linear bound on the Z correction.
        const Eigen::VectorXd times=Eigen::VectorXd::LinSpaced(21,segments.front().start,segments.back().end);
        const Eigen::MatrixXd basis=candidate.generateSplineBasis(times);
        for (int i=0;i<times.size();++i) {
            const auto point=candidate.getPosition(times[i]);
            const auto bounds=workspace(times[i],point);
            if (!bounds.allFinite() || bounds[0]>bounds[1]) {
                failure="candidate IK workspace bounds invalid";
                return false;
            }
            add(basis.row(i),bounds[0]-point.z(),bounds[1]-point.z());
        }
    }
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
                // Keep solver feasibility tolerance inside the physical bound.
                // Validation below still checks the original derivative limit.
                add(derivative.row(i)/scale, (-limit-value)/scale+solverTolerance,
                    (limit-value)/scale-solverTolerance);
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
    if (!fixedRowsFeasible) { failure="candidate fixed-state constraint violated"; return false; }
    if (freeVariables==0) return true;
    Matrix H = (config.nominalTrackingWeight + config.previousTrackingWeight) *
        Matrix::Identity(freeVariables,freeVariables);
    Matrix A(rows.size(),freeVariables);
    Vector g = Vector::Zero(freeVariables), lb(rows.size()), ub(rows.size());
    const Eigen::VectorXd oldZ=oldPoints.col(2);
    // In correction coordinates the nominal target is zero. The previous
    // target uses the same knot space, so each term has a distinct gradient.
    g = config.previousTrackingWeight * (controls.col(2)-oldZ).segment(3,freeVariables).cast<qpOASES::real_t>();
    // Track the previous motion as well as its control points. Independent
    // point costs alone allow short-span derivative spikes at the hard bounds,
    // so a geometrically valid candidate can diverge during policy transport.
    // Normalize by the existing physical limits and average over physical time
    // to keep this cost independent of corridor subdivision density.
    for (const auto& segment : segments) {
        Eigen::MatrixXd derivative=segment.bezierMap;
        for (int order=1;order<=3;++order) {
            derivative=((6-order)/(segment.end-segment.start)*
                (derivative.bottomRows(derivative.rows()-1)-derivative.topRows(derivative.rows()-1))).eval();
            const double limit=order==1 ? config.maxVelocity : order==2 ? config.maxAcceleration : config.maxJerk;
            const Matrix freeDerivative=(derivative.middleCols(3,freeVariables)/limit).cast<qpOASES::real_t>();
            const Vector residual=(derivative*(controls.col(2)-oldZ)/limit).cast<qpOASES::real_t>();
            const double weight=config.previousTrackingWeight*(segment.end-segment.start)/
                ((segments.back().end-segments.front().start)*derivative.rows());
            H.noalias()+=weight*freeDerivative.transpose()*freeDerivative;
            g.noalias()+=weight*freeDerivative.transpose()*residual;
        }
    }
    for (size_t i=0; i<rows.size(); ++i) { A.row(i)=rows[i].cast<qpOASES::real_t>(); lb[i]=lower[i]; ub[i]=upper[i]; }
    Vector variableLower = Vector::Constant(freeVariables,-config.maxTrajectoryCorrection);
    Vector variableUpper = Vector::Constant(freeVariables,config.maxTrajectoryCorrection);
    qpOASES::QProblem problem(freeVariables,rows.size());
    qpOASES::Options options; options.setToReliable(); options.printLevel=qpOASES::PL_NONE;
    // Corrections are centered at zero. Starting every variable at its lower
    // bound wastes the working-set budget releasing irrelevant -12 cm bounds.
    options.initialStatusBounds=qpOASES::ST_INACTIVE;
    options.boundTolerance=solverTolerance;
    options.terminationTolerance=solverTolerance;
    options.enableEqualities=qpOASES::BT_TRUE; problem.setOptions(options);
    qpOASES::int_t budget=200;
    const auto status=problem.init(H.data(),g.data(),A.data(),variableLower.data(),variableUpper.data(),lb.data(),ub.data(),budget);
    Vector solution=Vector::Zero(freeVariables);
    if (status!=qpOASES::SUCCESSFUL_RETURN) {
        failure = std::string("candidate QP failed: ") + qpOASES::MessageHandling::getErrorCodeMessage(status) +
            " (variables=" + std::to_string(freeVariables) + ", rows=" + std::to_string(rows.size()) +
            ", iterations=" + std::to_string(budget) + ")";
        return false;
    }
    failure = "candidate QP solution or constraint residual invalid";
    if (problem.getPrimalSolution(solution.data())!=qpOASES::SUCCESSFUL_RETURN ||
        !solution.allFinite() || (A*solution-ub).maxCoeff()>1e-6 || (lb-A*solution).maxCoeff()>1e-6) return false;
    Eigen::VectorXd correction=Eigen::VectorXd::Zero(n);
    correction.segment(3,freeVariables)=solution.cast<double>();
    candidate.setControlPoints(controls.col(0),controls.col(1),controls.col(2)+correction);
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
    const double nonnegative[] = {config_.activationDelay, config_.maxActivationDelay, config_.maxPositionDeviation,
        config_.maxVelocityDeviation, config_.maxTrajectoryCorrection, config_.positionUncertainty, config_.velocityUncertainty, config_.disturbanceBound,
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
    return validate(candidate,corridor,swingStart,swingEnd,nullptr);
}

bool SwingOnlineReplanner::validate(const BSpline& candidate, const TerrainSafetyCorridor& corridor,
                                   double swingStart, double swingEnd, std::string* failure) const {
    const auto reject = [&](std::string reason) {
        if (failure) *failure=std::move(reason);
        return false;
    };
    const auto mapped = remapCorridor(candidate, corridor, swingStart, swingEnd);
    if (!mapped.valid) return reject("candidate has no mapped corridor");
    const auto controls = candidate.getControlPoints();
    const auto constraints = corridorControlPointConstraints(mapped);
    Eigen::VectorXd q(controls.size());
    q << controls.col(0), controls.col(1), controls.col(2);
    if (!q.allFinite()) return reject("candidate controls are not finite");
    const double violation=(constraints.A*q-constraints.b).maxCoeff();
    if (violation>1e-8) return reject("candidate hull residual="+std::to_string(violation));
    for (const auto& segment : extractPolynomialSegments(candidate)) {
        Eigen::MatrixXd derivative = segment.bezierMap * controls;
        for (int order = 1; order <= 3; ++order) {
            derivative = ((6 - order) / (segment.end - segment.start) *
                (derivative.bottomRows(derivative.rows() - 1) - derivative.topRows(derivative.rows() - 1))).eval();
            const double limit = order == 1 ? config_.maxVelocity :
                                 order == 2 ? config_.maxAcceleration : config_.maxJerk;
            if (!derivative.allFinite()) return reject("candidate derivative is not finite");
            const double peak=derivative.cwiseAbs().maxCoeff();
            if (peak>limit+1e-7)
                return reject("candidate derivative order="+std::to_string(order)+" peak="+
                              std::to_string(peak)+" limit="+std::to_string(limit));
        }
    }
    return true;
}

SwingReplanDecision SwingOnlineReplanner::admit(const BSpline& old, const TerrainSafetyCorridor& corridor,
        double swingStart, double activation, double end, const Eigen::Vector3d& position,
        const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration) {
    return admit(old,corridor,swingStart,activation,end,position,velocity,acceleration,{});
}

SwingReplanDecision SwingOnlineReplanner::admit(const BSpline& old, const TerrainSafetyCorridor& corridor,
        double swingStart, double activation, double end, const Eigen::Vector3d& position,
        const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration,
        const std::function<Eigen::Vector2d(double,const Eigen::Vector3d&)>& workspace) {
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
    result.candidate = correctedReference(old, activation, end, correction, delta, corridor, swingStart);
    if (admissionCorridor.cells.empty())
        return rejected(SwingReplanOutcome::RejectedCorridor, "correction has no certified interval");
    admissionCorridor.cells.insert(admissionCorridor.cells.end(), corridor.cells.begin(), corridor.cells.end());
    const auto candidateCorridor = remapCorridor(result.candidate, admissionCorridor, swingStart, end);
    std::string refinementFailure;
    if (!refineCandidate(result.candidate, old, candidateCorridor, config_, refinementFailure, workspace))
        return rejected(SwingReplanOutcome::RejectedMargin, refinementFailure.c_str());
    std::string validationFailure;
    if (!validate(result.candidate, admissionCorridor, swingStart, end, &validationFailure))
        return rejected(SwingReplanOutcome::RejectedCandidate, validationFailure.c_str());
    if ((result.candidate.getPosition(end) - old.getPosition(end)).norm() > 1e-8 ||
        (result.candidate.getVelocity(end) - old.getVelocity(end)).norm() > 1e-7 ||
        (result.candidate.getAcceleration(end) - old.getAcceleration(end)).norm() > 1e-6)
        return rejected(SwingReplanOutcome::RejectedTerminal, "touchdown state changed");
    result.outcome = SwingReplanOutcome::Admitted;
    ++stats_.admitted;
    return result;
}
}  // namespace ocs2::legged_robot
