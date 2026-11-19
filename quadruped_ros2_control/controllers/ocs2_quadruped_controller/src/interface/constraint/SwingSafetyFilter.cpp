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
struct HoldPiece { double offset, duration; Eigen::Matrix<double,6,3> position,velocity,acceleration; };
Eigen::MatrixXd elevate(Eigen::MatrixXd controls) {
    while (controls.rows()<6) {
        const int degree=controls.rows()-1;
        Eigen::MatrixXd next(degree+2,3);
        next.row(0)=controls.row(0); next.bottomRows(1)=controls.bottomRows(1);
        for (int i=1;i<=degree;++i) {
            const double alpha=double(i)/(degree+1);
            next.row(i)=alpha*controls.row(i-1)+(1-alpha)*controls.row(i);
        }
        controls=std::move(next);
    }
    return controls;
}
std::vector<HoldPiece> referenceHold(const SwingSafetyFilter::FootState& state,double period) {
    std::vector<HoldPiece> result;
    if (state.reference) {
        const auto& reference=*state.reference;
        const Eigen::Vector3d offset=state.position-state.positionError-reference.getPosition(state.referenceTime);
        for (auto piece:extractPolynomialSegments(reference)) {
            const double lo=std::max(piece.start,state.referenceTime),hi=std::min(piece.end,state.referenceTime+period);
            if (hi<=lo) continue;
            if (lo>piece.start) piece=piece.split(lo).second;
            if (hi<piece.end) piece=piece.split(hi).first;
            const Eigen::MatrixXd p=piece.bezierMap*reference.getControlPoints();
            const Eigen::MatrixXd v=5.0/(hi-lo)*(p.bottomRows(5)-p.topRows(5));
            const Eigen::MatrixXd a=4.0/(hi-lo)*(v.bottomRows(4)-v.topRows(4));
            result.push_back({lo-state.referenceTime,hi-lo,
                p.rowwise()+offset.transpose(),elevate(v),elevate(a)});
        }
    } else {
        HoldPiece piece; piece.offset=0.0;piece.duration=period;
        const Eigen::Vector3d p=state.position-state.positionError,v=state.velocity-state.velocityError;
        for (int i=0;i<6;++i) {
            const double t=period*i/5.0,tt=period*period*i*(i-1)/20.0;
            piece.position.row(i)=(p+t*v+0.5*tt*state.referenceAcceleration).transpose();
            piece.velocity.row(i)=(v+t*state.referenceAcceleration).transpose();
            piece.acceleration.row(i)=state.referenceAcceleration.transpose();
        }
        result.push_back(std::move(piece));
    }
    return result;
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
    const double nextPhase=timeToTouchdown>0.0 ? phase+period*(1.0-phase)/timeToTouchdown : phase;
    for (const auto& cell : landing ? corridor.approachCells : corridor.cells) {
        // Include an upcoming cell if this hold crosses its boundary. Requiring
        // both entry sets now is conservative and avoids an unchecked switch.
        if (nextPhase < cell.start - 1e-10 || phase > cell.end + 1e-10) continue;
        for (const auto& plane : cell.halfSpaces) {
            const double supportE = config_.positionBound * plane.normal.lpNorm<1>();
            marginValid &= supportE <= plane.geometricOffset - plane.planningOffset + tolerance;
            const double h = plane.geometricOffset - plane.normal.dot(state.position);
            const double hdot = -plane.normal.dot(state.velocity);
            clearance = std::min(clearance, h);
            psi1 = std::min(psi1, hdot + config_.hocbfGain1 * h);
            // The robust HOCBF residual over a held acceleration is quadratic.
            // Its three Bernstein coefficients lower-bound the entire hold,
            // including a bounded, time-varying acceleration disturbance.
            const double sum=config_.hocbfGain1+config_.hocbfGain2;
            const double product=config_.hocbfGain1*config_.hocbfGain2;
            for (int i=0;i<3;++i) {
                const double t=period*i/2.0,tt=i==2 ? period*period : 0.0;
                const double gain=1.0+sum*t+0.5*product*tt;
                rows.push_back(gain*plane.normal);
                bounds.push_back((sum+product*t)*hdot+product*h-
                    gain*config_.disturbanceBound*plane.normal.norm()-config_.hocbfSampleRate*period);
            }
        }
    }
    if (rows.empty()) { result.status = SafetyFilterStatus::NoActiveSwing; return result; }
    int nHocbf = rows.size();
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
    if (config_.tubeEnabled) {
        const auto hold=referenceHold(state,period);
        double covered=0.0;
        for (const auto& piece:hold) covered+=piece.duration;
        if (covered<period-1e-9) { result.status=SafetyFilterStatus::InvalidState; return result; }
        for (const auto& row:tube) {
            const Eigen::Vector3d velocityWeight=config_.tubeRate*row.velocity+row.position;
            for (const auto& piece:hold) {
                const double end=piece.offset+piece.duration;
                const double uncertainty=config_.disturbanceBound*(row.velocity.norm()+
                    0.5*config_.tubeRate*row.position.norm()*end*end+velocityWeight.norm()*end);
                for (int i=0;i<6;++i) {
                    const double t=piece.offset+piece.duration*i/5.0;
                    const double tt=piece.offset*piece.offset+2*piece.offset*piece.duration*i/5.0+
                        piece.duration*piece.duration*i*(i-1)/20.0;
                    const Eigen::Vector3d freePosition=state.position+t*state.velocity-piece.position.row(i).transpose();
                    const Eigen::Vector3d freeVelocity=state.velocity-piece.velocity.row(i).transpose();
                    const Eigen::Vector3d coefficients=0.5*config_.tubeRate*tt*row.position+t*velocityWeight+row.velocity;
                    const double bound=config_.tubeRate*row.bound-config_.tubeRate*row.position.dot(freePosition)-
                        velocityWeight.dot(freeVelocity)+row.velocity.dot(piece.acceleration.row(i).transpose())-
                        uncertainty-config_.tubeSampleRate*period;
                    if (coefficients.norm()<1e-12) {
                        if (bound<-tolerance) { result.tubeStatus=SafetyFilterStatus::TubeRate; }
                    } else {
                        rows.push_back(coefficients); bounds.push_back(bound);
                    }
                }
            }
        }
    }
    if (config_.tubeEnabled && result.tubeStatus != SafetyFilterStatus::Solved) {
        result.status = result.tubeStatus;
        return result;
    }
    int nSafety = rows.size();
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
    // Tube Bernstein coefficients are axis-aligned; merge co-directed rows
    // within each diagnostic group to avoid inflating the final whole-body QP.
    std::vector<Eigen::Vector3d> compactRows;
    std::vector<double> compactBounds;
    const auto compact=[&](int begin,int end) {
        const int first=compactRows.size();
        for (int i=begin;i<end;++i) {
            const double norm=rows[i].norm();
            const Eigen::Vector3d normal=rows[i]/norm;
            bool merged=false;
            for (size_t j=first;j<compactRows.size();++j) {
                if ((compactRows[j]-normal).norm()<1e-10) {
                    compactBounds[j]=std::min(compactBounds[j],bounds[i]/norm); merged=true;break;
                }
            }
            if (!merged) {compactRows.push_back(normal);compactBounds.push_back(bounds[i]/norm);}
        }
    };
    const int oldHocbf=nHocbf,oldSafety=nSafety;
    compact(0,oldHocbf); nHocbf=compactRows.size();
    compact(oldHocbf,oldSafety); nSafety=compactRows.size();
    compact(oldSafety,rows.size()); rows=std::move(compactRows);bounds=std::move(compactBounds);
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
