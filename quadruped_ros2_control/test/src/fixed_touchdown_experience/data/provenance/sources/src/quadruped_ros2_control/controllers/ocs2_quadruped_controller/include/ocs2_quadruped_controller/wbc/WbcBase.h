//
// Created by qiayuan on 2022/7/1.
//
#ifndef WBCBASE_H
#define WBCBASE_H

#include "Task.h"
#include <array>
#include <functional>
#include "ocs2_quadruped_controller/interface/constraint/SwingSafetyFilter.h"

#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace ocs2::legged_robot
{
    // Decision Variables: x = [\dot u^T, F^T, \tau^T]^T
    class WbcBase
    {
        using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
        using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;

    public:
        virtual ~WbcBase() = default;

        WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                const PinocchioEndEffectorKinematics& eeKinematics);

        virtual void loadTasksSetting(const std::string& taskFile, bool verbose);

        virtual vector_t update(const vector_t& stateDesired, const vector_t& inputDesired,
                                const vector_t& rbdStateMeasured, size_t mode,
                                scalar_t period);

        // Columns are measured p/v and MPC p/v, in world coordinates, per leg.
        const Eigen::Matrix<scalar_t, 12, 4>& footTracking() const { return foot_tracking_; }
        Eigen::Matrix<scalar_t, 6, 4> measureFeet(const vector_t& rbdStateMeasured);
        Eigen::Matrix<scalar_t, 6, 1> comTracking();
        struct SwingReference {
            vector3_t position, velocity, acceleration;
            bool onlineUpdate = false;
        };
        using SwingReferenceCallback = std::function<bool(size_t, SwingReference&)>;
        void setSwingReferenceCallback(SwingReferenceCallback callback) { swing_reference_callback_ = std::move(callback); }
        bool solutionValid() const { return solution_valid_; }
        int solverStatus() const { return solver_status_; }
        int solverIterations() const { return solver_iterations_; }
        scalar_t constraintViolation() const { return constraint_violation_; }

        using SwingSafetyCallback = std::function<SafetyFilterResult(size_t, const vector3_t&, const vector3_t&, const vector3_t&)>;
        void setSwingSafetyCallback(SwingSafetyCallback callback) { swing_safety_callback_ = std::move(callback); }
        // Supplied only when the hardware exports its actual current-loop state.
        void setActuatorState(const vector_t& current, const vector_t& applied, double kp, double kd) {
            motor_current_ = current; motor_applied_ = applied; joint_kp_ = kp; joint_kd_ = kd;
        }
        const auto& actuatorPrediction() const { return actuator_prediction_; }
        bool actuatorModelValid() const { return actuator_model_valid_; }
        void copyControlHistoryFrom(const WbcBase& other) {
            input_last_ = other.input_last_;
            base_task_initialized_ = other.base_task_initialized_;
            motor_current_ = other.motor_current_; motor_applied_ = other.motor_applied_;
            joint_kp_ = other.joint_kp_; joint_kd_ = other.joint_kd_;
        }
        void resetControlHistory() { base_task_initialized_ = false; }
        const auto& safetyResults() const { return safety_results_; }
        const auto& safetyResiduals() const { return safety_residuals_; }
        const auto& commandedFootAcceleration() const { return commanded_foot_acceleration_; }
        const auto& referenceFootAcceleration() const { return reference_foot_acceleration_; }
        const auto& nominalFootAcceleration() const { return nominal_foot_acceleration_; }
        const auto& baselineFootAcceleration() const { return baseline_foot_acceleration_; }

    protected:
        void updateMeasured(const vector_t& rbdStateMeasured);

        void updateDesired(const vector_t& stateDesired, const vector_t& inputDesired);

        size_t getNumDecisionVars() const { return num_decision_vars_; }

        Task formulateFloatingBaseEomTask();        // 浮基动力学约束任务

        Task formulateTorqueLimitsTask();           // 扭矩限制任务

        Task formulateNoContactMotionTask();        // 无接触运动任务（足端加速度为零）

        Task formulateFrictionConeTask();           // 摩擦锥约束任务

        Task formulateBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period);   // 基座加速度跟踪任务

        Task formulateSwingLegTask(const Eigen::Matrix<scalar_t,3,4>* feasibleAcceleration = nullptr);       // 摆动腿任务（足端位置和速度跟踪）

        Task formulateContactForceTask(const vector_t& inputDesired) const;     // 接触力跟踪任务

        // Task formulateFootPlacementCbfTask();   // 足端位置安全约束任务（CBF）

        size_t num_decision_vars_;      // 决策变量数 [广义加速度（6 基座 + 12 关节 = 18）+足端力（4 足 × 3D = 12）+关节力矩（12）]    
        PinocchioInterface pinocchio_interface_measured_, pinocchio_interface_desired_;
        CentroidalModelInfo info_;

        std::unique_ptr<PinocchioEndEffectorKinematics> ee_kinematics_;
        CentroidalModelPinocchioMapping mapping_;

        vector_t q_measured_, v_measured_, input_last_;
        matrix_t j_, dj_;
        contact_flag_t contact_flag_{};
        size_t num_contacts_{};
        Eigen::Matrix<scalar_t, 12, 4> foot_tracking_ = Eigen::Matrix<scalar_t, 12, 4>::Zero();
        SwingSafetyCallback swing_safety_callback_;
        SwingReferenceCallback swing_reference_callback_;
        Task swing_safety_constraints_;
        std::array<SafetyFilterResult, 4> safety_results_;
        std::array<double, 4> safety_residuals_;
        vector_t nominal_swing_target_;
        Eigen::Matrix<scalar_t,3,4> commanded_foot_acceleration_, reference_foot_acceleration_, nominal_foot_acceleration_,
            baseline_foot_acceleration_;
        bool solution_valid_ = true;
        int solver_status_ = 0, solver_iterations_ = 0;
        scalar_t constraint_violation_ = 0.0;

        vector_t motor_current_, motor_applied_, actuator_prediction_;
        double joint_kp_ = 0.0, joint_kd_ = 0.0;
        bool actuator_model_valid_ = false;
        bool actuatorSafetyModel_ = true;
        bool base_task_initialized_ = false;

        // Task Parameters:
        vector_t torque_limits_;
        scalar_t friction_coeff_{}, swing_kp_{}, swing_kd_{};
    };
} // namespace legged
#endif // WBCBASE_H
