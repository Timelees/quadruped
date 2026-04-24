//
// Created by qiayuan on 2022/7/1.
//
#ifndef WBCBASE_H
#define WBCBASE_H

#include "Task.h"

#include <memory>

#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace ocs2::legged_robot
{
    class SwitchedModelReferenceManager;
}

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
            const PinocchioEndEffectorKinematics& eeKinematics,
            std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr = nullptr);

        virtual void loadTasksSetting(const std::string& taskFile, bool verbose);

        virtual vector_t update(const vector_t& stateDesired, const vector_t& inputDesired,
                                const vector_t& rbdStateMeasured, size_t mode,
                                scalar_t period, scalar_t currentTime);

    protected:
        void updateMeasured(const vector_t& rbdStateMeasured);

        void updateDesired(const vector_t& stateDesired, const vector_t& inputDesired);

        size_t getNumDecisionVars() const { return num_decision_vars_; }

        Task formulateFloatingBaseEomTask();        // 浮基动力学约束任务

        Task formulateTorqueLimitsTask();           // 扭矩限制任务

        Task formulateNoContactMotionTask();        // 无接触运动任务（足端加速度为零）

        Task formulateFrictionConeTask();           // 摩擦锥约束任务

        Task formulateBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period);   // 基座加速度跟踪任务

        Task formulateSwingLegTask();       // 摆动腿任务（足端位置和速度跟踪）

        Task formulateContactForceTask(const vector_t& inputDesired) const;     // 接触力跟踪任务

        Task formulateFootPlacementCbfTask();   // 足端位置安全约束任务（CBF）

        size_t num_decision_vars_;      // 决策变量数 [广义加速度（6 基座 + 12 关节 = 18）+足端力（4 足 × 3D = 12）+关节力矩（12）]    
        PinocchioInterface pinocchio_interface_measured_, pinocchio_interface_desired_;
        CentroidalModelInfo info_;

        std::unique_ptr<PinocchioEndEffectorKinematics> ee_kinematics_;
        CentroidalModelPinocchioMapping mapping_;
        std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr_;

        vector_t q_measured_, v_measured_, input_last_;
        matrix_t j_, dj_;   // 足端雅可比矩阵（3 × 自由度）,足端雅可比时间导数
        contact_flag_t contact_flag_{};
        size_t num_contacts_{};
        scalar_t currentTime_ = 0.0;

        // Task Parameters:
        vector_t torque_limits_;
        scalar_t friction_coeff_{}, swing_kp_{}, swing_kd_{};
        scalar_t foot_cbf_gamma_ = 1.0;
        scalar_t foot_cbf_xi_ = 5.0;
        scalar_t foot_cbf_relaxation_derivative_dt_ = 0.02;
        scalar_t foot_cbf_max_relaxation_rate_ = 0.5;
        scalar_t foot_cbf_max_relaxation_accel_ = 2.0;
    };
} // namespace legged
#endif // WBCBASE_H
