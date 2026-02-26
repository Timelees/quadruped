//
// Created based on cyber_guide ContactEstimator
// Kalman filter based contact estimation for quadruped robots
//

#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

#include <controller_common/CtrlInterfaces.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "log_save.h"

#include <array>
#include <memory>
#include <Eigen/Dense>

namespace ocs2::legged_robot
{

    // 动力学参数结构体
    struct DynamicParameters
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        matrix_t M;        // 广义坐标质量矩阵 (18×18)
        matrix_t C;        // 科里奥利矩阵 (18×18)
        vector_t grav;     // 重力项 G(q) (18×1)
        vector_t nle;      // 非线性效应 C(q,v)*v + G(q) (18×1)
    };

    // 单腿参数结构体
    struct LegParameters
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Matrix<double, 3, 18> Jc;     // 足端接触雅可比 (3×nv)，线性部分
        Eigen::Vector3d fi;                  // 足端力 (3×1)
        
        LegParameters() : Jc(Eigen::Matrix<double, 3, 18>::Zero()), 
                          fi(Eigen::Vector3d::Zero()) {}
    };

    constexpr int NUM_LEGS = 4;

    /**
     * @brief 基于卡尔曼滤波的接触估计器
     *
     * 融合三个观测量进行接触状态估计:
     * 1. 步态相位概率 (Pcontact) - 来自规划的步态时序
     * 2. 足端高度概率 (Pposition) - 基于足端z坐标
     * 3. 足端力概率 (Pforce) - 基于足端力传感器
     *
     * 使用卡尔曼滤波融合这些概率，输出最终的接触概率估计
     */

    class ContactEstimator
    {
    public:
        using scalar_t = double;
        using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
        using matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>;
        using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
        using Vector4 = Eigen::Matrix<scalar_t, NUM_LEGS, 1>;
        using Vector8 = Eigen::Matrix<scalar_t, NUM_LEGS * 2, 1>;
        using Matrix4 = Eigen::Matrix<scalar_t, NUM_LEGS, NUM_LEGS>;
        using Matrix8x4 = Eigen::Matrix<scalar_t, NUM_LEGS * 2, NUM_LEGS>;

        ContactEstimator(PinocchioInterface pinocchio_interface, CentroidalModelInfo info,
                         const PinocchioEndEffectorKinematics &ee_kinematics,
                         CtrlInterfaces &ctrl_component,
                         const rclcpp_lifecycle::LifecycleNode::SharedPtr &node);

        /**
         * @brief 初始化估计器参数
         */
        void setup();

        /**
         * @brief 运行卡尔曼滤波接触估计
         * @param foot_forces 四条腿的足端力 [FL, FR, RL, RR] (N)
         * @param foot_heights 四条腿的足端高度 [FL, FR, RL, RR] (m)
         * @param contact_phase 规划的接触相位 [0,1], 0=摆动开始, 1=支撑结束
         * @param swing_phase 规划的摆动相位 [0,1], 0=摆动开始, 1=摆动结束
         */
        void update(const std::array<scalar_t, NUM_LEGS> &foot_forces, const std::array<scalar_t, NUM_LEGS> &foot_heights,
                    const std::array<scalar_t, NUM_LEGS> &contact_phase, const std::array<scalar_t, NUM_LEGS> &swing_phase);

        // 获取接触标志  四条腿的接触标志 (true=接触, false=摆动)
        [[nodiscard]] contact_flag_t getContactFlags() const { return contact_flags_; }

        // 获取接触概率  四条腿的接触概率 [0,1]
        
        [[nodiscard]] std::array<scalar_t, NUM_LEGS> getContactProbabilities() const { return contact_prob_; }

        /**
         * @brief 获取接触相位（用于状态估计噪声调整）
         * @return 四条腿的接触相位 [0,1], 0=摆动/低置信度, 1=稳定支撑/高置信度
         */
        [[nodiscard]] std::array<scalar_t, NUM_LEGS> getContactPhase() const { return contact_phase_out_; }

       
        // 获取估计的足端力 (z方向分量)
       
        [[nodiscard]] std::array<scalar_t, NUM_LEGS> getFootForces() const { return foot_forces_; }

     
        //    获取估计的足端力向量
    
        [[nodiscard]] std::array<Eigen::Vector3d, NUM_LEGS> getFootForceVectors() const {
            std::array<Eigen::Vector3d, NUM_LEGS> forces;
            for (int i = 0; i < NUM_LEGS; ++i) {
                forces[i] = leg_parameters_[i].fi;
            }
            return forces;
        }

     
        //  设置关节力矩 (用于足端力估计)

        void setJointTorques(const Eigen::Matrix<double, 12, 1>& joint_torques) { tauAct_ = joint_torques; }

        // 设置机器人状态 (用于动力学计算)
         
        void setRbdState(const vector_t& rbd_state) { rbd_state_ = rbd_state; }

        // 更新动力学参数并计算足端力
        void updateAndComputeForce(scalar_t dt) {
            updateDynamicsParameters();
            computeFootForce(dt);
        }

        // 调试信息
        struct DebugInfo
        {
            std::array<scalar_t, NUM_LEGS> P_gait;     // 步态相位概率
            std::array<scalar_t, NUM_LEGS> P_position; // 足端高度概率
            std::array<scalar_t, NUM_LEGS> P_force;    // 足端力概率
            std::array<scalar_t, NUM_LEGS> X_est;      // 卡尔曼估计值
        };
        [[nodiscard]] DebugInfo getDebugInfo() const { return debug_info_; }

        // 参数设置
        void setContactThreshold(scalar_t threshold) { contact_threshold_ = threshold; }
        void setForceThreshold(scalar_t threshold) { force_threshold_ = threshold; }
        void setHeightThreshold(scalar_t threshold) { height_threshold_ = threshold; }

        /**
         * @brief 设置 MPC 期望的接触状态（用于对比分析）
         * @param expected_contact MPC 规划的期望接触标志 (true=期望接触, false=期望摆动)
         */
        void setExpectedContact(const contact_flag_t& expected_contact) { 
            expected_contact_ = expected_contact; 
        }

        /**
         * @brief 获取期望接触状态
         */
        [[nodiscard]] contact_flag_t getExpectedContact() const { return expected_contact_; }

        /**
         * @brief 获取接触状态误差（估计值 vs 期望值）
         * @return 四条腿的接触状态是否匹配 (true=匹配, false=不匹配)
         */
        [[nodiscard]] std::array<bool, NUM_LEGS> getContactMatchStatus() const {
            std::array<bool, NUM_LEGS> match_status;
            for (int i = 0; i < NUM_LEGS; ++i) {
                match_status[i] = (contact_flags_[i] == expected_contact_[i]);
            }
            return match_status;
        }

        /**
         * @brief 获取接触状态不匹配的腿数
         */
        [[nodiscard]] int getContactMismatchCount() const {
            int count = 0;
            for (int i = 0; i < NUM_LEGS; ++i) {
                if (contact_flags_[i] != expected_contact_[i]) {
                    ++count;
                }
            }
            return count;
        }

        void loadSettings(const std::string &task_file, bool verbose);

    private:
        // 动力学参数更新
        void updateDynamicsParameters();
        // 计算足端力
        void computeFootForce(scalar_t dt);
        // 计算步态相位概率   基于当前步态相位，计算处于接触状态的概率
        scalar_t computeGaitPhaseProbability(scalar_t gait_phase, int leg_idx, int is_swing, int is_contact) ;

        // 计算足端高度概率   足端越接近地面，接触概率越高
        scalar_t computeHeightProbability(scalar_t foot_height, int leg_idx);

        // 计算足端力概率   足端力越大，接触概率越高
        // is_swing: 是否在摆动相，摆动相时快速衰减力滤波值
        scalar_t computeForceProbability(scalar_t foot_force, int leg_idx, bool is_swing);

        // 基础成员（原本从 StateEstimateBase 继承）
        CentroidalModelInfo info_;          // 模型信息
        CtrlInterfaces& ctrl_component_;    // 控制接口
        vector_t rbd_state_;                // 机器人状态向量
        PinocchioInterface pinocchio_interface_;                        // 机器人运动学/动力学模型
        std::unique_ptr<PinocchioEndEffectorKinematics> ee_kinematics_; // 脚部运动学计算器
        DynamicParameters dynamics_parameters_;                          // 动力学参数
        
        // 足端帧名称
        const std::array<std::string, 4> foot_frame_names_ = {
            "FL_foot", "FR_foot", "RL_foot", "RR_foot"
        };

        // 卡尔曼滤波状态
        Matrix4 A_;                                             // 状态转移矩阵 (4x4, 恒等)
        Matrix4 B_;                                             // 输入矩阵 (4x4, 恒等)
        Matrix8x4 H_;                                           // 观测矩阵 (8x4)
        Matrix4 Q_;                                             // 过程噪声协方差
        Eigen::Matrix<scalar_t, NUM_LEGS * 2, NUM_LEGS * 2> R_; // 观测噪声协方差
        Matrix4 I_;                                             // 单位矩阵

        Vector4 X_est_; // 状态估计 (接触概率)
        Matrix4 P_est_; // 估计协方差
        Vector4 X_pre_; // 预测状态
        Matrix4 P_pre_; // 预测协方差

        // 高斯分布参数 (用于计算概率)
        // 相位参数
        scalar_t mu_c0_ = 0.0, sigma_c0_ = 0.01; // 支撑相早期参数
        scalar_t mu_c1_ = 1.0, sigma_c1_ = 0.01; // 支撑相后期参数
        scalar_t mu_s0_ = 0.0, sigma_s0_ = 0.01; // 摆动相早期参数
        scalar_t mu_s1_ = 1.0, sigma_s1_ = 0.01; // 摆动相后期参数
        scalar_t mu_hz_ = 0.02, sigma_hz_ = 0.01; // 足端高度参数：正常触地约0.019m，mu稍大于触地高度
        // 足端力参数
        std::array<scalar_t, NUM_LEGS> mu_fz_ = {0, 0, 0, 0};
        scalar_t sigma_fz_ = 15.0;
        LegParameters leg_parameters_[4];   // 各腿参数
        scalar_t gamma_, lambda_ = 200.0;           // 低通滤波参数

        // 预分配的向量/矩阵（避免每次调用时分配内存）
        vector_t qPino_;                    // 广义位置向量 (18)
        vector_t vPino_;                    // 广义速度向量 (18)
        vector_t zeroVel_;                  // 零速度向量 (18)
        Eigen::Matrix<double, 6, Eigen::Dynamic> J_full_;  // 完整雅可比矩阵 (6×nv)
        std::array<std::size_t, NUM_LEGS> ee_frame_ids_;  // 足端帧索引（缓存，避免在头文件包含 pinocchio）
        
        // computeFootForce 预分配变量
        Eigen::Matrix<double, 18, 1> qdot_;      // 广义速度
        Eigen::Matrix<double, 18, 1> tau_input_; // 输入力矩

        Eigen::Matrix<double, 12, 18> Jc_stack_;  // 堆叠接触雅可比 [Jc0; Jc1; Jc2; Jc3] (12×nv)
        Eigen::Matrix<double, 12, 1> tauAct_;    // 关节力矩
        Eigen::Matrix<double, 18, 1> rd_;   // 扰动力 (广义外力估计)
        Eigen::Matrix<double, 18, 1> Pk_;   // 动量 M * qdot
        Eigen::Matrix<double, 18, 1> Xn_;   // 未滤波外力估计
        Eigen::Matrix<double, 18, 1> Yn_1_; // 上一时刻滤波结果
        Eigen::Matrix<double, 18, 1> Yn_;   // 当前时刻滤波结果
        Eigen::Matrix<double, 12, 1> tauAct_prev_; // 上一时刻关节力矩（用于力矩滤波）

        // 估计的概率
        std::array<scalar_t, NUM_LEGS> P_contact_{0,0,0,0};
        std::array<scalar_t, NUM_LEGS> P_height_{0,0,0,0};
        std::array<scalar_t, NUM_LEGS> P_force_{0,0,0,0};

        // 噪声参数
        scalar_t contact_process_noise_ = 0.998;
        scalar_t tau_measure_noise_ = 0.930;
        scalar_t fheight_measure_noise_ = 0.841;

        // 原始力低通滤波（对力而非概率进行滤波，减少非线性滞后）
        std::array<scalar_t, NUM_LEGS> force_filtered_{};
        scalar_t force_filter_alpha_ = 0.5;  // 滤波系数，越大响应越快
        scalar_t swing_force_decay_ = 0.85;  // 摆动相力衰减系数，用于快速拉低力值

        // 输出
        contact_flag_t contact_flags_{};
        std::array<scalar_t, NUM_LEGS> contact_prob_{};
        std::array<scalar_t, NUM_LEGS> contact_phase_out_{};
        std::array<scalar_t, NUM_LEGS> foot_forces_{};  // 估计的足端力z分量（正值=接触力）
        std::array<scalar_t, NUM_LEGS> foot_force_z_filtered_{}; // 足端力z低通滤波值

        // MPC 期望的接触状态（用于对比分析）
        contact_flag_t expected_contact_{};

        // 阈值参数
        scalar_t contact_threshold_ = 0.5; // 接触判定阈值
        scalar_t force_threshold_ = 10.0;   // 力阈值 (N)  站立时力约 30-80N，摆动时力约 0-5N
        scalar_t height_threshold_ = 0.02; // 高度阈值 (m)

        int is_swing = 0, is_contact = 0;

        // 调试信息
        DebugInfo debug_info_{};

        bool initialized_ = false;
        scalar_t dt_ = 0.002; // 500Hz控制频率

        quadruped::logs::LogSave logger;
    };

} // namespace ocs2::legged_robot
