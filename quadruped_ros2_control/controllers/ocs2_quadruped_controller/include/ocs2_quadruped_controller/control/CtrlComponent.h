//
// Created by biao on 3/15/25.
//

#ifndef CTRLCOMPONENT_H
#define CTRLCOMPONENT_H
#include <memory>
#include <string>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_quadruped_controller/estimator/StateEstimateBase.h>
#include <ocs2_quadruped_controller/interface/LeggedInterface.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_quadruped_controller/perceptive/visualize/FootPlacementVisualization.h>
#include <ocs2_quadruped_controller/perceptive/visualize/SphereVisualization.h>

#include "TargetManager.h"
#include "ocs2_quadruped_controller/interface/io_message/LowlevelCmd.h"
#include "ocs2_quadruped_controller/interface/io_message/LowlevelState.h"
#include "ocs2_quadruped_controller/interface/io_interface/IOInterface.h"

namespace ocs2
{
    class MPC_BASE;
    class MPC_MRT_Interface;
    class CentroidalModelRbdConversions;
}

namespace ocs2::legged_robot
{

    class CtrlComponent
    {
    public:
        explicit CtrlComponent(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                               CtrlInterfaces& ctrl_interfaces,
                             LowlevelCmd* lowCmd, LowlevelState* lowState, IOInterface* ioInter);

        void setupStateEstimate(const std::string& estimator_type);
        void updateState(const rclcpp::Time& time, const rclcpp::Duration& period);
        void init();

        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        std::unique_ptr<LeggedInterface> legged_interface_;
        std::unique_ptr<PinocchioEndEffectorKinematics> ee_kinematics_; // 足端动力学
        std::unique_ptr<LeggedRobotVisualizer> visualizer_;
        std::shared_ptr<MPC_BASE> mpc_;
        std::unique_ptr<MPC_MRT_Interface> mpc_mrt_interface_;  // MPC实时接口

        SystemObservation observation_;     // 当前系统观测值
        vector_t measured_rbd_state_;       // 测量到的刚体状态
        std::atomic_bool mpc_running_{};

        bool verbose_ = false;

        std::string task_file_;     // 算法参数
        std::string urdf_file_;     
        std::string reference_file_;    // 默认参考
        std::string gait_file_;     // 步态时序参数
       

        LowlevelCmd* lowCmd;
        LowlevelState* lowState;
        IOInterface* ioInter;

    private:
        void setupLeggedInterface();
        void setupMpc();
        void setupMrt();

        bool enable_perceptive_ = false;
        CtrlInterfaces& ctrl_interfaces_;
        std::unique_ptr<StateEstimateBase> estimator_;
        std::unique_ptr<CentroidalModelRbdConversions> rbd_conversions_;
        std::unique_ptr<TargetManager> target_manager_;

        std::unique_ptr<FootPlacementVisualization> footPlacementVisualizationPtr_;
        std::unique_ptr<SphereVisualization> sphereVisualizationPtr_;

        std::vector<std::string> joint_names_;
        std::vector<std::string> feet_names_;
        std::string robot_pkg_;

        // Nonlinear MPC
        benchmark::RepeatedTimer mpc_timer_;
        std::thread mpc_thread_;
        std::atomic_bool controller_running_{};

        
    };
}


#endif //CTRLCOMPONENT_H
