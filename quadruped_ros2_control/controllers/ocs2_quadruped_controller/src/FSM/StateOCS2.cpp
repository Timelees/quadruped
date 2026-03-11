//
// Created by tlab-uav on 25-2-27.
//

#include "ocs2_quadruped_controller/FSM/StateOCS2.h"

#include <algorithm>
#include <angles/angles.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_quadruped_controller/wbc/WeightedWbc.h>
#include <ocs2_sqp/SqpMpc.h>

namespace ocs2::legged_robot
{
    StateOCS2::StateOCS2(CtrlInterfaces& ctrl_interfaces,
                         const std::shared_ptr<CtrlComponent>& ctrl_component)
        : FSMState(FSMStateName::OCS2, "OCS2 State", ctrl_interfaces),
          ctrl_component_(ctrl_component),
          node_(ctrl_component->node_)
    {
        // 获取自gazebo.yaml/cyber_controllers.yaml的ros_parameters
        if (!node_->has_parameter("default_hip_kp"))
        {
            node_->declare_parameter("default_hip_kp", default_hip_kp);
        }
        if (!node_->has_parameter("default_hip_kd"))
        {
            node_->declare_parameter("default_hip_kd", default_hip_kd);
        }
        if (!node_->has_parameter("default_thigh_kp"))
        {
            node_->declare_parameter("default_thigh_kp", default_thigh_kp);
        }
        if (!node_->has_parameter("default_thigh_kd"))
        {
            node_->declare_parameter("default_thigh_kd", default_thigh_kd);
        }
        if (!node_->has_parameter("default_calf_kp"))
        {
            node_->declare_parameter("default_calf_kp", default_calf_kp);
        }
        if (!node_->has_parameter("default_calf_kd"))
        {
            node_->declare_parameter("default_calf_kd", default_calf_kd);
        }
        default_hip_kp = node_->get_parameter("default_hip_kp").as_double();
        default_hip_kd = node_->get_parameter("default_hip_kd").as_double();
        default_thigh_kp = node_->get_parameter("default_thigh_kp").as_double();
        default_thigh_kd = node_->get_parameter("default_thigh_kd").as_double();
        default_calf_kp = node_->get_parameter("default_calf_kp").as_double();
        default_calf_kd = node_->get_parameter("default_calf_kd").as_double();

        // selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(leggedInterface_->getPinocchioInterface(),
        //                                                                        leggedInterface_->getGeometryInterface(), pinocchioMapping, nh));

        // Whole body control
        wbc_ = std::make_shared<WeightedWbc>(ctrl_component_->legged_interface_->getPinocchioInterface(),
                                             ctrl_component_->legged_interface_->getCentroidalModelInfo(),
                                             *ctrl_component_->ee_kinematics_);
        wbc_->loadTasksSetting(ctrl_component_->task_file_, ctrl_component_->verbose_);

        // Safety Checker
        safety_checker_ = std::make_shared<SafetyChecker>(ctrl_component_->legged_interface_->getCentroidalModelInfo());

        // 构建 WBC 索引 → ctrl_interfaces 索引的映射表
        // WBC/模型固定顺序: FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf, RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf
        const std::vector<std::string> wbc_joint_order = {
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"
        };
        
        // ctrl_interfaces 的顺序来自 ctrl_component_->joint_names_（即 YAML 的 joints 参数）
        const auto& interface_joint_order = ctrl_component_->getJointNames();

        for (int wbc_idx = 0; wbc_idx < 12; ++wbc_idx)
        {
            const auto& wbc_joint_name = wbc_joint_order[wbc_idx];
            auto it = std::find(interface_joint_order.begin(), interface_joint_order.end(), wbc_joint_name);
            if (it != interface_joint_order.end())
            {
                wbc_to_interface_map_[wbc_idx] = static_cast<int>(std::distance(interface_joint_order.begin(), it));
            }
            else
            {
                // 如果找不到对应关节，打印警告并使用恒等映射
                RCLCPP_WARN(node_->get_logger(), 
                           "[StateOCS2] WBC joint '%s' not found in interface joint list, using identity mapping",
                           wbc_joint_name.c_str());
                wbc_to_interface_map_[wbc_idx] = wbc_idx;
            }
        }
        
        // 打印映射表用于调试
        RCLCPP_INFO(node_->get_logger(), "[StateOCS2] WBC -> Interface joint mapping:");
        for (int i = 0; i < 12; ++i)
        {
            RCLCPP_INFO(node_->get_logger(), "  WBC[%d] %s -> Interface[%d] %s",
                       i, wbc_joint_order[i].c_str(),
                       wbc_to_interface_map_[i], 
                       interface_joint_order[wbc_to_interface_map_[i]].c_str());
        }

    }

    void StateOCS2::enter()
    {
        ctrl_component_->init();
    }

    void StateOCS2::run(const rclcpp::Time& /**time**/,
                        const rclcpp::Duration& period)
    {
        if (ctrl_component_->mpc_running_ == false)
        {
            return;
        }

        // Load the latest MPC policy
        ctrl_component_->mpc_mrt_interface_->updatePolicy();

        // Evaluate the current policy
        size_t planned_mode = 0; // The mode that is active at the time the policy is evaluated at.
        ctrl_component_->mpc_mrt_interface_->evaluatePolicy(ctrl_component_->observation_.time,
                                                            ctrl_component_->observation_.state,
                                                            optimized_state_,
                                                            optimized_input_, planned_mode);

        // Whole body control
        ctrl_component_->observation_.input = optimized_input_;

        wbc_timer_.startTimer();
        vector_t x = wbc_->update(optimized_state_, optimized_input_, ctrl_component_->measured_rbd_state_,
                                  planned_mode,
                                  period.seconds());
        wbc_timer_.endTimer();
        // wbc计算的12个关节的扭矩，关节期望位置和期望速度
        vector_t torque = x.tail(12);
        vector_t pos_des = centroidal_model::getJointAngles(optimized_state_,
                                                            ctrl_component_->legged_interface_->
                                                                             getCentroidalModelInfo());
        vector_t vel_des = centroidal_model::getJointVelocities(optimized_input_,
                                                                ctrl_component_->legged_interface_->
                                                                getCentroidalModelInfo());
                                                     
        

        // FL  FR  RL  RR - WBC输出的固定顺序
        // 使用 wbc_to_interface_map_ 将 WBC 索引映射到正确的 ctrl_interfaces 索引
        for (int wbc_idx = 0; wbc_idx < 12; wbc_idx++)
        {
            int interface_idx = wbc_to_interface_map_[wbc_idx];
            
            ctrl_interfaces_.joint_torque_command_interface_[interface_idx].get().set_value(torque(wbc_idx));
            ctrl_interfaces_.joint_position_command_interface_[interface_idx].get().set_value(pos_des(wbc_idx));
            ctrl_interfaces_.joint_velocity_command_interface_[interface_idx].get().set_value(vel_des(wbc_idx));
            
            // 针对不同关节设置控制参数（基于 WBC 索引，因为 kp/kd 是按关节类型设置的）
            if(wbc_idx % 3 == 0){
                ctrl_interfaces_.joint_kd_command_interface_[interface_idx].get().set_value(default_hip_kd);
                ctrl_interfaces_.joint_kp_command_interface_[interface_idx].get().set_value(default_hip_kp);
            }
            else if(wbc_idx % 3 == 1)
            {
                ctrl_interfaces_.joint_kp_command_interface_[interface_idx].get().set_value(default_thigh_kp);
                ctrl_interfaces_.joint_kd_command_interface_[interface_idx].get().set_value(default_thigh_kd);
            }
            else
            {
                ctrl_interfaces_.joint_kp_command_interface_[interface_idx].get().set_value(default_calf_kp);
                ctrl_interfaces_.joint_kd_command_interface_[interface_idx].get().set_value(default_calf_kd);
            }

            // std::cout << "WBC[" << wbc_idx << "]->Interface[" << interface_idx << "] - Torque: " << torque(wbc_idx) << ", Position: " << pos_des(wbc_idx) << ", Velocity: " << vel_des(wbc_idx) << ", kp: " << ctrl_interfaces_.joint_kp_command_interface_[interface_idx].get().get_value() << ", kd: " << ctrl_interfaces_.joint_kd_command_interface_[interface_idx].get().get_value() << std::endl;
        }
        
    

        // Visualization
        ctrl_component_->visualizer_->update(ctrl_component_->mpc_mrt_interface_->getPolicy(),
                                             ctrl_component_->mpc_mrt_interface_->getCommand());
        
    }


    void StateOCS2::exit()
    {
        
    }

    FSMStateName StateOCS2::checkChange()
    {
        // Safety check, if failed, stop the controller
        if (!safety_checker_->check(ctrl_component_->observation_, optimized_state_, optimized_input_))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[Legged Controller] Safety check failed, stopping the controller.");
            return FSMStateName::PASSIVE;
        }
        switch (ctrl_interfaces_.control_inputs_.command)
        {
        case 1:
            return FSMStateName::PASSIVE;
        default:
            return FSMStateName::OCS2;
        }
    }
}
