// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gz_quadruped_hardware/gz_system.hpp"

#include <gz/msgs/imu.pb.h>
#include <gz/msgs/wrench.pb.h>

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gz/physics/Geometry.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/ForceTorque.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>
#define GZ_TRANSPORT_NAMESPACE gz::transport::
#define GZ_MSGS_NAMESPACE gz::msgs::

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "gz_quadruped_hardware/actuator.hpp"

struct jointData
{
    /// \brief Joint's names.
    std::string name;

    /// \brief Joint's type.
    sdf::JointType joint_type;

    /// \brief Joint's axis.
    sdf::JointAxis joint_axis;

    /// \brief Current joint position
    double joint_position;

    /// \brief Current joint velocity
    double joint_velocity;

    /// \brief Current joint effort
    double joint_effort;

    /// \brief Current cmd joint position
    double joint_position_cmd;

    /// \brief Current cmd joint velocity
    double joint_velocity_cmd;

    /// \brief Current cmd joint effort
    double joint_effort_cmd;

    double joint_kp_cmd;

    double joint_kd_cmd;

    /// \brief flag if joint is actuated (has command interfaces) or passive
    bool is_actuated;

    /// \brief handles to the joints from within Gazebo
    sim::Entity sim_joint;

    // 关节电机执行器索引
    int actuator_index = -1;

    /// \brief Control method defined in the URDF for each joint.
    gz_quadruped_hardware::GazeboSimSystemInterface::ControlMethod joint_control_method;
};

class ImuData
{
public:
    /// \brief imu's name.
    std::string name{};

    /// \brief imu's topic name.
    std::string topicName{};

    /// \brief handles to the imu from within Gazebo
    sim::Entity sim_imu_sensors_ = sim::kNullEntity;

    /// \brief An array per IMU with 4 orientation, 3 angular velocity and 3 linear acceleration
    std::array<double, 10> imu_sensor_data_;

    /// \brief callback to get the IMU topic values
    void OnIMU(const GZ_MSGS_NAMESPACE IMU& _msg);
};

void ImuData::OnIMU(const GZ_MSGS_NAMESPACE IMU& _msg)
{
    this->imu_sensor_data_[0] = _msg.orientation().x();
    this->imu_sensor_data_[1] = _msg.orientation().y();
    this->imu_sensor_data_[2] = _msg.orientation().z();
    this->imu_sensor_data_[3] = _msg.orientation().w();
    this->imu_sensor_data_[4] = _msg.angular_velocity().x();
    this->imu_sensor_data_[5] = _msg.angular_velocity().y();
    this->imu_sensor_data_[6] = _msg.angular_velocity().z();
    this->imu_sensor_data_[7] = _msg.linear_acceleration().x();
    this->imu_sensor_data_[8] = _msg.linear_acceleration().y();
    this->imu_sensor_data_[9] = _msg.linear_acceleration().z();
}

class ForceTorqueData
{
public:
    /// \brief force-torque sensor's name.
    std::string name{};

    /// \brief force-torque sensor's topic name.
    std::string topicName{};

    /// \brief handles to the force torque from within Gazebo
    sim::Entity sim_ft_sensors_ = sim::kNullEntity;

    /// \brief An array per FT
    std::array<double, 6> ft_sensor_data_;

    /// \brief  Current foot end effort
    double foot_effort;

    /// \brief callback to get the Force Torque topic values
    void OnForceTorque(const GZ_MSGS_NAMESPACE Wrench& _msg);
};

void ForceTorqueData::OnForceTorque(const GZ_MSGS_NAMESPACE Wrench& _msg)
{
    this->ft_sensor_data_[0] = _msg.force().x();
    this->ft_sensor_data_[1] = _msg.force().y();
    this->ft_sensor_data_[2] = _msg.force().z();
    this->ft_sensor_data_[3] = _msg.torque().x();
    this->ft_sensor_data_[4] = _msg.torque().y();
    this->ft_sensor_data_[5] = _msg.torque().z();
    this->foot_effort = sqrt(pow(_msg.force().x(), 2) + pow(_msg.force().y(), 2) + pow(_msg.force().z(), 2));
}

/// \brief Ground Truth Odometry Data - 获取机器人在Gazebo中的真实位姿
class OdomData
{
public:
    /// \brief odom sensor's name (对应xacro中的sensor name)
    std::string name{};

    /// \brief Link name to track (机器人base link的名称)
    std::string link_name{};

    /// \brief Entity of the link to track
    sim::Entity sim_link_ = sim::kNullEntity;

    /// \brief Odom data: [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, 
    ///                    orient_x, orient_y, orient_z, orient_w,
    ///                    angular_vel_x, angular_vel_y, angular_vel_z]
    std::array<double, 13> odom_data_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                       0.0, 0.0, 0.0, 1.0,  // quaternion (x,y,z,w), w=1 for identity
                                       0.0, 0.0, 0.0};
};

class gz_quadruped_hardware::GazeboSimSystemPrivate
{
public:
    GazeboSimSystemPrivate() = default;

    ~GazeboSimSystemPrivate() = default;

    /// \brief Degrees od freedom.
    size_t n_dof_;

    /// \brief last time the write method was called.
    rclcpp::Time last_update_sim_time_ros_;

    /// \brief vector with the joint's names.
    std::vector<jointData> joints_;

    /// \brief vector with the imus .
    std::vector<std::shared_ptr<ImuData>> imus_;

    /// \brief vector with the foot force-torque sensors.
    std::vector<std::shared_ptr<ForceTorqueData>> ft_sensors_;

    /// \brief vector with the ground truth odometry sensors.
    std::vector<std::shared_ptr<OdomData>> odoms_;

    /// \brief state interfaces that will be exported to the Resource Manager
    std::vector<hardware_interface::StateInterface> state_interfaces_;

    /// \brief command interfaces that will be exported to the Resource Manager
    std::vector<hardware_interface::CommandInterface> command_interfaces_;

    /// \brief Entity component manager, ECM shouldn't be accessed outside those
    /// methods, otherwise the app will crash
    sim::EntityComponentManager* ecm;

    /// \brief controller update rate
    unsigned int update_rate;

    /// \brief Gazebo communication node.
    GZ_TRANSPORT_NAMESPACE Node node;

    rclcpp::Publisher<control_input_msgs::msg::SimImu>::SharedPtr imu_pub;
    rclcpp::Publisher<control_input_msgs::msg::SimJoint>::SharedPtr joint_pub;
   
    rclcpp::Node::SharedPtr ros_node;

    // Enable currentloop response limit of the motors
    bool use_currentloop_response_ = true;
    // Enable TN curve limit of the motors
    bool use_TNcurve_motormodel_ = true;

    gazebo::Actuator_ motor_;
    
};

namespace gz_quadruped_hardware
{
    bool GazeboSimSystem::initSim(
        rclcpp::Node::SharedPtr& model_nh,
        std::map<std::string, sim::Entity>& enableJoints,
        const hardware_interface::HardwareInfo& hardware_info,      // 存放的xacro里的那些信息
        sim::EntityComponentManager& _ecm,
        int& update_rate)
    {
        this->dataPtr = std::make_unique<GazeboSimSystemPrivate>();
        this->dataPtr->last_update_sim_time_ros_ = rclcpp::Time();

        this->nh_ = model_nh;
        this->dataPtr->ecm = &_ecm;
        this->dataPtr->n_dof_ = hardware_info.joints.size();



        this->dataPtr->update_rate = update_rate;
        // 初始化ros节点
        this->dataPtr->ros_node = model_nh;

        
        RCLCPP_DEBUG(this->nh_->get_logger(), "n_dof_ %lu", this->dataPtr->n_dof_);

        this->dataPtr->joints_.resize(this->dataPtr->n_dof_);

        if (this->dataPtr->n_dof_ == 0)
        {
            RCLCPP_ERROR_STREAM(this->nh_->get_logger(), "There is no joint available");
            return false;
        }

        for (unsigned int j = 0; j < this->dataPtr->n_dof_; j++)
        {
            auto& joint_info = hardware_info.joints[j];
            std::string joint_name = this->dataPtr->joints_[j].name = joint_info.name;

            auto it_joint = enableJoints.find(joint_name);
            if (it_joint == enableJoints.end())
            {
                RCLCPP_WARN_STREAM(
                    this->nh_->get_logger(), "Skipping joint in the URDF named '" << joint_name <<
                    "' which is not in the gazebo model.");
                continue;
            }

            sim::Entity simjoint = enableJoints[joint_name];
            this->dataPtr->joints_[j].sim_joint = simjoint;
            this->dataPtr->joints_[j].joint_type = _ecm.Component<sim::components::JointType>(
                simjoint)->Data();
            this->dataPtr->joints_[j].joint_axis = _ecm.Component<sim::components::JointAxis>(
                simjoint)->Data();

            // 分配执行器索引（0 到 11）
            this->dataPtr->joints_[j].actuator_index = j;
            

            // Create joint position component if one doesn't exist
            if (!_ecm.EntityHasComponentType(
                simjoint,
                sim::components::JointPosition().TypeId()))
            {
                _ecm.CreateComponent(simjoint, sim::components::JointPosition());
            }

            // Create joint velocity component if one doesn't exist
            if (!_ecm.EntityHasComponentType(
                simjoint,
                sim::components::JointVelocity().TypeId()))
            {
                _ecm.CreateComponent(simjoint, sim::components::JointVelocity());
            }

            // Create joint transmitted wrench component if one doesn't exist
            if (!_ecm.EntityHasComponentType(
                simjoint,
                sim::components::JointTransmittedWrench().TypeId()))
            {
                _ecm.CreateComponent(simjoint, sim::components::JointTransmittedWrench());
            }

            // Accept this joint and continue configuration
            RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading joint: " << joint_name);

            RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tState:");

            auto get_initial_value =
                [this, joint_name](const hardware_interface::InterfaceInfo& interface_info)
            {
                double initial_value{0.0};
                if (!interface_info.initial_value.empty())
                {
                    try
                    {
                        initial_value = hardware_interface::stod(interface_info.initial_value);
                        RCLCPP_INFO(this->nh_->get_logger(), "\t\t\t found initial value: %f", initial_value);
                    }
                    catch (std::invalid_argument&)
                    {
                        RCLCPP_ERROR_STREAM(
                            this->nh_->get_logger(),
                            "Failed converting initial_value string to real number for the joint "
                            << joint_name
                            << " and state interface " << interface_info.name
                            << ". Actual value of parameter: " << interface_info.initial_value
                            << ". Initial value will be set to 0.0");
                        throw std::invalid_argument("Failed converting initial_value string");
                    }
                }
                return initial_value;
            };

            double initial_position = std::numeric_limits<double>::quiet_NaN();
            double initial_velocity = std::numeric_limits<double>::quiet_NaN();
            double initial_effort = std::numeric_limits<double>::quiet_NaN();

            // register the state handles
            for (unsigned int i = 0; i < joint_info.state_interfaces.size(); ++i)
            {
                if (joint_info.state_interfaces[i].name == "position")
                {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
                    this->dataPtr->state_interfaces_.emplace_back(
                        joint_name,
                        hardware_interface::HW_IF_POSITION,
                        &this->dataPtr->joints_[j].joint_position);
                    initial_position = get_initial_value(joint_info.state_interfaces[i]);
                    this->dataPtr->joints_[j].joint_position = initial_position;
                }
                if (joint_info.state_interfaces[i].name == "velocity")
                {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
                    this->dataPtr->state_interfaces_.emplace_back(
                        joint_name,
                        hardware_interface::HW_IF_VELOCITY,
                        &this->dataPtr->joints_[j].joint_velocity);
                    initial_velocity = get_initial_value(joint_info.state_interfaces[i]);
                    this->dataPtr->joints_[j].joint_velocity = initial_velocity;
                }
                if (joint_info.state_interfaces[i].name == "effort")
                {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
                    this->dataPtr->state_interfaces_.emplace_back(
                        joint_name,
                        hardware_interface::HW_IF_EFFORT,
                        &this->dataPtr->joints_[j].joint_effort);
                    initial_effort = get_initial_value(joint_info.state_interfaces[i]);
                    this->dataPtr->joints_[j].joint_effort = initial_effort;
                }
            }

            RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tCommand:");

            // register the command handles
            for (unsigned int i = 0; i < joint_info.command_interfaces.size(); ++i)
            {
                if (joint_info.command_interfaces[i].name == "position")
                {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
                    this->dataPtr->command_interfaces_.emplace_back(
                        joint_name,
                        hardware_interface::HW_IF_POSITION,
                        &this->dataPtr->joints_[j].joint_position_cmd);
                    if (!std::isnan(initial_position))
                    {
                        this->dataPtr->joints_[j].joint_position_cmd = initial_position;
                    }
                }
                else if (joint_info.command_interfaces[i].name == "velocity")
                {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
                    this->dataPtr->command_interfaces_.emplace_back(
                        joint_name,
                        hardware_interface::HW_IF_VELOCITY,
                        &this->dataPtr->joints_[j].joint_velocity_cmd);
                    if (!std::isnan(initial_velocity))
                    {
                        this->dataPtr->joints_[j].joint_velocity_cmd = initial_velocity;
                    }
                }
                else if (joint_info.command_interfaces[i].name == "effort")
                {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
                    this->dataPtr->command_interfaces_.emplace_back(
                        joint_name,
                        hardware_interface::HW_IF_EFFORT,
                        &this->dataPtr->joints_[j].joint_effort_cmd);
                    if (!std::isnan(initial_effort))
                    {
                        this->dataPtr->joints_[j].joint_effort_cmd = initial_effort;
                    }
                }
                else if (joint_info.command_interfaces[i].name == "kp")
                {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t kp");
                    this->dataPtr->command_interfaces_.emplace_back(
                        joint_name,
                        "kp",
                        &this->dataPtr->joints_[j].joint_kp_cmd);
                    this->dataPtr->joints_[j].joint_kp_cmd = 0.0;
                }
                else if (joint_info.command_interfaces[i].name == "kd")
                {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t kd");
                    this->dataPtr->command_interfaces_.emplace_back(
                        joint_name,
                        "kd",
                        &this->dataPtr->joints_[j].joint_kd_cmd);
                    this->dataPtr->joints_[j].joint_kd_cmd = 0.0;
                }

                // independently of existence of command interface set initial value if defined
                if (!std::isnan(initial_position))
                {
                    this->dataPtr->joints_[j].joint_position = initial_position;
                    this->dataPtr->ecm->CreateComponent(
                        this->dataPtr->joints_[j].sim_joint,
                        sim::components::JointPositionReset({initial_position}));
                }
                if (!std::isnan(initial_velocity))
                {
                    this->dataPtr->joints_[j].joint_velocity = initial_velocity;
                    this->dataPtr->ecm->CreateComponent(
                        this->dataPtr->joints_[j].sim_joint,
                        sim::components::JointVelocityReset({initial_velocity}));
                }
            }

            // check if joint is actuated (has command interfaces) or passive
            this->dataPtr->joints_[j].is_actuated = joint_info.command_interfaces.size() > 0;
        }

        registerSensors(hardware_info);

        return true;
    }

    
    void GazeboSimSystem::registerSensors(
        const hardware_interface::HardwareInfo& hardware_info)
    {
        // Collect gazebo sensor handles
        size_t n_sensors = hardware_info.sensors.size();
        std::vector<hardware_interface::ComponentInfo> sensor_components_;

        for (unsigned int j = 0; j < n_sensors; j++)
        {
            hardware_interface::ComponentInfo component = hardware_info.sensors[j];
            sensor_components_.push_back(component);
        }
        // This is split in two steps: Count the number and type of sensor and associate the interfaces
        // So we have resize only once the structures where the data will be stored, and we can safely
        // use pointers to the structures

        // IMU Sensors
        this->dataPtr->ecm->Each<sim::components::Imu,
                                 sim::components::Name>(
            [&](const sim::Entity& _entity,
                const sim::components::Imu*,
                const sim::components::Name* _name) -> bool
            {
                auto imuData = std::make_shared<ImuData>();
                RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading sensor: " << _name->Data());

                auto sensorTopicComp = this->dataPtr->ecm->Component<
                    sim::components::SensorTopic>(_entity);
                if (sensorTopicComp)
                {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Topic name: " << sensorTopicComp->Data());
                }

                RCLCPP_INFO_STREAM(
                    this->nh_->get_logger(), "\tState:");
                imuData->name = _name->Data();
                imuData->sim_imu_sensors_ = _entity;

                hardware_interface::ComponentInfo component;
                for (auto& comp : sensor_components_)
                {
                    if (comp.name == _name->Data())
                    {
                        component = comp;
                    }
                }

                static const std::map<std::string, size_t> interface_name_map = {
                    {"orientation.x", 0},
                    {"orientation.y", 1},
                    {"orientation.z", 2},
                    {"orientation.w", 3},
                    {"angular_velocity.x", 4},
                    {"angular_velocity.y", 5},
                    {"angular_velocity.z", 6},
                    {"linear_acceleration.x", 7},
                    {"linear_acceleration.y", 8},
                    {"linear_acceleration.z", 9},
                };

                for (const auto& state_interface : component.state_interfaces)
                {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t " << state_interface.name);

                    size_t data_index = interface_name_map.at(state_interface.name);
                    this->dataPtr->state_interfaces_.emplace_back(
                        imuData->name,
                        state_interface.name,
                        &imuData->imu_sensor_data_[data_index]);
                }
                this->dataPtr->imus_.push_back(imuData);
                return true;
            });

        // Foot Force torque sensor
        this->dataPtr->ecm->Each<sim::components::ForceTorque,
                                 sim::components::Name>(
            [&](const sim::Entity& _entity,
                const sim::components::ForceTorque*,
                const sim::components::Name* _name) -> bool
            {
                auto ftData = std::make_shared<ForceTorqueData>();
                RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading Foot Force sensor: " << _name->Data());

                auto sensorTopicComp = this->dataPtr->ecm->Component<
                    sim::components::SensorTopic>(_entity);
                if (sensorTopicComp)
                {
                    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Topic name: " << sensorTopicComp->Data());
                }
                ftData->name = _name->Data();
                ftData->sim_ft_sensors_ = _entity;
                this->dataPtr->state_interfaces_.emplace_back(
                    "foot_force",
                    ftData->name,
                    &ftData->foot_effort);
                this->dataPtr->ft_sensors_.push_back(ftData);
                return true;
            });

        // Ground Truth Odometry Sensor - 从URDF/xacro中读取配置
        // 查找名为 "ground_truth" 的传感器配置
        for (const auto& component : sensor_components_)
        {
            if (component.name == "ground_truth")
            {
                auto odomData = std::make_shared<OdomData>();
                odomData->name = component.name;
                
                // 从参数中获取要追踪的link名称，默认为 "base"
                if (component.parameters.find("link_name") != component.parameters.end())
                {
                    odomData->link_name = component.parameters.at("link_name");
                }
                else
                {
                    odomData->link_name = "base";  // 默认追踪base link
                }

                RCLCPP_INFO_STREAM(this->nh_->get_logger(), 
                    "Loading Ground Truth Odom sensor: " << odomData->name << 
                    ", tracking link: " << odomData->link_name);

                // 查找对应的link entity
                this->dataPtr->ecm->Each<sim::components::Link, sim::components::Name>(
                    [&](const sim::Entity& _entity,
                        const sim::components::Link*,
                        const sim::components::Name* _name) -> bool
                    {
                        if (_name->Data() == odomData->link_name)
                        {
                            odomData->sim_link_ = _entity;
                            RCLCPP_INFO_STREAM(this->nh_->get_logger(), 
                                "Found link entity for: " << odomData->link_name);

                            // 确保该link有WorldPose组件
                            if (!this->dataPtr->ecm->EntityHasComponentType(
                                _entity, sim::components::WorldPose().TypeId()))
                            {
                                this->dataPtr->ecm->CreateComponent(_entity, sim::components::WorldPose());
                            }
                            // 确保该link有WorldLinearVelocity组件
                            if (!this->dataPtr->ecm->EntityHasComponentType(
                                _entity, sim::components::WorldLinearVelocity().TypeId()))
                            {
                                this->dataPtr->ecm->CreateComponent(_entity, sim::components::WorldLinearVelocity());
                            }
                            // 确保该link有WorldAngularVelocity组件（与WorldPose/WorldLinearVelocity保持一致）
                            // 注意：混用 world-frame 线速度 + body-frame 角速度会导致下游估计/控制出现系统性漂移。
                            if (!this->dataPtr->ecm->EntityHasComponentType(
                                _entity, sim::components::WorldAngularVelocity().TypeId()))
                            {
                                this->dataPtr->ecm->CreateComponent(_entity, sim::components::WorldAngularVelocity());
                            }
                            return false;  // 找到后停止遍历
                        }
                        return true;
                    });

                if (odomData->sim_link_ == sim::kNullEntity)
                {
                    RCLCPP_WARN_STREAM(this->nh_->get_logger(), 
                        "Could not find link: " << odomData->link_name << " for ground truth odom");
                }

                // 注册状态接口
                static const std::map<std::string, size_t> odom_interface_map = {
                    {"position.x", 0},
                    {"position.y", 1},
                    {"position.z", 2},
                    {"linear_velocity.x", 3},
                    {"linear_velocity.y", 4},
                    {"linear_velocity.z", 5},
                    {"orientation.x", 6},
                    {"orientation.y", 7},
                    {"orientation.z", 8},
                    {"orientation.w", 9},
                    {"angular_velocity.x", 10},
                    {"angular_velocity.y", 11},
                    {"angular_velocity.z", 12},
                };

                RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tState interfaces:");
                for (const auto& state_interface : component.state_interfaces)
                {
                    auto it = odom_interface_map.find(state_interface.name);
                    if (it != odom_interface_map.end())
                    {
                        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t " << state_interface.name);
                        this->dataPtr->state_interfaces_.emplace_back(
                            odomData->name,
                            state_interface.name,
                            &odomData->odom_data_[it->second]);
                    }
                    else
                    {
                        RCLCPP_WARN_STREAM(this->nh_->get_logger(), 
                            "Unknown odom interface: " << state_interface.name);
                    }
                }

                this->dataPtr->odoms_.push_back(odomData);
            }
        }
    }

    CallbackReturn GazeboSimSystem::on_init(const hardware_interface::HardwareInfo& info)
    {
        if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn GazeboSimSystem::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(
            this->nh_->get_logger(), "System Successfully configured!");

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> GazeboSimSystem::export_state_interfaces()
    {
        return std::move(this->dataPtr->state_interfaces_);
    }

    std::vector<hardware_interface::CommandInterface> GazeboSimSystem::export_command_interfaces()
    {
        return std::move(this->dataPtr->command_interfaces_);
    }

    CallbackReturn GazeboSimSystem::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn GazeboSimSystem::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type GazeboSimSystem::read(
        const rclcpp::Time& /*time*/,
        const rclcpp::Duration& /*period*/)
    {
        for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i)
        {
            if (this->dataPtr->joints_[i].sim_joint == sim::kNullEntity)
            {
                continue;
            }

            // Get the joint velocity
            const auto* jointVelocity =
                this->dataPtr->ecm->Component<sim::components::JointVelocity>(
                    this->dataPtr->joints_[i].sim_joint);

            // Get the joint force via joint transmitted wrench
            const auto* jointWrench =
                this->dataPtr->ecm->Component<sim::components::JointTransmittedWrench>(
                    this->dataPtr->joints_[i].sim_joint);

            // Get the joint position
            const auto* jointPositions =
                this->dataPtr->ecm->Component<sim::components::JointPosition>(
                    this->dataPtr->joints_[i].sim_joint);

            this->dataPtr->joints_[i].joint_position = jointPositions->Data()[0];
            this->dataPtr->joints_[i].joint_velocity = jointVelocity->Data()[0];
            gz::physics::Vector3d force_or_torque;
            if (this->dataPtr->joints_[i].joint_type == sdf::JointType::PRISMATIC)
            {
                force_or_torque = {
                    jointWrench->Data().force().x(),
                    jointWrench->Data().force().y(),
                    jointWrench->Data().force().z()
                };
            }
            else
            {
                // REVOLUTE and CONTINUOUS
                force_or_torque = {
                    jointWrench->Data().torque().x(),
                    jointWrench->Data().torque().y(),
                    jointWrench->Data().torque().z()
                };
            }
            // Calculate the scalar effort along the joint axis
            this->dataPtr->joints_[i].joint_effort = force_or_torque.dot(
                gz::physics::Vector3d{
                    this->dataPtr->joints_[i].joint_axis.Xyz()[0],
                    this->dataPtr->joints_[i].joint_axis.Xyz()[1],
                    this->dataPtr->joints_[i].joint_axis.Xyz()[2]
                });
            
        }

        for (unsigned int i = 0; i < this->dataPtr->imus_.size(); ++i)
        {
            if (this->dataPtr->imus_[i]->topicName.empty())
            {
                auto sensorTopicComp = this->dataPtr->ecm->Component<
                    sim::components::SensorTopic>(this->dataPtr->imus_[i]->sim_imu_sensors_);
                if (sensorTopicComp)
                {
                    this->dataPtr->imus_[i]->topicName = sensorTopicComp->Data();
                    RCLCPP_INFO_STREAM(
                        this->nh_->get_logger(), "IMU " << this->dataPtr->imus_[i]->name <<
                        " has a topic name: " << sensorTopicComp->Data());

                    this->dataPtr->node.Subscribe(
                        this->dataPtr->imus_[i]->topicName, &ImuData::OnIMU,
                        this->dataPtr->imus_[i].get());
                }
            }
        }

        for (unsigned int i = 0; i < this->dataPtr->ft_sensors_.size(); ++i)
        {
            if (this->dataPtr->ft_sensors_[i]->topicName.empty())
            {
                auto sensorTopicComp = this->dataPtr->ecm->Component<
                    sim::components::SensorTopic>(this->dataPtr->ft_sensors_[i]->sim_ft_sensors_);
                if (sensorTopicComp)
                {
                    this->dataPtr->ft_sensors_[i]->topicName = sensorTopicComp->Data();
                    RCLCPP_INFO_STREAM(
                        this->nh_->get_logger(), "ForceTorque " << this->dataPtr->ft_sensors_[i]->name <<
                        " has a topic name: " << sensorTopicComp->Data());

                    this->dataPtr->node.Subscribe(
                        this->dataPtr->ft_sensors_[i]->topicName, &ForceTorqueData::OnForceTorque,
                        this->dataPtr->ft_sensors_[i].get());
                }
            }
        }

        // Read Ground Truth Odometry data from Gazebo
        for (unsigned int i = 0; i < this->dataPtr->odoms_.size(); ++i)
        {
            if (this->dataPtr->odoms_[i]->sim_link_ == sim::kNullEntity)
            {
                continue;
            }

            // Get the world pose of the link (position + orientation)
            const auto* worldPose = this->dataPtr->ecm->Component<sim::components::WorldPose>(
                this->dataPtr->odoms_[i]->sim_link_);
            
            // Get the world linear velocity of the link
            const auto* worldLinVel = this->dataPtr->ecm->Component<sim::components::WorldLinearVelocity>(
                this->dataPtr->odoms_[i]->sim_link_);

            // Get the world angular velocity of the link
            const auto* worldAngVel = this->dataPtr->ecm->Component<sim::components::WorldAngularVelocity>(
                this->dataPtr->odoms_[i]->sim_link_);

            if (worldPose)
            {
                // Position
                this->dataPtr->odoms_[i]->odom_data_[0] = worldPose->Data().Pos().X();
                this->dataPtr->odoms_[i]->odom_data_[1] = worldPose->Data().Pos().Y();
                this->dataPtr->odoms_[i]->odom_data_[2] = worldPose->Data().Pos().Z();
                // Orientation (quaternion: x, y, z, w)
                this->dataPtr->odoms_[i]->odom_data_[6] = worldPose->Data().Rot().X();
                this->dataPtr->odoms_[i]->odom_data_[7] = worldPose->Data().Rot().Y();
                this->dataPtr->odoms_[i]->odom_data_[8] = worldPose->Data().Rot().Z();
                this->dataPtr->odoms_[i]->odom_data_[9] = worldPose->Data().Rot().W();
            }

            if (worldLinVel)
            {
                this->dataPtr->odoms_[i]->odom_data_[3] = worldLinVel->Data().X();
                this->dataPtr->odoms_[i]->odom_data_[4] = worldLinVel->Data().Y();
                this->dataPtr->odoms_[i]->odom_data_[5] = worldLinVel->Data().Z();
            }

            if (worldAngVel)
            {
                // Angular velocity (world frame)
                this->dataPtr->odoms_[i]->odom_data_[10] = worldAngVel->Data().X();
                this->dataPtr->odoms_[i]->odom_data_[11] = worldAngVel->Data().Y();
                this->dataPtr->odoms_[i]->odom_data_[12] = worldAngVel->Data().Z();
            }
        }


        return hardware_interface::return_type::OK;
    }
    // 关节控制模式切换
    hardware_interface::return_type
    GazeboSimSystem::perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces)
    {
        for (unsigned int j = 0; j < this->dataPtr->joints_.size(); j++)
        {
            for (const std::string& interface_name : stop_interfaces)
            {
                // Clear joint control method bits corresponding to stop interfaces
                if (interface_name == (this->dataPtr->joints_[j].name + "/" +
                    hardware_interface::HW_IF_POSITION))
                {
                    this->dataPtr->joints_[j].joint_control_method &=
                        static_cast<ControlMethod_>(VELOCITY & EFFORT);
                }
                else if (interface_name == (this->dataPtr->joints_[j].name + "/" + // NOLINT
                    hardware_interface::HW_IF_VELOCITY))
                {
                    this->dataPtr->joints_[j].joint_control_method &=
                        static_cast<ControlMethod_>(POSITION & EFFORT);
                }
                else if (interface_name == (this->dataPtr->joints_[j].name + "/" + // NOLINT
                    hardware_interface::HW_IF_EFFORT))
                {
                    this->dataPtr->joints_[j].joint_control_method &=
                        static_cast<ControlMethod_>(POSITION & VELOCITY);
                }
            }

            // Set joint control method bits corresponding to start interfaces
            for (const std::string& interface_name : start_interfaces)
            {
                if (interface_name == (this->dataPtr->joints_[j].name + "/" +
                    hardware_interface::HW_IF_POSITION))
                {
                    this->dataPtr->joints_[j].joint_control_method |= POSITION;
                }
                else if (interface_name == (this->dataPtr->joints_[j].name + "/" + // NOLINT
                    hardware_interface::HW_IF_VELOCITY))
                {
                    this->dataPtr->joints_[j].joint_control_method |= VELOCITY;
                }
                else if (interface_name == (this->dataPtr->joints_[j].name + "/" + // NOLINT
                    hardware_interface::HW_IF_EFFORT))
                {
                    this->dataPtr->joints_[j].joint_control_method |= EFFORT;
                }
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type GazeboSimSystem::write(
        const rclcpp::Time& /*time*/,
        const rclcpp::Duration& /*period*/)
    {
        for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i)
        {
            if (this->dataPtr->joints_[i].sim_joint == sim::kNullEntity)
            {
                continue;
            }
            if (!this->dataPtr->ecm->Component<sim::components::JointForceCmd>(
                this->dataPtr->joints_[i].sim_joint))
            {
                this->dataPtr->ecm->CreateComponent(
                    this->dataPtr->joints_[i].sim_joint,
                    sim::components::JointForceCmd({0}));
            }
            else
            {
                const auto jointEffortCmd =
                    this->dataPtr->ecm->Component<sim::components::JointForceCmd>(
                        this->dataPtr->joints_[i].sim_joint);
                
                // PD控制器
                double torque = this->dataPtr->joints_[i].joint_effort_cmd +
                    this->dataPtr->joints_[i].joint_kp_cmd * (
                        this->dataPtr->joints_[i].joint_position_cmd -
                        this->dataPtr->joints_[i].joint_position)
                    +
                    this->dataPtr->joints_[i].joint_kd_cmd * (
                        this->dataPtr->joints_[i].joint_velocity_cmd -
                        this->dataPtr->joints_[i].joint_velocity);
                
                // TN 曲线电机模型
                if(this->dataPtr->use_TNcurve_motormodel_){
                    torque = this->dataPtr->motor_.GetTorque(torque, this->dataPtr->joints_[i].joint_velocity);
                }

                // 电流环响应
                if(this->dataPtr->use_currentloop_response_){
                    torque = this->dataPtr->motor_.CerrentLoopResponse(torque, this->dataPtr->joints_[i].joint_velocity, 
                                                                        this->dataPtr->joints_[i].actuator_index);
                }
                
                *jointEffortCmd = sim::components::JointForceCmd(
                    {torque});
                
                
            }
            
        }

        return hardware_interface::return_type::OK;
    }
} // namespace gz_quadruped_hardware

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
    gz_quadruped_hardware::GazeboSimSystem, gz_quadruped_hardware::GazeboSimSystemInterface)
