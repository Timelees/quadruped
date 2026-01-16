

#include "cyber_quadruped_hardware/cyber_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


// ============================================================================
// CyberSystem 实现
// ============================================================================

namespace cyber_quadruped_hardware
{

std::string CyberSystem::getLcmUrl(int64_t port, int64_t ttl)
{
    return "udpm://239.255.76.67:" + std::to_string(port) + "?ttl=" + std::to_string(ttl);
}

hardware_interface::CallbackReturn CyberSystem::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != 
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 初始化关节数据
    joints_.resize(info_.joints.size());
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        joints_[i].name = info_.joints[i].name;
        joints_[i].position = 0.0;
        joints_[i].velocity = 0.0;
        joints_[i].effort = 0.0;
        joints_[i].position_cmd = 0.0;
        joints_[i].velocity_cmd = 0.0;
        joints_[i].effort_cmd = 0.0;
        joints_[i].kp_cmd = 0.0;
        joints_[i].kd_cmd = 0.0;
    }

    // 初始化 IMU 数据
    if (!info_.sensors.empty())
    {
        imu_data_.name = info_.sensors[0].name;
    }

    // 初始化机器人状态
    memset(&robot_state_, 0, sizeof(robot_state_));
    memset(&motor_cmd_, 0, sizeof(motor_cmd_));

    RCLCPP_INFO(rclcpp::get_logger("CyberSystem"), 
                "Successfully initialized with %zu joints", joints_.size());

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CyberSystem::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("CyberSystem"), "Configuring...");

    // 创建 LCM 实例
    try
    {
        motor_data_lcm_ = std::make_unique<lcm::LCM>(getLcmUrl(7667, 255));
        motor_ctrl_state_lcm_ = std::make_unique<lcm::LCM>(getLcmUrl(7667, 255));
        robot_state_lcm_ = std::make_unique<lcm::LCM>(getLcmUrl(7669, 255));
        motor_ctrl_lcm_ = std::make_unique<lcm::LCM>(getLcmUrl(7667, 255)); 
        leg_data_lcm_ = std::make_unique<lcm::LCM>(getLcmUrl(7667, 255));

        robot_state_.err_flag |= 0x02;

        if (!motor_data_lcm_->good() || !motor_ctrl_state_lcm_->good() ||
            !robot_state_lcm_->good() || !motor_ctrl_lcm_->good() ||
            !leg_data_lcm_->good())
        {
            RCLCPP_ERROR(rclcpp::get_logger("CyberSystem"), 
                        "Failed to initialize LCM communication!");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // 订阅 LCM 话题
        motor_data_lcm_->subscribe("spi_data", &CyberSystem::handleMotorData, this);
        motor_ctrl_state_lcm_->subscribe("motor_ctrl_state", 
                                         &CyberSystem::handleMotorCtrlState, this);
        leg_data_lcm_->subscribe("leg_control",
                                    &CyberSystem::handleLegData, this);
        robot_state_lcm_->subscribe("state_estimator", 
                                    &CyberSystem::handleRobotState, this);

  
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("CyberSystem"), 
                    "Exception during LCM setup: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("CyberSystem"), 
                "Successfully configured! LCM URLs:");
    RCLCPP_INFO(rclcpp::get_logger("CyberSystem"), 
                "  - Motor data/ctrl: %s", getLcmUrl(7667, 255).c_str());
    RCLCPP_INFO(rclcpp::get_logger("CyberSystem"), 
                "  - Robot state:     %s", getLcmUrl(7669, 255).c_str());
    RCLCPP_INFO(rclcpp::get_logger("CyberSystem"), 
                "Waiting for robot LCM communication...");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CyberSystem::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("CyberSystem"), "Activating...");

    // 启动 LCM 接收线程
    running_ = true;
    first_run_ = true;
    startup_count_ = 0;  // 重置启动计数器
    comm_lost_.store(true);  // 重置通信状态
    first_motor_ctrl_state_received_.store(false);  // 重置首次收到消息标志
    // 启动 LCM 线程
    startLcmThreads();

    RCLCPP_INFO(rclcpp::get_logger("CyberSystem"), 
                "Successfully activated! Waiting for robot communication...");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CyberSystem::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("CyberSystem"), "Deactivating...");

    // 停止 LCM 线程
    stopLcmThreads();

    // 发送零命令
    memset(&motor_cmd_, 0, sizeof(motor_cmd_));
    sendMotorCommand();

    RCLCPP_INFO(rclcpp::get_logger("CyberSystem"), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CyberSystem::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("CyberSystem"), "Cleaning up...");
    
    motor_data_lcm_.reset();
    motor_ctrl_state_lcm_.reset();
    robot_state_lcm_.reset();
    motor_ctrl_lcm_.reset();
    leg_data_lcm_.reset();

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CyberSystem::on_shutdown(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("CyberSystem"), "Shutting down...");
    stopLcmThreads();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CyberSystem::on_error(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_ERROR(rclcpp::get_logger("CyberSystem"), "Error occurred!");
    stopLcmThreads();
    
    // 发送零命令确保安全
    memset(&motor_cmd_, 0, sizeof(motor_cmd_));
    if (motor_ctrl_lcm_ && motor_ctrl_lcm_->good())
    {
        sendMotorCommand();
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CyberSystem::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // 导出关节状态接口
    for (auto & joint : joints_)
    {
        state_interfaces.emplace_back(
            joint.name, hardware_interface::HW_IF_POSITION, &joint.position);
        state_interfaces.emplace_back(
            joint.name, hardware_interface::HW_IF_VELOCITY, &joint.velocity);
        state_interfaces.emplace_back(
            joint.name, hardware_interface::HW_IF_EFFORT, &joint.effort);
    }

    // 导出 IMU 状态接口
    // 注意：LCM 消息中 quat 顺序是 [w, x, y, z]
    // ros2_control 接口按 [w, x, y, z] 顺序导出
    state_interfaces.emplace_back(
        imu_data_.name, "orientation.w", &imu_data_.orientation[0]);
    state_interfaces.emplace_back(
        imu_data_.name, "orientation.x", &imu_data_.orientation[1]);
    state_interfaces.emplace_back(
        imu_data_.name, "orientation.y", &imu_data_.orientation[2]);
    state_interfaces.emplace_back(
        imu_data_.name, "orientation.z", &imu_data_.orientation[3]);
    state_interfaces.emplace_back(
        imu_data_.name, "angular_velocity.x", &imu_data_.angular_velocity[0]);
    state_interfaces.emplace_back(
        imu_data_.name, "angular_velocity.y", &imu_data_.angular_velocity[1]);
    state_interfaces.emplace_back(
        imu_data_.name, "angular_velocity.z", &imu_data_.angular_velocity[2]);
    state_interfaces.emplace_back(
        imu_data_.name, "linear_acceleration.x", &imu_data_.linear_acceleration[0]);
    state_interfaces.emplace_back(
        imu_data_.name, "linear_acceleration.y", &imu_data_.linear_acceleration[1]);
    state_interfaces.emplace_back(
        imu_data_.name, "linear_acceleration.z", &imu_data_.linear_acceleration[2]);

    // 导出足端力状态接口
    // 顺序与配置文件中 foot_force_interfaces 一致: FL, RL, FR, RR
    state_interfaces.emplace_back(
        foot_force_data_.name, "FL_foot_force", &foot_force_data_.forces[0]);
    state_interfaces.emplace_back(
        foot_force_data_.name, "RL_foot_force", &foot_force_data_.forces[1]);
    state_interfaces.emplace_back(
        foot_force_data_.name, "FR_foot_force", &foot_force_data_.forces[2]);
    state_interfaces.emplace_back(
        foot_force_data_.name, "RR_foot_force", &foot_force_data_.forces[3]);

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CyberSystem::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // 导出关节命令接口
    for (auto & joint : joints_)
    {
        command_interfaces.emplace_back(
            joint.name, hardware_interface::HW_IF_POSITION, &joint.position_cmd);
        command_interfaces.emplace_back(
            joint.name, hardware_interface::HW_IF_VELOCITY, &joint.velocity_cmd);
        command_interfaces.emplace_back(
            joint.name, hardware_interface::HW_IF_EFFORT, &joint.effort_cmd);
        command_interfaces.emplace_back(
            joint.name, "kp", &joint.kp_cmd);
        command_interfaces.emplace_back(
            joint.name, "kd", &joint.kd_cmd);
    }

    return command_interfaces;
}

hardware_interface::return_type CyberSystem::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 定期打印通信状态（每5秒一次，用于诊断）
    static int read_count = 0;
    read_count++;
    if (read_count % 2500 == 0)  // 500Hz * 5s = 2500
    {
        RCLCPP_INFO(rclcpp::get_logger("CyberSystem"),
                    "[LCM Status] err_flag: 0x%x, motor_ctrl_state_received: %s",
                    robot_state_.err_flag,
                    first_motor_ctrl_state_received_.load() ? "YES" : "NO");
                   
    }

    // 检查通信状态
    // 只在通信状态发生变化时打印一次（避免刷屏）
    // err_flag 的 bit1 (0x02) 表示通信丢失超过1000ms
    bool now_lost = (robot_state_.err_flag & 0x02) != 0;
    if (now_lost && !comm_lost_.load())
    {
        RCLCPP_WARN(rclcpp::get_logger("CyberSystem"),
                    "[LCM] Communication LOST! No data received for >1000ms. err_flag: 0x%x",
                    robot_state_.err_flag);
        comm_lost_.store(true);
    }
    else if (!now_lost && comm_lost_.load())
    {
        RCLCPP_INFO(rclcpp::get_logger("CyberSystem"),
                    "\033[32m[LCM] Communication CONNECTED! Robot is online. err_flag: 0x%x\033[0m",
                    robot_state_.err_flag);
        comm_lost_.store(false);
    }

    // 将 LCM 接收的数据更新到关节状态
    for (size_t i = 0; i < joints_.size() && i < 12; ++i)
    {
        joints_[i].position = static_cast<double>(robot_state_.q[i]);
        joints_[i].velocity = static_cast<double>(robot_state_.qd[i]);
        joints_[i].effort = static_cast<double>(robot_state_.tau[i]);
    }

    // 更新 IMU 数据
    imu_data_.orientation[0] = robot_state_.quat[0];
    imu_data_.orientation[1] = robot_state_.quat[1];
    imu_data_.orientation[2] = robot_state_.quat[2];
    imu_data_.orientation[3] = robot_state_.quat[3];
    imu_data_.angular_velocity[0] = robot_state_.omega[0];
    imu_data_.angular_velocity[1] = robot_state_.omega[1];
    imu_data_.angular_velocity[2] = robot_state_.omega[2];
    imu_data_.linear_acceleration[0] = robot_state_.acc[0];
    imu_data_.linear_acceleration[1] = robot_state_.acc[1];
    imu_data_.linear_acceleration[2] = robot_state_.acc[2];

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CyberSystem::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // We'll guard access to shared data with data_mutex_. To avoid deadlocks
    // when sendMotorCommand() also acquires the mutex, prepare data under
    // lock and call sendMotorCommand() after releasing the lock.
    bool need_send = false;
    bool need_error_return = false;
    bool need_zero_cmd = false;

    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // 启动稳定期：前 STARTUP_WAIT_COUNT 次循环发送零命令（但必须发送，否则实机不响应）
        if (startup_count_ <= STARTUP_WAIT_COUNT)
        {
            startup_count_++;
            memset(&motor_cmd_, 0, sizeof(motor_cmd_));
            if (startup_count_ == STARTUP_WAIT_COUNT)
            {
                RCLCPP_INFO(rclcpp::get_logger("CyberSystem"), 
                            "Startup wait period completed, ready to send motor commands.");
            }
            // 启动期间也必须发送零命令，否则实机不会响应状态数据
            need_send = true;
            need_zero_cmd = true;
            // 启动期间跳过后续检查，直接发送零命令
        }
        else
        {
            // 检查通信状态
            if (robot_state_.err_flag & 0x02)
            {
                // 只有在第一次发现通信丢失时，记录日志
                if (!comm_lost_.load())
                {
                    RCLCPP_WARN(rclcpp::get_logger("CyberSystem"),
                                "Communication lost over 1000ms! Please check the lcm communication and reset motor cmd!!! err_flag: 0x%x",
                                robot_state_.err_flag);
                    comm_lost_.store(true);
                }
                // 通信丢失时也必须持续发送零命令，等待实机恢复响应
                memset(&motor_cmd_, 0, sizeof(motor_cmd_));
                need_zero_cmd = true;
                need_send = true;
                first_run_ = true;
            }

            // 检查电机错误
            for (int i = 0; i < 12; ++i)
            {
                if (robot_state_.motor_flags[i] & 0x3FFFFFFF)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("CyberSystem"),
                                "Motor %d error: 0x%x! Robot need estop for safety!", i, robot_state_.motor_flags[i]);
                    memset(&motor_cmd_, 0, sizeof(motor_cmd_));
                    need_zero_cmd = true;
                    need_send = true;
                    need_error_return = true;
                    break;
                }
            }

            // 将命令接口数据写入电机命令（只有在没有致命错误时）
            if (!need_error_return && !need_zero_cmd)
            {
                for (size_t i = 0; i < joints_.size() && i < 12; ++i)
                {
                    motor_cmd_.q_des[i] = static_cast<float>(joints_[i].position_cmd);
                    motor_cmd_.qd_des[i] = static_cast<float>(joints_[i].velocity_cmd);
                    motor_cmd_.tau_des[i] = static_cast<float>(joints_[i].effort_cmd);
                    motor_cmd_.kp_des[i] = static_cast<float>(joints_[i].kp_cmd);
                    motor_cmd_.kd_des[i] = static_cast<float>(joints_[i].kd_cmd);
                }
                need_send = true;
            }
        }  // end of else (startup period completed)
    }

    // 发送命令（在未持有 data_mutex_ 时进行，以避免死锁）
    if (need_send)
    {
        sendMotorCommand();
        first_run_ = false;
    }

    if (need_error_return)
    {
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

void CyberSystem::startLcmThreads()
{
    motor_data_thread_ = std::thread(&CyberSystem::motorDataLcmThread, this);
    motor_ctrl_state_thread_ = std::thread(&CyberSystem::motorCtrlStateLcmThread, this);
    robot_state_thread_ = std::thread(&CyberSystem::robotStateLcmThread, this);
    leg_data_thread_ = std::thread(&CyberSystem::legDataLcmThread, this);
    // user_code_control_thread_ = std::thread(&CyberSystem::Control, this);
}

void CyberSystem::stopLcmThreads()
{
    running_ = false;

    if (motor_data_thread_.joinable())
        motor_data_thread_.join();
    if (motor_ctrl_state_thread_.joinable())
        motor_ctrl_state_thread_.join();
    if (robot_state_thread_.joinable())
        robot_state_thread_.join();
    if (leg_data_thread_.joinable())
        leg_data_thread_.join();
}

void CyberSystem::motorDataLcmThread()
{
    RCLCPP_INFO(rclcpp::get_logger("CyberSystem"), "[LCM Thread] motorDataLcmThread started");
    while (running_)
    {
        int ret = motor_data_lcm_->handleTimeout(1000);
        // ret > 0 表示收到了消息
        static int timeout_count = 0;
        if (ret == 0)
        {
            timeout_count++;
            if (timeout_count % 5 == 0)  // 每5秒超时打印一次
            {
                RCLCPP_DEBUG(rclcpp::get_logger("CyberSystem"), 
                            "[LCM Thread] motorData timeout (count: %d)", timeout_count);
            }
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("CyberSystem"), "[LCM Thread] motorDataLcmThread stopped");
}

void CyberSystem::motorCtrlStateLcmThread()
{
    RCLCPP_INFO(rclcpp::get_logger("CyberSystem"), "[LCM Thread] motorCtrlStateLcmThread started");
    while (running_)
    {
        motor_ctrl_state_lcm_->handleTimeout(1000);
    }
    RCLCPP_INFO(rclcpp::get_logger("CyberSystem"), "[LCM Thread] motorCtrlStateLcmThread stopped");
}

void CyberSystem::robotStateLcmThread()
{
    RCLCPP_INFO(rclcpp::get_logger("CyberSystem"), "[LCM Thread] robotStateLcmThread started");
    while (running_)
    {
        robot_state_lcm_->handleTimeout(1000);
    }
    RCLCPP_INFO(rclcpp::get_logger("CyberSystem"), "[LCM Thread] robotStateLcmThread stopped");
}

void CyberSystem::legDataLcmThread()
{
    RCLCPP_INFO(rclcpp::get_logger("CyberSystem"), "[LCM Thread] legDataLcmThread started");
    while (running_)
    {
        leg_data_lcm_->handleTimeout(1000);
    }
    RCLCPP_INFO(rclcpp::get_logger("CyberSystem"), "[LCM Thread] legDataLcmThread stopped");
}

void CyberSystem::handleLegData(
    const lcm::ReceiveBuffer* rbuf,
    const std::string& chan,
    const leg_control_data_lcmt* msg)
{
    // 如果需要使用 leg_control_data 中的数据，可以在这里处理
    // 目前留空，因为主要数据从 spi_data 获取
    ( void )rbuf;
    ( void )chan;
    (void)msg;
}

void CyberSystem::handleMotorData(
    const lcm::ReceiveBuffer* rbuf,
    const std::string& chan,
    const spi_data_t* msg)
{
    ( void )rbuf;
    ( void )chan;
    
    // 首次收到电机数据时打印日志
    static bool first_motor_data_received = false;
    if (!first_motor_data_received)
    {
        first_motor_data_received = true;
        RCLCPP_INFO(rclcpp::get_logger("CyberSystem"),
                    "\033[32m[LCM] First spi_data (motor state) message received!\033[0m");
    }
    
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 应用符号转换（根据实机 SDK）
    for (int i = 0; i < 4; ++i)
    {
        robot_state_.q[i * 3 + 0] = msg->q_abad[i] * joint_sign_[i * 3 + 0];
        robot_state_.q[i * 3 + 1] = msg->q_hip[i] * joint_sign_[i * 3 + 1];
        robot_state_.q[i * 3 + 2] = msg->q_knee[i] * joint_sign_[i * 3 + 2];
        robot_state_.qd[i * 3 + 0] = msg->qd_abad[i] * joint_sign_[i * 3 + 0];
        robot_state_.qd[i * 3 + 1] = msg->qd_hip[i] * joint_sign_[i * 3 + 1];
        robot_state_.qd[i * 3 + 2] = msg->qd_knee[i] * joint_sign_[i * 3 + 2];
        robot_state_.tau[i * 3 + 0] = msg->tau_abad[i] * joint_sign_[i * 3 + 0];
        robot_state_.tau[i * 3 + 1] = msg->tau_hip[i] * joint_sign_[i * 3 + 1];
        robot_state_.tau[i * 3 + 2] = msg->tau_knee[i] * joint_sign_[i * 3 + 2];
    }

    for (int i = 0; i < 12; ++i)
    {
        robot_state_.motor_flags[i] = msg->flags[i];
    }
}


void CyberSystem::handleMotorCtrlState(
    const lcm::ReceiveBuffer* rbuf,
    const std::string& chan,
    const motor_ctrl_state_lcmt* msg)
{
    ( void )rbuf;
    ( void )chan;
    
    // 首次收到消息时打印日志
    if (!first_motor_ctrl_state_received_.load())
    {
        first_motor_ctrl_state_received_.store(true);
        RCLCPP_INFO(rclcpp::get_logger("CyberSystem"),
                    "\033[32m[LCM] First motor_ctrl_state message received! Robot is responding. err_flag: 0x%x\033[0m",
                    msg->err_flag);
    }
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    robot_state_.err_flag = msg->err_flag;
    robot_state_.ctrl_topic_interval = msg->ctrl_topic_interval;
}

void CyberSystem::handleRobotState(
    const lcm::ReceiveBuffer* rbuf,
    const std::string& chan,
    const state_estimator_lcmt* msg)
{
    ( void )rbuf;
    ( void )chan;
    std::lock_guard<std::mutex> lock(data_mutex_);

    for (int i = 0; i < 4; ++i)
    {
        robot_state_.quat[i] = msg->quat[i];
    }
    for (int i = 0; i < 3; ++i)
    {
        robot_state_.rpy[i] = msg->rpy[i];
        robot_state_.omega[i] = msg->omegaBody[i];
        robot_state_.acc[i] = msg->aBody[i];
    }
}

void CyberSystem::sendMotorCommand()
{
    motor_ctrl_lcmt cmd;

    // 拷贝 motor_cmd_ 到本地副本以避免锁持有时间过长
    MotorCmd local_cmd;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        local_cmd = motor_cmd_;
    }

    // 应用符号转换
    for (int i = 0; i < 12; ++i)
    {
        cmd.q_des[i] = local_cmd.q_des[i] * joint_sign_[i];
        cmd.qd_des[i] = local_cmd.qd_des[i] * joint_sign_[i];
        cmd.kp_des[i] = local_cmd.kp_des[i];
        cmd.kd_des[i] = local_cmd.kd_des[i];
        cmd.tau_des[i] = local_cmd.tau_des[i] * joint_sign_[i];
    }

    // 调试：打印发送状态（每5秒一次）
    static int send_count = 0;
    send_count++;
    if (send_count % 2500 == 1)  // 第一次和每5秒打印一次
    {
        RCLCPP_INFO(rclcpp::get_logger("CyberSystem"),
                    "[LCM Send] Publishing motor_ctrl command (count: %d)", send_count);
    }

    motor_ctrl_lcm_->publish("motor_ctrl", &cmd);

}

}  // namespace cyber_quadruped_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    cyber_quadruped_hardware::CyberSystem,
    hardware_interface::SystemInterface)
