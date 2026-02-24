

#ifndef CYBER_QUADRUPED_HARDWARE__CYBER_SYSTEM_HPP_
#define CYBER_QUADRUPED_HARDWARE__CYBER_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
// ensure rclcpp::Clock is a complete type
#include "rclcpp/clock.hpp"


#include "leg_control_data_lcmt.hpp"
#include "motor_ctrl_lcmt.hpp"
#include "motor_ctrl_state_lcmt.hpp"
#include "spi_data_t.hpp"
#include "state_estimator_lcmt.hpp"

// LCM includes
#include <lcm/lcm-cpp.hpp>

namespace cyber_quadruped_hardware
{
    /**
     * @brief 机器人状态数据结构
     */
    struct RobotData
    {
        float q[12];               // 12个关节电机角度，弧度制，0:右前腿横滚髋关节(对应hip_joint)，1:右前腿大腿关节(对应thigh_joint)，2:右前腿膝关节(对应calf_joint)
                                   //  3/4/5:左前腿  6/7/8:右后腿  9/10/11
        float qd[12];              // 电机角速度，弧度制
        float tau[12];             // 电机扭矩 N.M
        float quat[4];             // 机身姿态四元数 xyzW，右手坐标系
        float rpy[3];              // 机身姿态横滚、俯仰、偏航角 弧度制
        float acc[3];              // 加速度计值
        float omega[3];            // 角速度计值
        float ctrl_topic_interval; // 控制帧接收时间间隔
        uint32_t motor_flags[12];  // 电机模式和报错信息
        int16_t err_flag;
    };

    /**
     * @brief 电机命令数据结构
     */
    struct MotorCmd
    {
        // 期望扭矩tau = tau_des + (q_des - q)*kp_des + (qd_des - qd)*kd_des
        float q_des[12];   // 12个关机电机期望角度，弧度制  0/3/6/9:-0.68~0.68  1/4:2.79
                           // ~ -1.32  7/10:3.14 ~ -0.97  2/5/8/11:-0.52 ~ -2.53
        float qd_des[12];  // 电机期望角速度，弧度制  +-12弧度/秒@24NM
        float kp_des[12];  // 电机位置控制比例系数 0~200
        float kd_des[12];  // 电机速度控制比例系数 0~10
        float tau_des[12]; // 电机期望前馈扭矩  +-12NM
    };
    /**
     * @brief CyberDog 四足机器人 ros2_control 硬件接口
     *
     * 该类实现了 ros2_control 的 SystemInterface，
     * 通过 LCM 与实机进行通信
     */
    class CyberSystem : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(CyberSystem)

        // 生命周期回调
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State &previous_state) override;

        // 导出接口
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        // 读写函数
        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        // LCM 通信相关
        std::string getLcmUrl(int64_t port, int64_t ttl);
        void startLcmThreads();
        void stopLcmThreads();

        // LCM 回调函数
        void handleMotorData(const lcm::ReceiveBuffer *rbuf,
                             const std::string &chan,
                             const spi_data_t *msg);
        void handleMotorCtrlState(const lcm::ReceiveBuffer *rbuf,
                                  const std::string &chan,
                                  const motor_ctrl_state_lcmt *msg);
        void handleRobotState(const lcm::ReceiveBuffer *rbuf,
                              const std::string &chan,
                              const state_estimator_lcmt *msg);
        void handleLegData(const lcm::ReceiveBuffer *rbuf,
                           const std::string &chan,
                           const leg_control_data_lcmt *msg);

        // LCM 线程函数
        void motorDataLcmThread();
        void motorCtrlStateLcmThread();
        void robotStateLcmThread();
        void legDataLcmThread();

        // 发送电机命令
        void sendMotorCommand();

        // 关节数据存储（与 StateInterface/CommandInterface 绑定）
        struct JointData
        {
            std::string name;
            // state interface
            double position = 0.0;
            double velocity = 0.0;
            double effort = 0.0;
            // command interface
            double position_cmd = 0.0;
            double velocity_cmd = 0.0;
            double effort_cmd = 0.0;
            double kp_cmd = 0.0;
            double kd_cmd = 0.0;
        };

        // IMU 数据存储
        struct ImuData
        {
            std::string name;
            double orientation[4] = {1, 0, 0, 0}; // w, x, y, z (与 LCM 消息 quat 顺序一致)
            double angular_velocity[3] = {0, 0, 0};
            double linear_acceleration[3] = {0, 0, 0};
        };

        // 足端力数据存储
        struct FootForceData
        {
            std::string name = "foot_force";
            double forces[4] = {0, 0, 0, 0};  // FL, RL, FR, RR (与配置文件顺序一致)
        };

        // 数据成员
        std::vector<JointData> joints_;
        ImuData imu_data_;
        FootForceData foot_force_data_;

        // LCM 实例
        std::unique_ptr<lcm::LCM> motor_data_lcm_;
        std::unique_ptr<lcm::LCM> motor_ctrl_state_lcm_;
        std::unique_ptr<lcm::LCM> robot_state_lcm_;
        std::unique_ptr<lcm::LCM> motor_ctrl_lcm_;
        std::unique_ptr<lcm::LCM> leg_data_lcm_;    

        // LCM 线程
        std::thread motor_data_thread_;
        std::thread motor_ctrl_state_thread_;
        std::thread robot_state_thread_;
        std::thread leg_data_thread_;
        std::thread user_code_control_thread_;

        // 线程控制
        std::atomic<bool> running_{false};
        std::mutex data_mutex_;

        // 机器人状态和命令（线程安全）
        RobotData robot_state_;
        MotorCmd  motor_cmd_;

        // 关节符号映射（根据你的实机 SDK）
        const int joint_sign_[12] = {1, -1, -1, 1, -1, -1, 1, -1, -1, 1, -1, -1};

        // 配置参数
        double loop_rate_ = 500.0;
        bool first_run_ = true;
        
        // 启动稳定期计数器（对应 MotorCtrl::sendCmd 中的 count <= 1000 逻辑）
        long long startup_count_ = 0;
        static constexpr long long STARTUP_WAIT_COUNT = 1000;  // 等待1000次循环后再发送实际命令

    // 持久化 clock，避免在宏调用中使用临时 clock 导致悬挂引用
    rclcpp::Clock clock_{RCL_STEADY_TIME};
    // 通信状态，true 表示当前认为通信丢失（初始化为 true，启动时认为未连接）
    std::atomic<bool> comm_lost_{true};
    // 首次收到 motor_ctrl_state 消息的标志
    std::atomic<bool> first_motor_ctrl_state_received_{false};
    };

} // namespace cyber_quadruped_hardware

#endif // CYBER_QUADRUPED_HARDWARE__CYBER_SYSTEM_HPP_
