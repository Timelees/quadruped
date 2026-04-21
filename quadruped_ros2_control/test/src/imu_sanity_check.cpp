/**
 * IMU Sanity Check Tool (ROS 2 Version)
 * 依赖: rclcpp, sensor_msgs, Eigen3
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <Eigen/Dense>
#include <vector>
#include <numeric>
#include <cmath>
#include <memory>

using std::placeholders::_1;

class ImuSanityChecker : public rclcpp::Node {
public:
    ImuSanityChecker() : Node("imu_sanity_check"), sample_count_(0), is_finished_(false), last_time_seconds_(0.0) {
        
        // 声明参数，方便运行时修改 topic 名称
        // 运行命令示例: ros2 run <pkg> imu_sanity_check --ros-args -p imu_topic:=/my_imu
        this->declare_parameter<std::string>("imu_topic", "/imu_sensor_broadcaster/imu");
        std::string imu_topic = this->get_parameter("imu_topic").as_string();

        // 使用 SensorDataQoS 以获得最佳兼容性 (自动匹配 Best Effort 或 Reliable)
        auto qos = rclcpp::SensorDataQoS();
        
        sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, qos, std::bind(&ImuSanityChecker::imuCallback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Starting IMU Sanity Check...");
        RCLCPP_INFO(this->get_logger(), "Please keep the robot STATIC (Stationary) for 3 seconds.");
        RCLCPP_INFO(this->get_logger(), "Listening to topic: %s", imu_topic.c_str());
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        if (is_finished_) return;

        // 获取时间戳 (ROS2 方式)
        double current_time = rclcpp::Time(msg->header.stamp).seconds();

        // 1. 检查时间戳连续性
        if (last_time_seconds_ > 0.0) {
            double dt = current_time - last_time_seconds_;
            // 过滤掉极其微小的异常 dt (如果是同一个时间戳发了多次)
            if (dt > 1e-6) {
                dt_history_.push_back(dt);
            }
        }
        last_time_seconds_ = current_time;

        // 2. 存储数据 (ROS2 msg 使用 shared pointer)
        Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        Eigen::Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

        acc_samples_.push_back(acc);
        gyro_samples_.push_back(gyro);
        sample_count_++;

        // 采集约 200 帧 或 足够的数据后停止
        if (sample_count_ >= 300) {
            generateReport();
            is_finished_ = true;
            rclcpp::shutdown(); // 任务完成，关闭节点
        }
    }

    void generateReport() {
        RCLCPP_INFO(this->get_logger(), "\n\n================= IMU DATA 报告 =================\n");

        if (dt_history_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No valid dt samples collected. Check timestamp format.");
            return;
        }

        // --- 1. 频率分析 ---
        double sum_dt = std::accumulate(dt_history_.begin(), dt_history_.end(), 0.0);
        double mean_dt = sum_dt / dt_history_.size();
        double freq = 1.0 / mean_dt;

        double sq_sum_dt = 0.0;
        for(double dt : dt_history_) sq_sum_dt += (dt - mean_dt) * (dt - mean_dt);
        double std_dt = std::sqrt(sq_sum_dt / dt_history_.size());

        RCLCPP_INFO(this->get_logger(), "[Frequency] 平均频率: %.2f Hz (dt: %.5f s)", freq, mean_dt);
        RCLCPP_INFO(this->get_logger(), "[Frequency] Jitter (Std): %.5f s", std_dt);
        
        if (std_dt > 0.1 * mean_dt) {
            RCLCPP_WARN(this->get_logger(), "!!! WARNING: High Jitter! Timestamps are unstable.");
        }

        // --- 2. 加速度计分析 ---
        Eigen::Vector3d acc_sum = Eigen::Vector3d::Zero();
        for (const auto& a : acc_samples_) acc_sum += a;
        Eigen::Vector3d acc_mean = acc_sum / sample_count_;
        double g_norm = acc_mean.norm();

        RCLCPP_INFO(this->get_logger(), "[加速度计--Accel] Mean (X, Y, Z): %.3f, %.3f, %.3f", acc_mean.x(), acc_mean.y(), acc_mean.z());
        RCLCPP_INFO(this->get_logger(), "[加速度计--Accel] Norm (Gravity): %.3f", g_norm);

        if (g_norm > 9.0 && g_norm < 10.5) {
            RCLCPP_INFO(this->get_logger(), "[Unit Check] Accel looks like m/s^2 (Good).");
        } else if (g_norm > 0.9 && g_norm < 1.1) {
            RCLCPP_ERROR(this->get_logger(), "!!! ERROR: Accel looks like 'g'. Multiply raw data by 9.81.");
        } else {
            RCLCPP_WARN(this->get_logger(), "!!! WARNING: Unusual gravity magnitude.");
        }

        // --- 3. 陀螺仪分析 ---
        Eigen::Vector3d gyro_sum = Eigen::Vector3d::Zero();
        for (const auto& g : gyro_samples_) gyro_sum += g;
        Eigen::Vector3d gyro_bias = gyro_sum / sample_count_;

        RCLCPP_INFO(this->get_logger(), "[陀螺仪--Gyro] Mean Bias (X, Y, Z): %.4f, %.4f, %.4f", gyro_bias.x(), gyro_bias.y(), gyro_bias.z());

        if (gyro_bias.norm() > 0.5) {
            RCLCPP_WARN(this->get_logger(), "!!! WARNING: High Gyro Bias. Is unit 'deg/s'? If so, multiply by (PI/180).");
        } else {
            RCLCPP_INFO(this->get_logger(), "[Unit Check] Gyro looks like rad/s (Good).");
        }

        // --- 4. 噪声分析 ---
        Eigen::Vector3d acc_var_sum = Eigen::Vector3d::Zero();
        Eigen::Vector3d gyro_var_sum = Eigen::Vector3d::Zero();
        for (size_t i = 0; i < sample_count_; ++i) {
            Eigen::Vector3d acc_diff = acc_samples_[i] - acc_mean;
            Eigen::Vector3d gyro_diff = gyro_samples_[i] - gyro_bias;
            acc_var_sum += acc_diff.cwiseProduct(acc_diff);
            gyro_var_sum += gyro_diff.cwiseProduct(gyro_diff);
        }
        Eigen::Vector3d acc_std = (acc_var_sum / sample_count_).cwiseSqrt();
        Eigen::Vector3d gyro_std = (gyro_var_sum / sample_count_).cwiseSqrt();

        RCLCPP_INFO(this->get_logger(), "[噪声--Noise Std] Acc: %.4f, %.4f, %.4f", acc_std.x(), acc_std.y(), acc_std.z());
        RCLCPP_INFO(this->get_logger(), "[噪声--Noise Std] Gyro: %.4f, %.4f, %.4f", gyro_std.x(), gyro_std.y(), gyro_std.z());
        
        RCLCPP_INFO(this->get_logger(), "\n\n===========================================================\n");
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
    int sample_count_;
    bool is_finished_;
    double last_time_seconds_;
    std::vector<double> dt_history_;
    std::vector<Eigen::Vector3d> acc_samples_;
    std::vector<Eigen::Vector3d> gyro_samples_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuSanityChecker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}