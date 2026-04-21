#pragma once

#include <eigen3/Eigen/Dense>
#include <array>
#include <cmath>
#include <memory>
#include "controller_common/common/enumClass.h"

namespace ocs2::legged_robot
{

/**
 * @brief 腿部运动学计算类
 * 
 * 参考 unitree_guide 的实现，使用解析几何方法计算足端位置、速度和雅可比矩阵
 * 适用于四足机器人的单腿运动学计算
 */
class QuadrupedLeg
{
public:
    using Vector3d = Eigen::Vector3d;
    using Matrix3d = Eigen::Matrix3d;

    /**
     * @brief 构造函数
     * 
     * @param leg_id 腿部ID (0: FL, 1: FR, 2: RL, 3: RR)
     * cyberdog的顺序(0: FR, 1: FL, 2: RR, 3: RL)
     * @param abad_link_length 髋关节连杆长度 (侧摆关节到大腿关节)
     * @param hip_link_length 大腿连杆长度
     * @param knee_link_length 小腿连杆长度
     * @param hip_to_body 髋关节相对于机体坐标系的位置
     */
    QuadrupedLeg(int leg_id, double abad_link_length, double hip_link_length,
                 double knee_link_length, const Vector3d& hip_to_body);

    ~QuadrupedLeg() = default;

    /**
     * @brief 正向运动学: 计算足端相对于髋关节的位置
     * 
     * @param q 关节角度 [abad, hip, knee]
     * @return 足端相对于髋关节的位置
     */
    Vector3d calcFootPosToHip(const Vector3d& q) const;

    /**
     * @brief 正向运动学: 计算足端相对于机体坐标系的位置
     * 
     * @param q 关节角度 [abad, hip, knee]
     * @return 足端相对于机体坐标系的位置
     */
    Vector3d calcFootPosToBody(const Vector3d& q) const;

    /**
     * @brief 计算足端速度
     * 
     * @param q 关节角度 [abad, hip, knee]
     * @param qd 关节角速度 [abad_dot, hip_dot, knee_dot]
     * @return 足端速度 (机体坐标系)
     */
    Vector3d calcFootVel(const Vector3d& q, const Vector3d& qd) const;

     /**
     * @brief 逆向运动学：计算足端坐标为p时，腿上3个关节的角度，frame表示p所在的坐标系
     * 
     * @param p 足端在指定坐标系中的位置
     * @param frame 指定坐标系
     * @return 关节角度 [abad, hip, knee]
     */
    Vector3d calcInverseKinematics(const Vector3d& p, FrameType frame) const;

    /**
     * @brief 逆向运动学求导：根据关节角度q和足端速度vEe，计算关节角速度
     * 
     * @param q 关节角度 [abad, hip, knee]
     * @param vEe 足端速度 (机体坐标系)
     * @return 关节角速度 [abad_dot, hip_dot, knee_dot]
     */
    Vector3d calcInverseKinematicsDeriv(const Vector3d& q, const Vector3d& vEe) const;

    /**
     * @brief 逆向运动学求导：根据足端在frame坐标系下的位置坐标pEe和速度vEe，计算关节角速度
     * 
     * @param pEe 足端在指定坐标系中的位置
     * @param vEe 足端速度 (机体坐标系)
     * @param frame 指定坐标系
     * @return 关节角速度 [abad_dot, hip_dot, knee_dot]
     */
    Vector3d calcInverseKinematicsDeriv(const Vector3d& pEe, const Vector3d& vEe, FrameType frame) const;


    /**
     * @brief 根据关节角度q和足端力force计算关节力矩
     * 
     * @param q 足端在指定坐标系中的位置
     * @param force 指定坐标系
     * @return 关节力矩 [abad_torque, hip_torque, knee_torque]
     */
    Vector3d calcJointTorques(const Vector3d& q, const Vector3d& force) const;


    /**
     * @brief 计算雅可比矩阵 (足端速度对关节角速度的偏导)
     * 
     * @param q 关节角度 [abad, hip, knee]
     * @return 3x3 雅可比矩阵
     */
    Matrix3d calcJacobian(const Vector3d& q) const;

    /**
     * @brief 获取髋关节相对于机体坐标系的位置
     * 
     * @return 髋关节位置
     */
    Vector3d getHipToBody() const { return hip_to_body_; }

    /**
     * @brief 获取侧摆标志 (左腿为-1, 右腿为1)
     */
    int getSideSign() const { return side_sign_; }

    /**
     * @brief 侧摆关节计算
     * @param py 足端位置的y坐标
     * @param pz 足端位置的z坐标
     * @param l1 abad连杆的长度
     * @return abad关节角度
     */
    float calcAbadAngle(float py, float pz, float l1) const;


    /**
     * @brief 小腿关节计算
     * @param b3z hip连杆的长度
     * @param b4z knee连杆的长度
     * @param b distance between shoulder and footpoint
     * @return knee关节角度
     */
    float calcKneeAngle(float b3z, float b4z, float b) const;


    /**
     * @brief 大腿关节计算
     * @param q1 hip关节的角度
     * @param q3 knee关节的角度
     * @param px 足端位置的x坐标
     * @param py 足端位置的y坐标
     * @param pz 足端位置的z坐标
     * @param b3z hip连杆的长度
     * @param b4z knee连杆的长度
     * @return hip关节角度
     */
    float calcHipAngle(float q1, float q3, float px, float py, float pz, float b3z, float b4z) const;

protected:
    int side_sign_;                 // 侧摆标志: 左腿(-1), 右腿(+1)
    double abad_link_length_;       // 髋关节连杆长度
    double hip_link_length_;        // 大腿连杆长度
    double knee_link_length_;       // 小腿连杆长度
    Vector3d hip_to_body_;          // 髋关节相对于机体坐标系的位置
};


/**
 * @brief Cyberdog 机器人腿部运动学类
 * 
 * Cyberdog 的连杆参数: abad=0.094, hip=0.12, knee=0.12
 */
class CyberdogLeg : public QuadrupedLeg
{
public:
    /**
     * @brief 构造函数
     * 
     * @param leg_id 腿部ID (0: FL, 1: FR, 2: RL, 3: RR)
     * @param hip_to_body 髋关节相对于机体坐标系的位置
     */
    CyberdogLeg(int leg_id, const Vector3d& hip_to_body)
        : QuadrupedLeg(leg_id, 0.094, 0.12, 0.12, hip_to_body) {}

    ~CyberdogLeg() = default;
};



/**
 * @brief 四足机器人运动学模型
 * 
 * 管理四条腿的运动学计算，提供统一的接口
 */
class QuadrupedRobotModel
{
public:
    using Vector3d = Eigen::Vector3d;
    using Matrix3d = Eigen::Matrix3d;

    /**
     * @brief 默认构造函数 
     */
    QuadrupedRobotModel();

    /**
     * @brief 构造函数
     * 
     * @param legs 四条腿的指针数组
     */
    QuadrupedRobotModel(std::array<std::unique_ptr<QuadrupedLeg>, 4> legs);

    ~QuadrupedRobotModel() = default;

    /**
     * @brief 获取所有足端相对于机体坐标系的位置
     * 
     * @param joint_angles 所有关节角度 (12维: FL[3], FR[3], RL[3], RR[3])
     * @return 四个足端的位置
     */
    std::array<Vector3d, 4> getFootPositions(const Eigen::Matrix<double, 12, 1>& joint_angles) const;

    /**
     * @brief 获取所有足端速度
     * 
     * @param joint_angles 所有关节角度 (12维)
     * @param joint_velocities 所有关节角速度 (12维)
     * @return 四个足端的速度
     */
    std::array<Vector3d, 4> getFootVelocities(const Eigen::Matrix<double, 12, 1>& joint_angles,
                                              const Eigen::Matrix<double, 12, 1>& joint_velocities) const;

    /**
     * @brief 获取指定腿的足端位置
     * 
     * @param leg_id 腿部ID
     * @param q 该腿的关节角度 (3维)
     * @return 足端相对于机体坐标系的位置
     */
    Vector3d getFootPos(int leg_id, const Vector3d& q) const;

    /**
     * @brief 获取指定腿的足端速度
     * 
     * @param leg_id 腿部ID
     * @param q 该腿的关节角度 (3维)
     * @param qd 该腿的关节角速度 (3维)
     * @return 足端速度
     */
    Vector3d getFootVel(int leg_id, const Vector3d& q, const Vector3d& qd) const;

    /**
     * @brief 获取指定腿的雅可比矩阵
     * 
     * @param leg_id 腿部ID
     * @param q 该腿的关节角度 (3维)
     * @return 3x3 雅可比矩阵
     */
    Matrix3d getJacobian(int leg_id, const Vector3d& q) const;

private:
    std::array<std::unique_ptr<QuadrupedLeg>, 4> legs_;
};

} // namespace ocs2::legged_robot
