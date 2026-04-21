#include "controller_common/common/leg.h"

#include <iostream>
#include <stdexcept>
#include <memory>

namespace ocs2::legged_robot
{

// ==================== QuadrupedLeg 实现 ====================

QuadrupedLeg::QuadrupedLeg(int leg_id, double abad_link_length, double hip_link_length,
                           double knee_link_length, const Vector3d& hip_to_body)
    : abad_link_length_(abad_link_length),
      hip_link_length_(hip_link_length),
      knee_link_length_(knee_link_length),
      hip_to_body_(hip_to_body)
{
    // 根据腿部ID确定侧摆方向
    // 左腿 (FL=0, RL=2): side_sign = -1
    // 右腿 (FR=1, RR=3): side_sign = +1
    if (leg_id == 0 || leg_id == 2)
    {
        side_sign_ = -1;
    }
    else if (leg_id == 1 || leg_id == 3)
    {
        side_sign_ = 1;
    }
    else
    {
        throw std::invalid_argument("Invalid leg ID: " + std::to_string(leg_id) + ". Must be 0-3.");
    }
}

QuadrupedLeg::Vector3d QuadrupedLeg::calcFootPosToHip(const Vector3d& q) const
{
    // q(0): abad 关节角度 (侧摆)
    // q(1): hip 关节角度 (大腿)
    // q(2): knee 关节角度 (小腿)

    const double l1 = side_sign_ * abad_link_length_;
    const double l2 = -hip_link_length_;
    const double l3 = -knee_link_length_;

    const double s1 = std::sin(q(0));
    const double s2 = std::sin(q(1));
    const double s3 = std::sin(q(2));

    const double c1 = std::cos(q(0));
    const double c2 = std::cos(q(1));
    const double c3 = std::cos(q(2));

    const double c23 = c2 * c3 - s2 * s3;  // cos(q2 + q3)
    const double s23 = s2 * c3 + c2 * s3;  // sin(q2 + q3)

    Vector3d pos;
    pos(0) = l3 * s23 + l2 * s2;
    pos(1) = -l3 * s1 * c23 + l1 * c1 - l2 * c2 * s1;
    pos(2) = l3 * c1 * c23 + l1 * s1 + l2 * c1 * c2;

    return pos;
}

QuadrupedLeg::Vector3d QuadrupedLeg::calcFootPosToBody(const Vector3d& q) const
{
    return hip_to_body_ + calcFootPosToHip(q);
}

QuadrupedLeg::Vector3d QuadrupedLeg::calcFootVel(const Vector3d& q, const Vector3d& qd) const
{
    return calcJacobian(q) * qd;
}

QuadrupedLeg::Vector3d QuadrupedLeg::calcInverseKinematics(const Vector3d& p, FrameType frame) const
{
    Vector3d pEe2H;
    if(frame == FrameType::HIP)
        pEe2H = p;
    else if(frame == FrameType::BODY)
        pEe2H = p - hip_to_body_;
    else{
        std::cout << "[ERROR] The frame of QuadrupedLeg::calcQ can only be HIP or BODY!" << std::endl;
        exit(-1);
    }

    float q1, q2, q3;
    Vector3d qResult;
    float px, py, pz;
    float b2y, b3z, b4z, a, b, c;

    px = pEe2H(0);
    py = pEe2H(1);
    pz = pEe2H(2);

    b2y = abad_link_length_ * side_sign_;
    b3z = -hip_link_length_;
    b4z = -knee_link_length_;
    a = abad_link_length_;
    c = sqrt(pow(px, 2) + pow(py, 2) + pow(pz, 2)); // whole length
    b = sqrt(pow(c, 2) - pow(a, 2)); // distance between shoulder and footpoint

    q1 = calcAbadAngle(py, pz, b2y);
    q3 = calcKneeAngle(b3z, b4z, b);
    q2 = calcHipAngle(q1, q3, px, py, pz, b3z, b4z);

    qResult(0) = q1;
    qResult(1) = q2;
    qResult(2) = q3;

    return qResult;
}

QuadrupedLeg::Vector3d QuadrupedLeg::calcInverseKinematicsDeriv(const Vector3d& q, const Vector3d& vEe) const
{
    return calcJacobian(q).inverse() * vEe;
}

QuadrupedLeg::Vector3d QuadrupedLeg::calcInverseKinematicsDeriv(const Vector3d& pEe, const Vector3d& vEe, FrameType frame) const
{
    Vector3d q = calcInverseKinematics(pEe, frame);
    return calcJacobian(q).inverse() * vEe;
}

QuadrupedLeg::Vector3d QuadrupedLeg::calcJointTorques(const Vector3d& q, const Vector3d& force) const
{
    return calcJacobian(q).transpose() * force;
}

QuadrupedLeg::Matrix3d QuadrupedLeg::calcJacobian(const Vector3d& q) const
{
    Matrix3d jaco;

    float l1 = abad_link_length_ * side_sign_;
    float l2 = -hip_link_length_;
    float l3 = -knee_link_length_;

    float s1 = std::sin(q(0));
    float s2 = std::sin(q(1));
    float s3 = std::sin(q(2));

    float c1 = std::cos(q(0));
    float c2 = std::cos(q(1));
    float c3 = std::cos(q(2));

    float c23 = c2 * c3 - s2 * s3;
    float s23 = s2 * c3 + c2 * s3;

    jaco(0, 0) = 0;
    jaco(1, 0) = -l3 * c1 * c23 - l2 * c1 * c2 - l1 * s1;
    jaco(2, 0) = -l3 * s1 * c23 - l2 * c2 * s1 + l1 * c1;
    jaco(0, 1) = l3 * c23 + l2 * c2;
    jaco(1, 1) = l3 * s1 * s23 + l2 * s1 * s2;
    jaco(2, 1) = -l3 * c1 * s23 - l2 * c1 * s2;
    jaco(0, 2) = l3 * c23;
    jaco(1, 2) = l3 * s1 * s23;
    jaco(2, 2) = -l3 * c1 * s23;

    return jaco;
}

float QuadrupedLeg::calcAbadAngle(float py, float pz, float l1) const
{
    float q1;
    float L = sqrt(pow(py,2)+pow(pz,2)-pow(l1,2));
    q1 = atan2(pz*l1+py*L, py*l1-pz*L);
    return q1;
}

float QuadrupedLeg::calcKneeAngle(float b3z, float b4z, float b) const
{
    float q3, temp;
    temp = (pow(b3z, 2) + pow(b4z, 2) - pow(b, 2))/(2*fabs(b3z*b4z));
    if(temp>1) temp = 1;
    if(temp<-1) temp = -1;
    q3 = acos(temp);
    q3 = -(M_PI - q3); //0~180
    return q3;
}

float QuadrupedLeg::calcHipAngle(float q1, float q3, float px, float py, float pz, float b3z, float b4z) const
{
    float q2, a1, a2, m1, m2;
    
    a1 = py*sin(q1) - pz*cos(q1);
    a2 = px;
    m1 = b4z*sin(q3);
    m2 = b3z + b4z*cos(q3);
    q2 = atan2(m1*a1+m2*a2, m1*a2-m2*a1);
    return q2;
}


// ==================== QuadrupedRobotModel 实现 ====================

QuadrupedRobotModel::QuadrupedRobotModel()
{
    // Cyberdog 的髋关节相对于机体坐标系的位置
    // 参考 URDF 或机器人参数
    // FL: 前左, FR: 前右, RL: 后左, RR: 后右
    const Vector3d hip_fl(0.164, 0.042, 0.0);
    const Vector3d hip_fr(0.164, -0.042, 0.0);
    const Vector3d hip_rl(-0.164, 0.042, 0.0);
    const Vector3d hip_rr(-0.164, -0.042, 0.0);

    legs_[0] = std::make_unique<CyberdogLeg>(0, hip_fr);
    legs_[1] = std::make_unique<CyberdogLeg>(1, hip_fl);
    legs_[2] = std::make_unique<CyberdogLeg>(2, hip_rr);
    legs_[3] = std::make_unique<CyberdogLeg>(3, hip_rl);
}

QuadrupedRobotModel::QuadrupedRobotModel(std::array<std::unique_ptr<QuadrupedLeg>, 4> legs)
    : legs_(std::move(legs))
{
}

std::array<QuadrupedRobotModel::Vector3d, 4> QuadrupedRobotModel::getFootPositions(
    const Eigen::Matrix<double, 12, 1>& joint_angles) const
{
    std::array<Vector3d, 4> positions;
    for (int i = 0; i < 4; ++i)
    {
        Vector3d q = joint_angles.segment<3>(i * 3);
        positions[i] = legs_[i]->calcFootPosToBody(q);
    }
    return positions;
}

std::array<QuadrupedRobotModel::Vector3d, 4> QuadrupedRobotModel::getFootVelocities(
    const Eigen::Matrix<double, 12, 1>& joint_angles,
    const Eigen::Matrix<double, 12, 1>& joint_velocities) const
{
    std::array<Vector3d, 4> velocities;
    for (int i = 0; i < 4; ++i)
    {
        Vector3d q = joint_angles.segment<3>(i * 3);
        Vector3d qd = joint_velocities.segment<3>(i * 3);
        velocities[i] = legs_[i]->calcFootVel(q, qd);
    }
    return velocities;
}

QuadrupedRobotModel::Vector3d QuadrupedRobotModel::getFootPos(int leg_id, const Vector3d& q) const
{
    if (leg_id < 0 || leg_id > 3)
    {
        throw std::invalid_argument("Invalid leg ID: " + std::to_string(leg_id));
    }
    return legs_[leg_id]->calcFootPosToBody(q);
}

QuadrupedRobotModel::Vector3d QuadrupedRobotModel::getFootVel(int leg_id, const Vector3d& q, const Vector3d& qd) const
{
    if (leg_id < 0 || leg_id > 3)
    {
        throw std::invalid_argument("Invalid leg ID: " + std::to_string(leg_id));
    }
    return legs_[leg_id]->calcFootVel(q, qd);
}

QuadrupedRobotModel::Matrix3d QuadrupedRobotModel::getJacobian(int leg_id, const Vector3d& q) const
{
    if (leg_id < 0 || leg_id > 3)
    {
        throw std::invalid_argument("Invalid leg ID: " + std::to_string(leg_id));
    }
    return legs_[leg_id]->calcJacobian(q);
}

} // namespace ocs2::legged_robot
