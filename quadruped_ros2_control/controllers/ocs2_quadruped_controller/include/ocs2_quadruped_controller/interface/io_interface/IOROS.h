
#ifndef IOROS_H
#define IOROS_H

#include "rclcpp/rclcpp.hpp"
#include "ocs2_quadruped_controller/interface/io_interface/IOInterface.h"
#include "control_input_msgs/msg/legged_command.hpp"
#include "control_input_msgs/msg/sim_imu.hpp"
#include "control_input_msgs/msg/sim_joint.hpp"

#include <string>

class IOROS : public IOInterface{
public:
    IOROS();
    ~IOROS();
    void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);
    void recvUserParas(User_Parameters *paras);
private:
    void sendCmd(const LowlevelCmd *cmd);
    void recvState(LowlevelState *state);

    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<control_input_msgs::msg::SimJoint>::SharedPtr state_sub;
    rclcpp::Subscription<control_input_msgs::msg::SimImu>::SharedPtr imu_sub;
    rclcpp::Publisher<control_input_msgs::msg::LeggedCommand>::SharedPtr cmd_pub;

    control_input_msgs::msg::LeggedCommand cmd;
    control_input_msgs::msg::SimImu imu;
    control_input_msgs::msg::SimJoint state;

    LowlevelState _lowState;
    //repeated functions for multi-thread
    void initRecv();
    void initSend();

    //Callback functions for ROS
    void imuCallback(const control_input_msgs::msg::SimImu & msg);
    void jointStateCallback(const control_input_msgs::msg::SimJoint & msg);

};


#endif //CYBER_GUIDE_IOROS_H
