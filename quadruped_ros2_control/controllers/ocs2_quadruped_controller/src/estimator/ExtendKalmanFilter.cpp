#include <ocs2_quadruped_controller/estimator/ExtendKalmanFilter.h>
#include <ocs2_core/misc/LoadData.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <utility>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
namespace ocs2::legged_robot {
    ExtendKalmanFilter::ExtendKalmanFilter(PinocchioInterface pinocchio_interface,
                                           CentroidalModelInfo info,
                                           const PinocchioEndEffectorKinematics &ee_kinematics,
                                           CtrlInterfaces &ctrl_component,
                                           const rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
        : StateEstimateBase(std::move(info), ctrl_component, node),
          pinocchio_interface_(std::move(pinocchio_interface)),
          ee_kinematics_(ee_kinematics.clone()),
          numContacts_(info_.numThreeDofContacts + info_.numSixDofContacts),
          dimContacts_(3 * numContacts_),
          numState_(6 + dimContacts_),
          numObserve_(2 * dimContacts_ + numContacts_) {
        
        logger_ = std::make_unique<quadruped::logs::LogSave>();
        // 记录日志
        if(is_logger_enabled_){
            
            logger_->setEnabled(true);  // 启用日志记录
            // 创建日志文件
            std::string timeStamp = logger_->timestampNow();
            std::string path = std::string("Mylogs/") + std::string("ExtendedKalmanFilter") + "_" + timeStamp + ".txt";
            logger_->setDefaultFilePath(path);
            if(!logger_->open()){
                RCLCPP_ERROR(node_->get_logger(), "Failed to open log file.");
            }
            logger_->appendLine("\n ExtendedKalman Filter Initialization:");
            logger_->appendLine("=============================================================================\n");

        }else{
            logger_->setEnabled(false);
        }

        // --- resize and initialize all dynamic containers early to avoid invalid memory access ---
        // state and covariance sizes
        x_pri.setZero(numState_);
        x_post.setZero(numState_);
        P_pri.setZero(numState_, numState_);
        P_post.setZero(numState_, numState_);
        A.setZero(numState_, numState_);
        B.setZero(numState_, 3);
        Q.setZero(numState_, numState_);

        // observation sizes
        y_actual.setZero(numObserve_);
        y_estimate.setZero(numObserve_);
        error_y.setZero(numObserve_);
        Serror_y.setZero(numObserve_);
        C.setZero(numObserve_, numState_);
        R.setZero(numObserve_, numObserve_);
        SC.setZero(numObserve_, numState_);
        K.setZero(numState_, numObserve_);

        // feet-related vectors
        feet_positions_.setZero(dimContacts_);
        feet_velocities_.setZero(dimContacts_);
        feet_heights_.setZero(numContacts_);

        eye3.setIdentity(3, 3);
        // 设置观测矩阵 C
        C.setZero(numObserve_, numState_);
        matrix_t c1(3, 6), c2(3, 6);        // 基座位置到足端位置映射、基座速度到足端速度映射
        c1 << matrix3_t::Identity(), matrix3_t::Zero();     
        c2 << matrix3_t::Zero(), matrix3_t::Identity();
        C.setZero(numObserve_, numState_);
        for (size_t i = 0; i < numContacts_; ++i) {
            C.block(3 * i, 0, 3, 6) = c1;                      // 位置
            C.block(3 * (numContacts_ + i), 0, 3, 6) = c2;     // 速度
            C(2 * dimContacts_ + i, 6 + 3 * i + 2) = 1.0;      // 足端z方向高度
        }
        C.block(0, 6, dimContacts_, dimContacts_) = -matrix_t::Identity(dimContacts_, dimContacts_);   // 足端位置修正

        if(is_logger_enabled_){
            logger_->appendLine("\n Initialization  C Matrix:");
            logger_->appendLine("----------C Matrix------------");
            logger_->appendData(C, numObserve_, numState_);
            logger_->appendLine("----------C Matrix------------");
        }



        // 预测噪声协方差矩阵 Q
        Q.setIdentity(numState_, numState_);
        Q.block(0, 0, 3, 3) = imu_process_noise_position_  * eye3;    // 位置预测噪声
        Q.block(3, 3, 3, 3) = imu_process_noise_velocity_  * eye3;    // 速度预测噪声
        for(size_t i = 0; i < numContacts_; ++i){
            Q.block(6 + 3 * i, 6 + 3 * i, 3, 3) = footProcessNoisePosition_ * eye3;     // 足端位置预测噪声
        }

        if(is_logger_enabled_){
            logger_->appendLine("\n Initialization  Q Matrix:");
            logger_->appendLine("----------Q Matrix------------");
            logger_->appendData(Q, numState_, numState_);
            logger_->appendLine("----------Q Matrix------------");
        }

        // 观测噪声协方差矩阵 R
        R.setIdentity(numObserve_, numObserve_);
        R.block(0, 0, dimContacts_, dimContacts_) = 
                        R.block(0, 0, dimContacts_, dimContacts_) * footSensorNoisePosition_;  // 足端位置观测噪声
        R.block(dimContacts_, dimContacts_, dimContacts_, dimContacts_) = 
                        R.block(dimContacts_, dimContacts_, dimContacts_, dimContacts_) * footSensorNoiseVelocity_;    // 足端速度观测噪声
        R.block(2 * dimContacts_, 2 * dimContacts_, numContacts_, numContacts_) =
                        R.block(2 * dimContacts_, 2 * dimContacts_, numContacts_, numContacts_) * footHeightSensorNoise_;  // 足端高度观测噪声


        if(is_logger_enabled_){
            logger_->appendLine("\n Initialization  R Matrix:");
            logger_->appendLine("----------R Matrix------------");
            logger_->appendData(R, numObserve_, numObserve_);
            logger_->appendLine("----------R Matrix------------");
        }

        // 设置状态转移矩阵A和控制输入矩阵B
        A.setIdentity(numState_, numState_);
        B.setZero(numState_, 3);


        ee_kinematics_->setPinocchioInterface(pinocchio_interface_);
        init_state();

        initPublishers();
    }

    void ExtendKalmanFilter::init_state(){
        // 初始化先验估计误差协方差矩阵 P_pri
        P_pri.setIdentity(numState_, numState_);
        P_pri = 3.0 * P_pri;

        const auto &model = pinocchio_interface_.getModel();
        auto &data = pinocchio_interface_.getData();
        size_t actuatedDofNum = info_.actuatedDofNum;

        vector_t qPino(info_.generalizedCoordinatesNum);    // 包含基座姿态（rbd_state_.head<3>）和关节角度
        vector_t vPino(info_.generalizedCoordinatesNum);    // 包含角速度（从 rbd_state_ 转换）和关节速度
        qPino.setZero();
        qPino.segment<3>(3) = rbd_state_.head<3>(); // Only set orientation, let position in origin.
        qPino.tail(actuatedDofNum) = rbd_state_.segment(6, actuatedDofNum);

        vPino.setZero();
        vPino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(qPino.segment<3>(3), rbd_state_.segment<3>(info_.generalizedCoordinatesNum));
        // Only set angular velocity, let linear velocity be zero
        vPino.tail(actuatedDofNum) = rbd_state_.segment(6 + info_.generalizedCoordinatesNum, actuatedDofNum);
    
        // 运动学计算
        forwardKinematics(model, data, qPino, vPino);
        updateFramePlacements(model, data);



        // 获取足端位置和速度
        const auto eePos = ee_kinematics_->getPosition(vector_t());
        const auto eeVel = ee_kinematics_->getVelocity(vector_t(), vector_t());


        // 初始化状态向量先验估计值x_pri
        x_pri.setZero(numState_);
        x_pri.segment(0, 3) = rbd_state_.segment(3, 3);      // base初始状态位置
        for(size_t i = 0; i < numContacts_; ++i){
            x_pri.segment(6 + 3 * i, 3) = eePos[i];         // 足端初始位置(世界坐标系下)
        }


        if(is_logger_enabled_){
            logger_->appendLine("\n Initialization  P_pri Matrix:");
            logger_->appendLine("----------P_pri Matrix------------");
            logger_->appendData(P_pri, numState_, numState_);
            logger_->appendLine("----------P_pri Matrix------------");

            logger_->appendLine("\n Initialization  qPino Vector:");
            logger_->appendLine("----------qPino Vector------------");
            logger_->appendData(qPino, info_.generalizedCoordinatesNum, 1);

            logger_->appendLine("\n Initialization  vPino Vector:");
            logger_->appendLine("----------vPino Vector------------");
            logger_->appendData(vPino, info_.generalizedCoordinatesNum, 1);
            logger_->appendLine("----------vPino Vector------------");

            logger_->appendLine("\n Initialization  x_pri Vector:");
            logger_->appendLine("----------x_pri Vector------------");

            logger_->appendLine("----------x_pri Vector------------");
        }
    }

    vector_t ExtendKalmanFilter::update(const rclcpp::Time &time, const rclcpp::Duration &period){
        updateJointStates();        // 获取关节角度和关节速度
        updateContact();            // 更新脚部接触状态
        updateImu();                // 获取IMU四元数姿态、线加速度和角速度


        scalar_t dt = period.seconds();
        A.block(0, 3, 3, 3) = dt * eye3;
        B.block(0, 0, 3, 3) = 0.5 * dt * dt * eye3;
        B.block(3, 0, 3, 3) = dt * eye3;

        // 定义输入向量u (IMU测量值)
        vector3_t g(0, 0, -9.81);  // 重力加速度向量
        vector3_t u = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)) * linear_accel_local_ + g;      // IMU测量值转换到世界坐标系下
        
        // 更新Q矩阵
        Q.block(0, 0, 3, 3) = imu_process_noise_position_ * (dt / 20.f) * eye3;
        Q.block(3, 3, 3, 3) = imu_process_noise_velocity_ * (dt * 9.81f / 20.f) * eye3;
       
        const auto &model = pinocchio_interface_.getModel();
        auto &data = pinocchio_interface_.getData();
        size_t actuatedDofNum = info_.actuatedDofNum;

        vector_t qPino(info_.generalizedCoordinatesNum);    // 包含基座姿态（rbd_state_.head<3>）和关节角度
        vector_t vPino(info_.generalizedCoordinatesNum);    // 包含角速度（从 rbd_state_ 转换）和关节速度
        qPino.setZero();
        qPino.segment<3>(3) = rbd_state_.head<3>(); // Only set orientation, let position in origin.
        qPino.tail(actuatedDofNum) = rbd_state_.segment(6, actuatedDofNum);

        vPino.setZero();
        vPino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
            qPino.segment<3>(3),
            rbd_state_.segment<3>(info_.generalizedCoordinatesNum));
        // Only set angular velocity, let linear velocity be zero
        vPino.tail(actuatedDofNum) = rbd_state_.segment(6 + info_.generalizedCoordinatesNum, actuatedDofNum);
        
        // 运动学计算
        forwardKinematics(model, data, qPino, vPino);
        updateFramePlacements(model, data);
        const auto eePos = ee_kinematics_->getPosition(vector_t());
        const auto eeVel = ee_kinematics_->getVelocity(vector_t(), vector_t());



        // 非接触状态时更新Q,R矩阵腿部部分的值
        for(size_t i = 0; i < numContacts_; ++i){
            bool isContact = contact_flag_[i];
            scalar_t high_suspect_number(high_suspect_number_); 
            // 处于非接触状态的脚，提高对应的预测噪声协方差，信任观测值
            Q.block(6 + 3 * i, 6 + 3 * i, 3, 3) = (isContact ? 1 : high_suspect_number) * dt * footProcessNoisePosition_ * eye3;        // 脚部位置预测噪声

            // 足端位置观测噪声
            R.block(i * 3, i * 3, 3, 3) = (isContact ? 1 : high_suspect_number) * footSensorNoisePosition_ * eye3;          // 足端位置观测噪声
            R.block(dimContacts_ + i * 3, dimContacts_ + i * 3, 3, 3)  = (isContact ? 1 : high_suspect_number) * footSensorNoiseVelocity_ * eye3;  // 足端速度观测噪声
            R(2 * dimContacts_ + i, 2 * dimContacts_ + i) = (isContact ? 1 : high_suspect_number) * footHeightSensorNoise_; // 足端高度观测噪声
        
            // 构造实际观测值y_actual
            feet_positions_.segment(3 * i, 3) = -eePos[i];
            feet_positions_.segment(3 * i, 3)[2] += foot_radius_;
            feet_velocities_.segment(3 * i, 3) = -eeVel[i];
        }

        // 预测步骤
        x_post = A * x_pri + B * u;        // 状态预测,计算后验估计值  (卡尔曼滤波公式1)
        P_post = A * P_pri * A.transpose() + Q;    // 先验误差协方差预测    （卡尔曼滤波公式2）

        // 估计的观测值
        y_estimate = C * x_post;

        // 构造实际观测值y_actual
        for(size_t i = 0; i < numContacts_; ++i){
            bool isContact = contact_flag_[i];      // 接触状态
            // 足端位置观测值
            y_actual.segment(i*3, 3) = feet_positions_.segment(i*3, 3);        
            // 足端速度观测值, 足端处于接触状态时，取计算出来的速度值，摆动相时，计算出来的速度不可靠，取基座的速度            
            y_actual.segment((numContacts_ + i)*3, 3) = (isContact ? feet_velocities_.segment(i*3, 3) : x_pri.segment(3, 3));         
            // 足端高度观测值
            y_actual(2 * dimContacts_ + i) = (isContact ? 0 : x_pri(2) + feet_positions_(2 * i));
        }

        // 残差协方差
        S = C * P_post * C.transpose() + R;
        S = 0.5 * (S + S.transpose());   // 保证对称性

        // kalman增益   卡尔曼滤波公式3
        error_y = y_actual - y_estimate;        // 观测误差
        Serror_y = S.fullPivHouseholderQr().solve(error_y);   // S^-1*error_y
        
        x_pri = x_post + P_post * C.transpose() * Serror_y;       // 后验估计值计算  卡尔曼滤波公式4

        SC = S.fullPivHouseholderQr().solve(C);         // S^-1*C
        P_pri = P_post - P_post * C.transpose() * SC * P_post;      // 误差协方差更新  卡尔曼滤波公式5
        P_pri = 0.5 * (P_pri + P_pri.transpose());   // 保证对称性

        // 减小位置漂移
        if(P_pri.block(0, 0, 2, 2).determinant() > 1e-6){
            // 将 x,y 与其余 16 个状态的协方差（交叉项）清零, 阻止这些状态通过协方差传播把误差传回位置
            P_pri.block(0, 2, 2, 16).setZero();
            P_pri.block(2, 0, 16, 2).setZero();
            P_pri.block(0, 0, 2, 2) /= 10.0;        // 缩小位置的不确定性，使滤波器更信任当前位置
        }

        // 更新估计值到rbd_state_
        updateLinear(x_pri.segment<3>(0), x_pri.segment<3>(3));

        auto odom = getOdomMsg();
        odom.header.stamp = time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base";
        publishMsgs(odom);

        return rbd_state_;
    }

    nav_msgs::msg::Odometry ExtendKalmanFilter::getOdomMsg() {
        nav_msgs::msg::Odometry odom;
        // 机器人在世界坐标系下的位置
        odom.pose.pose.position.x = x_pri.segment<3>(0)(0);
        odom.pose.pose.position.y = x_pri.segment<3>(0)(1);
        odom.pose.pose.position.z = x_pri.segment<3>(0)(2);
        // 世界坐标系下的方向
        odom.pose.pose.orientation.x = quat_.x();
        odom.pose.pose.orientation.y = quat_.y();
        odom.pose.pose.orientation.z = quat_.z();
        odom.pose.pose.orientation.w = quat_.w();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                // 位置协方差：取自卡尔曼协方差矩阵 P 的前3×3块
                odom.pose.covariance[i * 6 + j] = P_pri(i, j);
                // 方向协方差：来自外部预设值 orientationCovariance_
                odom.pose.covariance[6 * (3 + i) + (3 + j)] = orientationCovariance_(i * 3 + j);
            }
        }
        //  The twist in this message should be specified in the coordinate frame given by the child_frame_id: "base"
        vector_t twist = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)).transpose() * x_pri.segment<3>(3);
        // 机体坐标系中的线速度
        odom.twist.twist.linear.x = twist.x();
        odom.twist.twist.linear.y = twist.y();
        odom.twist.twist.linear.z = twist.z();
        // 机体坐标系中的角速度
        odom.twist.twist.angular.x = angular_vel_local_.x();
        odom.twist.twist.angular.y = angular_vel_local_.y();
        odom.twist.twist.angular.z = angular_vel_local_.z();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                // 线速度协方差：取自 P_pri 的速度部分 (3-5行/列)
                odom.twist.covariance[i * 6 + j] = P_pri.block<3, 3>(3, 3)(i, j);
                // 角速度协方差：来自预设值 angularVelCovariance_
                odom.twist.covariance[6 * (3 + i) + (3 + j)] = angularVelCovariance_(i * 3 + j);
            }
        }
        return odom;
    }

    void ExtendKalmanFilter::loadSettings(const std::string &task_file, const bool verbose) {
        boost::property_tree::ptree pt;
        read_info(task_file, pt);
        const std::string prefix = "kalmanFilter.";
        if (verbose) {
            std::cerr << "\n #### Kalman Filter Noise:";
            std::cerr << "\n #### =============================================================================\n";
        }
        loadData::loadPtreeValue(pt, is_logger_enabled_, prefix + "is_enabled_logger", verbose);        //  是否启用日志

        loadData::loadPtreeValue(pt, foot_radius_, prefix + "footRadius", verbose);
        loadData::loadPtreeValue(pt, imu_process_noise_position_, prefix + "imuProcessNoisePosition", verbose);
        loadData::loadPtreeValue(pt, imu_process_noise_velocity_, prefix + "imuProcessNoiseVelocity", verbose);
        loadData::loadPtreeValue(pt, footProcessNoisePosition_, prefix + "footProcessNoisePosition", verbose);
        loadData::loadPtreeValue(pt, footSensorNoisePosition_, prefix + "footSensorNoisePosition", verbose);
        loadData::loadPtreeValue(pt, footSensorNoiseVelocity_, prefix + "footSensorNoiseVelocity", verbose);
        loadData::loadPtreeValue(pt, footHeightSensorNoise_, prefix + "footHeightSensorNoise", verbose);
        loadData::loadPtreeValue(pt, high_suspect_number_, prefix + "highSuspectNumber", verbose);
    }

}