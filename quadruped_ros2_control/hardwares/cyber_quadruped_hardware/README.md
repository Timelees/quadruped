# CyberDog Quadruped Hardware Interface

这是一个基于 ros2_control 框架的 CyberDog 四足机器人硬件接口实现，用于将 OCS2 控制算法部署到实机。

网络路由设置
```
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev enp49s0
```

## 架构概述

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          实机部署架构                                         │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   ┌─────────────────┐                                                       │
│   │   CyberDog      │                                                       │
│   │   底层控制器     │ ◄──── LCM UDP 多播通信 ────►                           │
│   │   (运行在机载)   │                              │                        │
│   └─────────────────┘                              │                        │
│                                                    ▼                        │
│   ┌──────────────────────────────────────────────────────────────────┐     │
│   │                    cyber_quadruped_hardware                       │     │
│   │  ┌─────────────────────────────────────────────────────────────┐ │     │
│   │  │ CyberSystem (hardware_interface::SystemInterface)           │ │     │
│   │  │                                                             │ │     │
│   │  │  ┌─────────────┐     ┌──────────────┐     ┌─────────────┐  │ │     │
│   │  │  │ LCM 接收线程 │     │ 数据转换层   │     │ LCM 发送    │  │ │     │
│   │  │  │ spi_data    │────►│ RobotState  │     │ motor_ctrl  │  │ │     │
│   │  │  │ state_est   │     │ MotorCmd    │────►│             │  │ │     │
│   │  │  └─────────────┘     └──────────────┘     └─────────────┘  │ │     │
│   │  │          │                   │                    ▲        │ │     │
│   │  │          ▼                   ▼                    │        │ │     │
│   │  │  ┌───────────────────────────────────────────────────────┐ │ │     │
│   │  │  │     StateInterface          CommandInterface          │ │ │     │
│   │  │  │  - position (12 joints)    - position_cmd             │ │ │     │
│   │  │  │  - velocity (12 joints)    - velocity_cmd             │ │ │     │
│   │  │  │  - effort (12 joints)      - effort_cmd               │ │ │     │
│   │  │  │  - imu (10 values)         - kp_cmd, kd_cmd           │ │ │     │
│   │  │  └───────────────────────────────────────────────────────┘ │ │     │
│   │  └─────────────────────────────────────────────────────────────┘ │     │
│   └──────────────────────────────────────────────────────────────────┘     │
│                                        │                                    │
│                           export_state/command_interfaces                   │
│                                        ▼                                    │
│   ┌──────────────────────────────────────────────────────────────────┐     │
│   │                      Controller Manager                          │     │
│   │  ┌──────────────────────────────────────────────────────────┐   │     │
│   │  │                   Resource Manager                        │   │     │
│   │  │   管理所有 StateInterface / CommandInterface              │   │     │
│   │  └──────────────────────────────────────────────────────────┘   │     │
│   │                              │                                   │     │
│   │              ┌───────────────┴───────────────┐                  │     │
│   │              ▼                               ▼                  │     │
│   │  ┌─────────────────────┐     ┌─────────────────────────────┐   │     │
│   │  │ JointStateBroadcaster │   │ Ocs2QuadrupedController     │   │     │
│   │  │ (发布 /joint_states)  │   │ (OCS2 MPC + WBC 控制算法)   │   │     │
│   │  └─────────────────────┘     └─────────────────────────────┘   │     │
│   └──────────────────────────────────────────────────────────────────┘     │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## 数据流

### 读取数据 (read)
```
CyberDog 底层 → LCM → handleMotorData/handleRobotState → robot_state_ → joints_[i].position/velocity/effort → StateInterface → Controller
```

### 写入命令 (write)
```
Controller → CommandInterface → joints_[i].xxx_cmd → motor_cmd_ → sendMotorCommand() → LCM → CyberDog 底层
```

## 文件结构

```
cyber_quadruped_hardware/
├── CMakeLists.txt
├── package.xml
├── cyber_quadruped_hardware.xml          # pluginlib 插件描述
├── include/
│   └── cyber_quadruped_hardware/
│       └── cyber_system.hpp              # 硬件接口头文件
├── src/
│   └── cyber_system.cpp                  # 硬件接口实现
├── config/
│   ├── ros2_control_config.urdf.xacro   # URDF ros2_control 配置示例
│   └── cyber_controllers.yaml           # 控制器配置
└── launch/
    └── cyber_bringup.launch.py          # 启动文件
```

## 使用方法

### 1. 编译

```bash
cd ~/Code/quadruped_control_ws
colcon build --packages-select cyber_quadruped_hardware
source install/setup.bash
```

### 2. 配置 URDF

在你的机器人 URDF 文件中添加 ros2_control 配置。参考 `config/ros2_control_config.urdf.xacro`：

```xml
<ros2_control name="CyberQuadrupedSystem" type="system">
  <hardware>
    <plugin>cyber_quadruped_hardware/CyberSystem</plugin>
  </hardware>
  
  <!-- 关节配置 -->
  <joint name="FR_hip_joint">
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <command_interface name="effort"/>
    <command_interface name="kp"/>
    <command_interface name="kd"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  <!-- ... 其他 11 个关节 ... -->
  
  <!-- IMU 传感器 -->
  <sensor name="imu_sensor">
    <state_interface name="orientation.x"/>
    <!-- ... -->
  </sensor>
</ros2_control>
```

### 3. 启动

```bash
# 确保 CyberDog 底层已启动并正在发送 LCM 数据

# 启动控制系统
ros2 launch cyber_quadruped_hardware cyber_bringup.launch.py \
    robot_description_package:=your_robot_description \
    robot_description_file:=your_robot.urdf.xacro

# （可选）启动 RViz
ros2 launch cyber_quadruped_hardware cyber_bringup.launch.py use_rviz:=true
```

## 关节顺序

实机 SDK 的关节顺序（与 `joint_sign_` 数组对应）：

| Index | 关节名 | 位置 | 符号 |
|-------|--------|------|------|
| 0 | FR_hip_joint | 右前髋横摆 | +1 |
| 1 | FR_thigh_joint | 右前大腿 | -1 |
| 2 | FR_calf_joint | 右前小腿 | -1 |
| 3 | FL_hip_joint | 左前髋横摆 | +1 |
| 4 | FL_thigh_joint | 左前大腿 | -1 |
| 5 | FL_calf_joint | 左前小腿 | -1 |
| 6 | RR_hip_joint | 右后髋横摆 | +1 |
| 7 | RR_thigh_joint | 右后大腿 | -1 |
| 8 | RR_calf_joint | 右后小腿 | -1 |
| 9 | RL_hip_joint | 左后髋横摆 | +1 |
| 10 | RL_thigh_joint | 左后大腿 | -1 |
| 11 | RL_calf_joint | 左后小腿 | -1 |

**注意**: `joint_sign_` 用于处理实机关节方向与 URDF 模型的差异。

## LCM 通信

### 订阅的话题
- `spi_data` (端口 7667): 电机数据 (位置、速度、力矩)
- `motor_ctrl_state` (端口 7667): 控制状态
- `state_estimator` (端口 7669): IMU 和状态估计数据

### 发布的话题
- `motor_ctrl` (端口 7667): 电机控制命令

### 网络配置
LCM 使用 UDP 多播：`udpm://239.255.76.67:PORT?ttl=255`

确保网络接口支持多播，且防火墙允许 UDP 端口 7667 和 7669。

## 对比 Gazebo 仿真

| 功能 | Gazebo (gz_quadruped_hardware) | 实机 (cyber_quadruped_hardware) |
|------|--------------------------------|--------------------------------|
| 通信方式 | Gazebo ECM 组件 | LCM UDP 多播 |
| 数据读取 | `sim::components::JointPosition` | LCM 回调 `handleMotorData` |
| 命令发送 | `sim::components::JointForceCmd` | LCM `publish("motor_ctrl")` |
| 更新触发 | Gazebo 仿真步 | Controller Manager 周期 |

## 安全注意事项

1. **通信超时检测**: 如果 `err_flag & 0x02` 表示通信丢失，将发送零命令
2. **电机错误检测**: 检查 `motor_flags` 的错误位
3. **急停**: 出错时自动发送零命令
4. **关节限位**: 在控制器层面处理，硬件接口不做限位检查

## 调试

```bash
# 查看可用控制器
ros2 control list_controllers

# 查看硬件接口状态
ros2 control list_hardware_interfaces

# 查看关节状态
ros2 topic echo /joint_states

# 切换控制器
ros2 control switch_controllers --start ocs2_quadruped_controller --stop ...
```

## TODO

- [ ] 添加足端力传感器接口
- [ ] 添加更完善的错误恢复机制
- [ ] 支持配置文件动态加载 LCM 参数
- [ ] 添加数据记录功能
