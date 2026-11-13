# CyberDog Quadruped Hardware Interface
基于 ros2_control 框架的 CyberDog 四足机器人硬件接口实现，用于将 OCS2 控制算法部署到实机。

路由查看
```
route -n
```

如：
```
内核 IP 路由表
目标            网关            子网掩码        标志  跃点   引用  使用 接口
0.0.0.0         192.168.31.1    0.0.0.0         UG    600    0        0 wlp0s20f3
169.254.0.0     0.0.0.0         255.255.0.0     U     1000   0        0 wlp0s20f3
192.168.31.0    0.0.0.0         255.255.255.0   U     600    0        0 wlp0s20f3
192.168.44.0    0.0.0.0         255.255.255.0   U     100    0        0 enp49s0

```

网络路由设置
```
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev enp49s0
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
colcon build --packages-up-to cyber_quadruped_hardware --symlink-install
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
ros2 launch cyber_quadruped_hardware cyber_bringup.launch.py
```

## 感知模式的启动

机器人实机部分需要开启/cyberdog_1/camera/depth/color/points话题

```
ros2 lifecycle set /cyberdog_1/camera/camera_align configure
ros2 lifecycle set /cyberdog_1/camera/camera_align activate

ros2 service call /cyberdog_1/camera/realsense_frame_service   std_srvs/srv/SetBool "{data: true}"

```

## 实机话题收取

要获取实机的话题数据，需要执行以下

```
### ROS2 ###
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/lee/.cyclonedds.xml		# 更改为实际的路径
export ROS_DOMAIN_ID=42
```

注意：键盘控制的节点启动终端也要执行上述内容，否则无法控制





## LCM 通信

### 订阅的话题
- `spi_data` (端口 7667): 电机数据 (位置、速度、力矩)
- `motor_ctrl_state` (端口 7667): 控制状态
- `state_estimator` (端口 7669): IMU 和状态估计数据

### 发布的话题
- `motor_ctrl` (端口 7667): 电机控制命令




