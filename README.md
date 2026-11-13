# 环境
ubuntu22.04
ros2 humble + gazebo harmonic

- rosdep 

```
rosdep install --from-paths src --ignore-src -r -y
```

- 编译
```
colcon build --packages-up-to cyberdog_description keyboard_input --symlink-install
```

- Gazebo Harmonic Simulator

  [Install Gazebo Harmonic at Ubuntu 22.04](https://gazebosim.org/docs/harmonic/install_ubuntu/#binary-installation-on-ubuntu)
```
sudo apt-get install ros-humble-ros-gzharmonic

colcon build --packages-up-to gz_quadruped_hardware --symlink-install
```

- OCS2 编译

  安装OCS2 ROS2之前， 需先安装 [Pinocchio](https://stack-of-tasks.github.io/pinocchio/download.html). **不要使用 rosdep**!

  pinocchio 3.4版本可用

  ```bash
  cd ocs2_ros2
  git submodule update --init --recursive

  cd ..
  rosdep install --from-paths src --ignore-src -r -y
  ```

  ```
  colcon build --packages-up-to ocs2_quadruped_controller  --symlink-install
  ```

## MINVO 实验包

在工作区根目录构建并加载环境：

```bash
cd /home/lee/Code/quadruped_control_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to minvo_test --symlink-install
source install/setup.bash
cd src
ros2 launch minvo_test scene.launch.py scene:=pillars
```

`src/minvo_test` 是指向 `quadruped_ros2_control/test/src/minvo_test` 的符号链接，用于让 colcon 发现嵌套包，请保留。场景参数、数据导出、绘图和测试见 [minvo_test 使用说明](quadruped_ros2_control/test/src/minvo_test/README.md)。

# 启动

  ```
  ros2 launch ocs2_quadruped_controller gazebo.launch.py
  ```

  ```
  ros2 run keyboard_input keyboard_input
  ```
