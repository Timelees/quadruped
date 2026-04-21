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

# 启动

  ```
  ros2 launch ocs2_quadruped_controller gazebo.launch.py
  ```

  ```
  ros2 run keyboard_input keyboard_input
  ```