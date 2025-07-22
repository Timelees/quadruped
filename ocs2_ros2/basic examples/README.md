# OCS2 Basic Examples
[![](http://i0.hdslb.com/bfs/archive/1bf12ba98ed8e7fe01594bb2a713ddafdd580488.jpg)](https://www.bilibili.com/video/BV12vv9eGEns/)

This folder contains basic examples for the OCS2 library.

## 1. [Double Integrator](https://leggedrobotics.github.io/ocs2/robotic_examples.html#double-integrator)

* build
```bash
cd ~/ros2_ws
colcon build --packages-up-to ocs2_double_integrator_ros --symlink-install
```
* run
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_double_integrator_ros double_integrator.launch.py
```


## 2. [Cartpole](https://leggedrobotics.github.io/ocs2/robotic_examples.html#cartpole)

* build
```bash
cd ~/ros2_ws
colcon build --packages-up-to ocs2_cartpole_ros --symlink-install
```
* run
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_cartpole_ros cartpole.launch.py
```


## 3. [Ballbot](https://leggedrobotics.github.io/ocs2/robotic_examples.html#ballbot)

* build
```bash
cd ~/ros2_ws
colcon build --packages-up-to ocs2_ballbot_ros --symlink-install
```
* run
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_ballbot_ros ballbot_ddp.launch.py
```

## 4. [Quadrotor](https://leggedrobotics.github.io/ocs2/robotic_examples.html#quadrotor)

* build
```bash
cd ~/ros2_ws
colcon build --packages-up-to ocs2_quadrotor_ros --symlink-install
```
* run
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_quadrotor_ros quadrotor.launch.py
```

## 5. [Mobile Manipulator](https://leggedrobotics.github.io/ocs2/robotic_examples.html#mobile-manipulator)

* build
```bash
cd ~/ros2_ws
colcon build --packages-up-to ocs2_mobile_manipulator_ros --symlink-install
```
* run Mabi-Mobile
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_mobile_manipulator_ros manipulator_mabi_mobile.launch.py
```
* run Kinova Jaco2
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_mobile_manipulator_ros manipulator_kinova_j2n6.launch.py
```
* run Franka Panda
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_mobile_manipulator_ros franka.launch.py
```
* run Willow Garage PR2
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_mobile_manipulator_ros manipulator_pr2.launch.py
```
* run Clearpath Ridgeback with UR-5
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_mobile_manipulator_ros manipulator_ridgeback_ur5.launch.py 
```


## 6. [Legged Robot](https://leggedrobotics.github.io/ocs2/robotic_examples.html#legged-robot)
* build
```bash
cd ~/ros2_ws
colcon build --packages-up-to ocs2_legged_robot_ros --symlink-install
```
* run
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_legged_robot_ros legged_robot_ddp.launch.py
```