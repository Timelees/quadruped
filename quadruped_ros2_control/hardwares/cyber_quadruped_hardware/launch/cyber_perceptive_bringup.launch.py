# Copyright 2024 Your Name
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
CyberDog 实机感知模式启动文件
启动高程图建图、凸平面分解以及 OCS2 感知控制器。
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_description_package = context.launch_configurations['robot_description_package']
    robot_description_file = context.launch_configurations['robot_description_file']
    use_rviz = context.launch_configurations['use_rviz']
    use_sim_time = context.launch_configurations['use_sim_time']
    publish_real_base_link_bridge = LaunchConfiguration('publish_real_base_link_bridge')
    controller_base_frame = LaunchConfiguration('controller_base_frame')
    real_base_frame = LaunchConfiguration('real_base_frame')
    use_pointcloud_qos_relay = LaunchConfiguration('use_pointcloud_qos_relay')
    real_pointcloud_topic = LaunchConfiguration('real_pointcloud_topic')
    perception_pointcloud_topic = LaunchConfiguration('perception_pointcloud_topic')

    pkg_path = os.path.join(get_package_share_directory(robot_description_package))
    xacro_file = os.path.join(pkg_path, 'xacro', robot_description_file)
    robot_description = xacro.process_file(xacro_file).toxml()

    cyber_hardware_share = get_package_share_directory('cyber_quadruped_hardware')
    controller_config = os.path.join(
        cyber_hardware_share, 'config', 'cyber_controllers_perceptive.yaml'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time == 'true',
        }]
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': use_sim_time == 'true'},
            controller_config,
        ],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    imu_sensor_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    ocs2_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ocs2_quadruped_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    delay_ocs2_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[ocs2_controller_spawner],
        )
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('ocs2_quadruped_controller'),
        'config',
        'visualize_ocs2_cyberdog.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz),
        output='screen',
        parameters=[{'use_sim_time': use_sim_time == 'true'}],
    )

    # 实机 TF 树以 base_link 为根挂载 camera_* 帧；控制器感知链路使用
    # odom -> base。这里仅做 base -> base_link 桥接，复用实机发布的相机 TF。
    real_base_link_bridge_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='real_base_link_bridge_tf_publisher',
        arguments=[
            '0.0', '0.0', '0.0',
            '0.0', '0.0', '0.0', '1.0',
            controller_base_frame,
            real_base_frame,
        ],
        condition=IfCondition(publish_real_base_link_bridge),
        output='screen',
    )

    pointcloud_qos_relay_node = Node(
        package='cyber_quadruped_hardware',
        executable='pointcloud_qos_relay',
        name='pointcloud_qos_relay',
        parameters=[{
            'input_topic': real_pointcloud_topic,
            'output_topic': perception_pointcloud_topic,
            'queue_size': 1,
        }],
        condition=IfCondition(use_pointcloud_qos_relay),
        output='screen',
    )

    # 感知层：高程图建图 + 凸平面分解
    elevation_mapping_config = os.path.join(
        get_package_share_directory('ocs2_quadruped_controller'),
        'config',
        'elevation_mapping_cyberdog.yaml'
    )
    elevation_mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ocs2_quadruped_controller'),
                'launch',
                'elevation_mapping.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'parameter_file': elevation_mapping_config,
        }.items(),
    )

    return [
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        imu_sensor_broadcaster_spawner,
        delay_ocs2_controller_spawner,
        rviz_node,
        real_base_link_bridge_tf_publisher,
        pointcloud_qos_relay_node,
        elevation_mapping_launch,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_description_package',
            default_value='cyberdog_description',
            description='Package containing the robot URDF',
        ),
        DeclareLaunchArgument(
            'robot_description_file',
            default_value='robot.xacro',
            description='URDF/XACRO file',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to start RViz',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock (false for real robot)',
        ),
        DeclareLaunchArgument(
            'publish_real_base_link_bridge',
            default_value='true',
            description='Publish controller base to real robot base_link static TF bridge',
        ),
        DeclareLaunchArgument(
            'controller_base_frame',
            default_value='base',
            description='Base frame used by the controller and elevation mapping',
        ),
        DeclareLaunchArgument(
            'real_base_frame',
            default_value='base_link',
            description='Root frame of the real CyberDog TF subtree',
        ),
        DeclareLaunchArgument(
            'use_pointcloud_qos_relay',
            default_value='true',
            description='Relay real best-effort point cloud to a local reliable topic',
        ),
        DeclareLaunchArgument(
            'real_pointcloud_topic',
            default_value='/cyberdog_1/camera/depth/color/points',
            description='Best-effort point cloud topic published by the real robot',
        ),
        DeclareLaunchArgument(
            'perception_pointcloud_topic',
            default_value='/cyberdog_1/perception/depth/color/points_reliable',
            description='Reliable local point cloud topic consumed by RViz and elevation mapping',
        ),
    ]

    return LaunchDescription(
        declared_arguments + [
            OpaqueFunction(function=launch_setup),
        ]
    )
