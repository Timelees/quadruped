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
CyberDog 实机启动文件
使用 ros2_control 框架启动控制器
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_description_package = context.launch_configurations['robot_description_package']
    robot_description_file = context.launch_configurations['robot_description_file']
    use_rviz = context.launch_configurations['use_rviz']

    pkg_path = os.path.join(get_package_share_directory(robot_description_package))
    xacro_file = os.path.join(pkg_path, 'xacro', robot_description_file)
    robot_description = xacro.process_file(xacro_file).toxml()

    cyber_hardware_share = get_package_share_directory('cyber_quadruped_hardware')
    controller_config = os.path.join(cyber_hardware_share, 'config', 'cyber_controllers.yaml')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controller_config],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
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
        'visualize_ocs2.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    return [
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        delay_ocs2_controller_spawner,
        rviz_node,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_description_package',
            default_value='cyberdog_description',
            description='Package containing the robot URDF'
        ),
        DeclareLaunchArgument(
            'robot_description_file',
            default_value='robot.xacro',
            description='URDF/XACRO file'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to start RViz'
        ),
    ]
    return LaunchDescription(
        declared_arguments + [
            OpaqueFunction(function=launch_setup)
        ]
    )
