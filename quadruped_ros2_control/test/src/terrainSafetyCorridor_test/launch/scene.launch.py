from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package = get_package_share_directory('terrain_safety_corridor_test')
    robot = get_package_share_directory('cyberdog_description')
    return LaunchDescription([
        DeclareLaunchArgument('scene', default_value='bar'),
        DeclareLaunchArgument('task', default_value=os.path.join(robot, 'config', 'ocs2', 'task.info')),
        DeclareLaunchArgument('gait', default_value=os.path.join(robot, 'config', 'ocs2', 'gait.info')),
        DeclareLaunchArgument('output', default_value='/tmp/terrain_corridor_swing'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='terrain_safety_corridor_test', executable='terrain_corridor_test',
             arguments=['--scene', LaunchConfiguration('scene'), '--task', LaunchConfiguration('task'),
                        '--gait', LaunchConfiguration('gait'), '--output', LaunchConfiguration('output'), '--publish'], output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(package, 'config', 'scene.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
