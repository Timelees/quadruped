"""Launch an interactive production MINVO scene with an editable YAML fixture."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('minvo_test'))
    source = Path.cwd() / 'quadruped_ros2_control/test/src/minvo_test'
    root = source if (source / 'package.xml').exists() else share
    scene = LaunchConfiguration('scene')
    return LaunchDescription([
        DeclareLaunchArgument('scene', default_value='single', description='single, pillars, stairs'),
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([str(root / 'config'), [scene, '.yaml']])),
        DeclareLaunchArgument('data_dir', default_value=PathJoinSubstitution([str(root / 'data'), scene])),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='minvo_test', executable='minvo_scene', name='minvo_scene',
             arguments=[LaunchConfiguration('config'), LaunchConfiguration('data_dir')], output='screen'),
        Node(package='rviz2', executable='rviz2', name='minvo_scene_rviz',
             arguments=['-d', str(root / 'config/scene.rviz')], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
