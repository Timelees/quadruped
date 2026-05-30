"""
用法二：在 Gazebo 仿真已启动后，于【另一个终端】叠加启动本 launch。

  终端 1:
    ros2 launch ocs2_quadruped_controller gazebo.launch.py

  终端 2（等 Gazebo、控制器、TF(odom->base) 就绪后；勿在终端 1 再执行任何 ros2 launch）:
    ros2 launch plot_terrain convex.launch.py

  控制器通过话题 planar_terrain 接收地形并计算 SDF（图层 smooth_planar）。
  换地形图（需与 Gazebo world 一致）:
    ros2 launch plot_terrain convex.launch.py terrain_name:=stepping_stones_90.png
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    terrain_name = context.launch_configurations.get('terrain_name', 'stepping_stones_90.png')
    use_sim_time = context.launch_configurations.get('use_sim_time', 'true').lower() in ('true', '1', 'yes')
    param_file = context.launch_configurations.get(
        'perception_parameter_file',
        get_package_share_directory('plot_terrain') + '/config/parameters.yaml',
    )

    return [
        Node(
            package='plot_terrain',
            executable='plot_terrain_ConvexPlaneDecomposition',
            name='plot_terrain_ConvexPlaneDecomposition',
            output='screen',
            parameters=[
                param_file,
                {
                    'use_sim_time': use_sim_time,
                    'terrain_name': terrain_name,
                },
            ],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use /clock from Gazebo (required when stacking on gazebo.launch.py)',
        ),
        DeclareLaunchArgument(
            'perception_parameter_file',
            default_value=get_package_share_directory('plot_terrain') + '/config/parameters.yaml',
            description='Path to perception parameter YAML file',
        ),
        DeclareLaunchArgument(
            'terrain_name',
            default_value='stepping_stones_90.png',
            description='Terrain PNG in plot_terrain/data/ (match Gazebo world)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
