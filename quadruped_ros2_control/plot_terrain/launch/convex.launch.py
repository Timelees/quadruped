from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    """

    TF树结构说明 (正确的层级关系):
    world -> map -> odom -> base -> body -> ...
    """
    terrain_name = context.launch_configurations.get('terrain_name', 'stepping_stones_90.png')
    
    nodes = []
    

    # 获取参数文件路径
    param_file = context.launch_configurations.get('perception_parameter_file', 
                    get_package_share_directory('plot_terrain') + '/config/parameters.yaml')
    
    # Define node with parameters
    convex_plane_decomposition_node = Node(
        package='plot_terrain',
        executable='plot_terrain_ConvexPlaneDecomposition',
        name='plot_terrain_ConvexPlaneDecomposition',
        namespace='',
        output='screen',
        parameters=[
            param_file,
            {
                # 可以通过launch参数覆盖terrain_name
                'terrain_name': terrain_name,
            }
        ],
        # 使用仿真时间
        arguments=['--ros-args', '--log-level', 'info'],
    )
    nodes.append(convex_plane_decomposition_node)
    
    return nodes

def generate_launch_description():
    # Declare launch arguments
    robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='cyberdog',
        description='Name of the robot'
    )
  
    description_name = DeclareLaunchArgument(
        'description_name',
        default_value='cyberdog_description',
        description='Description name'
    )
    
    perception_parameter_file = DeclareLaunchArgument(
        'perception_parameter_file',
        default_value=get_package_share_directory('plot_terrain') + '/config/parameters.yaml',
        description='Path to perception parameter YAML file'
    )
    
    
    # 地形文件名参数，方便测试不同地形
    terrain_name = DeclareLaunchArgument(
        'terrain_name',
        default_value='stepping_stones_90.png',
        description='Name of the terrain image file in plot_terrain/data/'
    )

    return LaunchDescription([
        robot_name,
        description_name,
        perception_parameter_file,
        terrain_name,
        OpaqueFunction(function=launch_setup)
    ])