from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    """
    条件启动函数：根据参数决定启动配置
    
    TF树结构说明 (正确的层级关系):
    world -> map -> odom -> base -> body -> ...
    
    - Gazebo/ros_gz_bridge 发布: odom -> base
    - robot_state_publisher 发布: base -> body -> 各关节
    - 需要的静态变换: world -> map, map -> odom (当不与gazebo一起运行时)
    
    当与gazebo.launch.py一起运行时:
    - gazebo.launch.py 已经发布了 world->map 和 map->odom
    - 设置 publish_tf=false 避免冲突
    """
    publish_tf = context.launch_configurations.get('publish_tf', 'false').lower() == 'true'
    terrain_name = context.launch_configurations.get('terrain_name', 'stepping_stones_90.png')
    
    nodes = []
    
    # 仅当 publish_tf=true 时才发布 TF 变换 (单独运行时使用)
    if publish_tf:
        # Static transform publisher: world -> map (identity transform)
        # 连接Gazebo世界坐标系和ROS的map坐标系
        world_to_map_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map_tf_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'map'],
            output='screen'
        )
        nodes.append(world_to_map_tf)

        # Static transform publisher: map -> odom (identity transform)
        # 连接map坐标系和odom坐标系
        map_to_odom_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
            output='screen'
        )
        nodes.append(map_to_odom_tf)

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
    
    # 添加参数用于控制是否发布TF变换
    # 当与gazebo一起运行时，gazebo.launch.py会发布TF变换，此时设置为false（默认值）
    # 当单独运行时，设置为true以发布这些TF变换
    publish_tf = DeclareLaunchArgument(
        'publish_tf',
        default_value='false',  # 默认false，假设与gazebo一起运行
        description='Whether to publish world->odom and map->odom transforms. Set to true when running standalone.'
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
        publish_tf,
        terrain_name,
        OpaqueFunction(function=launch_setup)
    ])