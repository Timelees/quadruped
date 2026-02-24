import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler, \
    TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Gazebo World
    world = context.launch_configurations['world']
    default_sdf_path = os.path.join(get_package_share_directory('gz_quadruped_playground'), 'worlds', world + '.sdf')

    # Init Height When spawn the model
   
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', 
                   '-name', 'robot', 
                   '-allow_renaming', 'true', 
                   '-x', context.launch_configurations['x'],  
                   '-y', context.launch_configurations['y'],  
                   '-z', context.launch_configurations['z'],  
                   '-roll', context.launch_configurations['roll'],
                   '-pitch', context.launch_configurations['pitch'],
                   '-yaw', context.launch_configurations['yaw'], 
                   ],
    )

    # Robot Description
    pkg_description = context.launch_configurations['pkg_description']
    pkg_path = os.path.join(get_package_share_directory(pkg_description))
    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')
    robot_description = xacro.process_file(xacro_file, mappings={
        'GAZEBO': 'true'
    }).toxml()
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'publish_frequency': 20.0,
                'use_tf_static': True,
                'robot_description': robot_description,
                'ignore_timestamp': True
            }
        ],
    )

    rviz_config_file = os.path.join(get_package_share_directory("ocs2_quadruped_controller"), "config", "visualize_ocs2.rviz")
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=["-d", rviz_config_file]
    )

    joint_state_publisher = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"]
    )

    imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster",
                   "--controller-manager", "/controller_manager"]
    )

    ocs2_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ocs2_quadruped_controller", "--controller-manager", "/controller_manager"]
    )
    
    # elevation_launch_mapping_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [PathJoinSubstitution([FindPackageShare('ocs2_quadruped_controller'),
    #                                'launch',
    #                                'elevation_mapping.launch.py'])]),
  
    # )


    # plot_terrain - 发布高程图数据用于地形感知
    # 注意：publish_tf设置为false，因为TF变换已在本文件中发布
    # plot_terrain_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [PathJoinSubstitution([FindPackageShare('plot_terrain'),
    #                                'launch',
    #                                'convex.launch.py'])]),
    #     launch_arguments={
    #         'publish_tf': 'false',  # 不发布TF，避免与本文件中的TF冲突
    #         'terrain_name': 'stepping_stones.png',  # 确保与Gazebo世界地形一致
    #     }.items(),
    # )
    
    # ============ TF树结构 ============
    # 正确的TF树: world -> map -> odom -> base -> body -> ...
    # - Gazebo/ros_gz_bridge 发布: odom -> base
    # - robot_state_publisher 发布: base -> body -> 各关节
    # - 我们需要发布: world -> map, map -> odom
    
    # Static transform publisher: world -> map (identity transform)
    # 连接Gazebo世界坐标系和ROS的map坐标系
    world_to_map_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_tf_publisher',
        # x y z qx qy qz qw parent_frame child_frame
        arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'map'],
        output='screen'
    )
    
    # Static transform publisher: map -> odom (identity transform)
    # 连接map坐标系和odom坐标系
    # NOTE: 
    #   - 状态估计器使用 Ground Truth 初始化，输出的 odom→base 已经是 world 坐标系下的位置
    #   - 因此 map → odom 应该是单位变换
    #   - 如果状态估计器从 (0,0,0) 开始，则需要设置为机器人初始位置 (x, y, 0)
    map_to_odom_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf_publisher',
        # x y z qx qy qz qw parent_frame child_frame
        arguments=['-0', '-0', '0.0', '0.0', '0.0', '0.0', '1.0', 'map', 'odom'],
        output='screen'
    )

    return [
        rviz,
        robot_state_publisher,
        gz_spawn_entity,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 4 ', default_sdf_path])]),          
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_publisher,imu_sensor_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_publisher,
                on_exit=[ocs2_controller],
            )
        ),
        # elevation_launch_mapping_launch,
        # plot_terrain 启动 - 在控制器启动后延迟启动以确保TF树可用
        # TimerAction(
        #     period=3.0,  # 等待3秒，确保仿真和TF树已经初始化
        #     actions=[plot_terrain_launch],
        # ),
        world_to_map_tf_publisher,
        map_to_odom_tf_publisher,
    ]


def generate_launch_description():
    world = DeclareLaunchArgument(
        'world',
        default_value='step',       # baylands,  step,   stepping_stones_terrain
        description='The world to load'
    )

    pkg_description = DeclareLaunchArgument(
        'pkg_description',
        default_value='cyberdog_description',   # cyberdog_description
        description='package for robot description'
    )


    # robot initial position
    x_pos = DeclareLaunchArgument(
    'x',
    default_value='-0.0',
    description='Initial X position'
    )
    y_pos = DeclareLaunchArgument(
        'y',
        default_value='-0.0',
        description='Initial Y position'
    )
    z_pos = DeclareLaunchArgument(
        'z',
        default_value='0.5',
        description='Init height in simulation'
    )
    yaw = DeclareLaunchArgument(  
        'yaw',
        default_value='0.0',
        description='Initial Yaw orientation in radians'
    )
    roll = DeclareLaunchArgument(  
        'roll',
        default_value='0.0',
        description='Initial Roll orientation in radians'
    )
    pitch = DeclareLaunchArgument(  
        'pitch',
        default_value='0.0',
        description='Initial Pitch orientation in radians'      
    )


    # ros和gazebo之间的桥接通信
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            # "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",        # 单线激光雷达数据
            # "/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",    # 3D点云数据
            # "/depth_camera/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            # "/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            # "/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            # "/depth_camera/depth/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
        ],
        output="screen",
    )

    # gz_image_bridge_node = Node(
    #     package="ros_gz_image",
    #     executable="image_bridge",
    #     arguments=[
    #         '/depth_camera/color/image_raw',  # Gazebo 侧话题
    #         '/depth_camera/color/image_raw',  # ROS 侧话题
    #     ],
    #     output="screen",
    #     parameters=[
    #         {'use_sim_time': True}
    #     ],
    # )

    return LaunchDescription([
        world,
        pkg_description,
        z_pos,
        x_pos, 
        y_pos, 
        yaw,
        roll,
        pitch,
        gz_bridge_node,
        # gz_image_bridge_node,
        OpaqueFunction(function=launch_setup),
    ])
