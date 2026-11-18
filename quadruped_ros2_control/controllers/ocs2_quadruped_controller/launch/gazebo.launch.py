import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler, \
    TimerAction, Shutdown, LogInfo, ExecuteProcess
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Gazebo World
    world = context.launch_configurations['world']
    default_sdf_path = os.path.join(get_package_share_directory('gz_quadruped_playground'), 'worlds', world + '.sdf')
    world_sdf_name = os.path.basename(default_sdf_path)
    gazebo_cleanup_cmd = (
        f'pkill -TERM -f "gz sim.*{world_sdf_name}" 2>/dev/null || true; '
        'pkill -TERM -f "ruby .*gz sim" 2>/dev/null || true; '
        'pkill -TERM -x rviz2 2>/dev/null || true; '
        'sleep 1; '
        f'pkill -KILL -f "gz sim.*{world_sdf_name}" 2>/dev/null || true; '
        'pkill -KILL -f "ruby .*gz sim" 2>/dev/null || true; '
        'pkill -KILL -x rviz2 2>/dev/null || true'
    )

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
                   '-R', context.launch_configurations['roll'],
                   '-P', context.launch_configurations['pitch'],
                   '-Y', context.launch_configurations['yaw'],
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
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz')),
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

    # # ------------------------------------------------------------
    # # 自动轨迹生成节点——用于在仿真中自动生成轨迹，用于测试
    # traject_build = Node(
    #     package='test',
    #     executable='traject_build',
    #     name='traject_build',
    #     output='screen',
    #     parameters=[{
    #         'motion_sequence': context.launch_configurations['traject_motion_sequence'],
    #         'cmd2_to_cmd3_delay': float(context.launch_configurations['traject_cmd2_to_cmd3_delay']),
    #         'cmd3_to_motion_delay': float(context.launch_configurations['traject_cmd3_to_motion_delay']),
    #         'startup_delay': float(context.launch_configurations['traject_startup_delay']),
    #         'post_motion_delay': float(context.launch_configurations['traject_post_motion_delay']),
    #     }],
    # )
    # shutdown_launch_on_traject_exit = Shutdown(
    #     reason='traject_build finished, shutting down launch'
    # )
    # force_stop_simulation = ExecuteProcess(
    #     cmd=['bash', '-lc', gazebo_cleanup_cmd],
    #     output='screen',
    #     on_exit=[shutdown_launch_on_traject_exit],
    # )
    # traject_build_timer = TimerAction(
    #     period=float(context.launch_configurations['traject_start_delay']),
    #     actions=[traject_build],
    #     condition=IfCondition(LaunchConfiguration('auto_traject_build')),
    # )
    # # ------------------------------------------------------


    elevation_launch_mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ocs2_quadruped_controller'),
                                   'launch',
                                   'elevation_mapping.launch.py'])]),
  
    )

    
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
    map_to_odom_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf_publisher',
        # x y z qx qy qz qw parent_frame child_frame
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '1.0', 'map', 'odom'],
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
            launch_arguments=[
                ('gz_args', [' -r -v 2 ', '' if context.launch_configurations['gui'] == 'true' else '-s ', default_sdf_path]),
                ('on_exit_shutdown', 'true'),
            ]),          
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

        # # ----------------------------------------------------------
        # RegisterEventHandler(                   # 自动轨迹生成节点——用于在仿真中自动生成轨迹，用于测试
        #     event_handler=OnProcessExit(
        #         target_action=ocs2_controller,
        #         on_exit=[traject_build_timer],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=traject_build,
        #         on_exit=[
        #             LogInfo(msg='traject_build finished, stopping simulation and shutting down launch.'),
        #             force_stop_simulation,
        #         ],
        #     ),
        #     condition=IfCondition(LaunchConfiguration('auto_traject_build')),
        # ),
        # RegisterEventHandler(
        #     event_handler=OnShutdown(
        #         on_shutdown=[
        #             LogInfo(msg='Launch shutdown requested, ensuring Gazebo and RViz processes are terminated.'),
        #             ExecuteProcess(
        #                 cmd=['bash', '-lc', gazebo_cleanup_cmd],
        #                 output='screen',
        #             ),
        #         ]
        #     )
        # ),
        # # ----------------------------------------------------------

        elevation_launch_mapping_launch,

        world_to_map_tf_publisher,
        map_to_odom_tf_publisher,
    ]


def generate_launch_description():
    world = DeclareLaunchArgument(
        'world',
        default_value='stair10cm',       # default,stair10cm, stair15cm, stair20cm, ramp15deg, ramp25deg, stepping_stones， stepping_stones_terrain
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
    default_value='0.0',
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


    # # ----------------------------------------------------------
    # auto_traject_build = DeclareLaunchArgument(
    #     'auto_traject_build',
    #     default_value='true',
    #     description='Automatically run traject_build after the OCS2 controller is loaded'
    # )

    # # 轨迹对应参数：
    # # 前进2.5 m：  'w:7.15'
    # # 前进 2.5 m 后左转 75° 后前进2.5m: 'w:7.15;j:1.25;w:7.15'
    # # 前进 2.5 m 后右转 75° 后前进2.5m： 'w:7.15;l:1.25;w:7.15'
    # # 半径 3.5 m 完整圆: 'w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;',
    # # 原地旋转30s: 'j:30.0'  
    # traject_motion_sequence = DeclareLaunchArgument(
    #     'traject_motion_sequence',
    #     default_value='w:0.75;l:0.21;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;w:0.75;l:0.141;',
    #     description='Motion sequence for traject_build'
    # )
    # traject_cmd2_to_cmd3_delay = DeclareLaunchArgument(
    #     'traject_cmd2_to_cmd3_delay',
    #     default_value='2.0',
    #     description='Delay between command 2 and command 3 in traject_build'
    # )
    # traject_cmd3_to_motion_delay = DeclareLaunchArgument(
    #     'traject_cmd3_to_motion_delay',
    #     default_value='2.0',
    #     description='Delay between command 3 and motion sequence in traject_build'
    # )
    # traject_startup_delay = DeclareLaunchArgument(
    #     'traject_startup_delay',
    #     default_value='2.0',
    #     description='Initial wait inside traject_build before sending command 2'
    # )
    # traject_start_delay = DeclareLaunchArgument(
    #     'traject_start_delay',
    #     default_value='1.0',
    #     description='Delay after OCS2 controller load before starting traject_build'
    # )
    # traject_post_motion_delay = DeclareLaunchArgument(
    #     'traject_post_motion_delay',
    #     default_value='1.0',
    #     description='Delay after motion before traject_build exits and launch shuts down'
    # )
    # # ----------------------------------------------------------

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
            "/depth_camera/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            "/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/depth_camera/depth/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
        ],
        output="screen",
    )


    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Show Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='Show RViz'),
        world,
        pkg_description,
        z_pos,
        x_pos, 
        y_pos, 
        yaw,
        roll,
        pitch,


        # auto_traject_build,
        # traject_motion_sequence,
        # traject_cmd2_to_cmd3_delay,
        # traject_cmd3_to_motion_delay,
        # traject_startup_delay,
        # traject_start_delay,
        # traject_post_motion_delay,

        
        gz_bridge_node,
        OpaqueFunction(function=launch_setup),
    ])
