import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context, *args, **kwargs):
    # Get the package share directory for controllers
    pkg_legged_controllers = get_package_share_directory('ocs2_quadruped_controller')

    # Component container 对激光雷达数据进行过滤
    # container = ComposableNodeContainer(
    #     name='point_cloud_manager',
    #     namespace='',
    #     package='rclcpp_components',
    #     executable='component_container',
    #     composable_node_descriptions=[
    #         # Livox right filter component
    #         ComposableNode(
    #             package='sensor_filters',
    #             plugin='sensor_filters::PointCloud2FilterChain',
    #             name='livox_right_filter',
    #             remappings=[
    #                 ('input', '/livox/lidar_192_168_8_101'),
    #                 ('output', '/livox/right')
    #             ],
    #             parameters=[os.path.join(pkg_legged_controllers, 'config', 'body_filter.yaml')]
    #         ),
    #         # Livox left filter component
    #         ComposableNode(
    #             package='sensor_filters',
    #             plugin='sensor_filters::PointCloud2FilterChain',
    #             name='livox_left_filter',
    #             remappings=[
    #                 ('input', '/livox/lidar_192_168_8_102'),
    #                 ('output', '/livox/left')
    #             ],
    #             parameters=[os.path.join(pkg_legged_perceptive_controllers, 'config', 'body_filter.yaml')]
    #         )
    #     ],
    #     output='screen'
    # )

    # Elevation mapping node
    elevation_mapping_yaml = os.path.join(pkg_legged_controllers, 'config', 'elevation_mapping.yaml')
    elevation_mapping = Node(
        package='elevation_mapping',
        executable='elevation_mapping',
        name='elevation_mapping',
        parameters=[elevation_mapping_yaml],
        output='screen',
        # arguments=['--ros-args', '--log-level', 'elevation_mapping:=debug']
    )

    # Include convex_plane_decomposition.launch
    convex_plane_decomposition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('convex_plane_decomposition_ros'),
                'launch',
                'convex_plane_decomposition.launch.py'  # Assuming Python launch file
            ])
        ),
        launch_arguments={
            'parameter_file': os.path.join(pkg_legged_controllers, 'config', 'convex_plane_decomposition.yaml')
        }.items()
    )

    # Static transform publisher (map to odom)
    odom_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
        output='screen'
    )

    return [
        # container,
        elevation_mapping,
        convex_plane_decomposition_launch,
        odom_tf_publisher
    ]

def generate_launch_description():
    # Declare launch arguments
    # container_name_arg = DeclareLaunchArgument(
    #     name='container_name',
    #     default_value='point_cloud_container',
    #     description='Name of the component container'
    # )

    return LaunchDescription([
        # container_name_arg,
        OpaqueFunction(function=launch_setup)
    ])