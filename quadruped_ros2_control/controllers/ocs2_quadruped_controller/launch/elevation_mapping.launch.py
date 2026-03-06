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
    #---------------------------------------------------------------------------------
    # Static transform publisher (map to odom)
    odom_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
        output='screen'
    )
    world2map_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world2map_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'map'],
        output='screen'
    )
    #---------------------------------------------------------------------------------
    
    return [
        elevation_mapping,
        convex_plane_decomposition_launch,
        odom_tf_publisher,
        world2map_tf_publisher,
    ]

def generate_launch_description():

    return LaunchDescription([

        OpaqueFunction(function=launch_setup)
    ])