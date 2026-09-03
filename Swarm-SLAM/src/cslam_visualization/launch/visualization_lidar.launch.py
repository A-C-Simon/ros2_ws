import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    visualization_node = Node(package='cslam_visualization',
                               executable='visualization_node.py',
                               name='cslam_visualization',
                               parameters=[
                                   LaunchConfiguration('config'), {
                                       'use_sim_time':
                                       LaunchConfiguration('use_sim_time')
                                   }
                               ])
    print(LaunchConfiguration('rviz_config').perform(context))
    rviz_node = Node(package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config').perform(context)],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        )
    
    return [
        visualization_node,
        rviz_node,
    ]


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('config_path',
                              default_value=os.path.join(
                                  get_package_share_directory('cslam_visualization'),
                                  'config/'),
                              description=''),
        DeclareLaunchArgument('config_file',
                              default_value='lidar.yaml',
                              description=''),
        DeclareLaunchArgument('rviz_config_file',
                              default_value='lidar.rviz',
                              description=''),
        DeclareLaunchArgument('config',
                              default_value=[
                                  LaunchConfiguration('config_path'),
                                  LaunchConfiguration('config_file')
                              ],
                              description=''),
        DeclareLaunchArgument('rviz_config',
                              default_value=[
                                  LaunchConfiguration('config_path'),
                                  LaunchConfiguration('rviz_config_file')
                              ],
                              description=''),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Set true when visualizing a sim-time swarm (e.g. the '
                        'rover swarm, which runs off Gazebo /clock). The cslam '
                        'pose-graph TF frames are then stamped with sim time '
                        'and RViz must use the same clock or "robot0_map" '
                        'never resolves.'),
        OpaqueFunction(function=launch_setup)
    ])
