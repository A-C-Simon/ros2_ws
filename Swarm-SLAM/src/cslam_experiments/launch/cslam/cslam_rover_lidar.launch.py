import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Launch the 3 cslam nodes for ONE robot, wired to ONE rover.

    This is a rover-specific variant of cslam_lidar.launch.py. The only real
    difference is the two remappings on cslam_map_manager: cslam's relative
    `pointcloud`/`odom` topics are pointed at the rover's namespaced sensor
    topics instead of a dataset bag player. cslam's frontend reads odometry off
    the `odom` topic MESSAGE (not TF), so these two remaps are the entire
    sensor wiring between the rover and cslam.
    """
    rover_ns = LaunchConfiguration('rover_namespace').perform(context)
    pointcloud_topic = f'/{rover_ns}/lidar/points'
    odom_topic = f'/{rover_ns}/odom'

    loop_detection_node = Node(
        package='cslam',
        executable='loop_closure_detection_node.py',
        name='cslam_loop_closure_detection',
        parameters=[
            LaunchConfiguration('config'), {
                "robot_id": LaunchConfiguration('robot_id'),
                "max_nb_robots": LaunchConfiguration('max_nb_robots'),
            }
        ],
        namespace=LaunchConfiguration('namespace'))

    map_manager_node = Node(
        package='cslam',
        executable='lidar_handler_node.py',
        name='cslam_map_manager',
        parameters=[
            LaunchConfiguration('config'), {
                "robot_id": LaunchConfiguration('robot_id'),
                "max_nb_robots": LaunchConfiguration('max_nb_robots'),
            }
        ],
        remappings=[
            ('pointcloud', pointcloud_topic),
            ('odom', odom_topic),
        ],
        prefix=LaunchConfiguration('launch_prefix_cslam'),
        namespace=LaunchConfiguration('namespace'))

    pose_graph_manager_node = Node(
        package='cslam',
        executable='pose_graph_manager',
        name='cslam_pose_graph_manager',
        parameters=[
            LaunchConfiguration('config'), {
                "robot_id": LaunchConfiguration('robot_id'),
                "max_nb_robots": LaunchConfiguration('max_nb_robots'),
                "evaluation.enable_simulated_rendezvous":
                    LaunchConfiguration('enable_simulated_rendezvous'),
                "evaluation.rendezvous_schedule_file":
                    LaunchConfiguration('rendezvous_schedule_file'),
            }
        ],
        prefix=LaunchConfiguration('launch_prefix_cslam'),
        namespace=LaunchConfiguration('namespace'))

    return [
        loop_detection_node,
        map_manager_node,
        pose_graph_manager_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/r0',
                              description='cslam namespace for this robot (/r0, /r1, ...)'),
        DeclareLaunchArgument('robot_id', default_value='0', description=''),
        DeclareLaunchArgument('max_nb_robots', default_value='1', description=''),
        DeclareLaunchArgument('rover_namespace', default_value='rover_0',
                              description='Rover namespace to read sensors from (rover_0, rover_1, ...)'),
        DeclareLaunchArgument('config_path',
                              default_value=os.path.join(
                                  get_package_share_directory('cslam_experiments'),
                                  'config/'),
                              description=''),
        DeclareLaunchArgument('config_file',
                              default_value='rover_lidar.yaml',
                              description=''),
        DeclareLaunchArgument('config',
                              default_value=[
                                  LaunchConfiguration('config_path'),
                                  LaunchConfiguration('config_file')
                              ],
                              description=''),
        DeclareLaunchArgument('launch_prefix_cslam', default_value='',
                              description='Debug prefix, e.g. "xterm -e gdb -ex run --args"'),
        DeclareLaunchArgument('enable_simulated_rendezvous', default_value='false',
                              description=''),
        DeclareLaunchArgument('rendezvous_schedule_file', default_value='',
                              description=''),
        OpaqueFunction(function=launch_setup),
    ])
