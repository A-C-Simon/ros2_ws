import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Rover sim + a Nav2/slam_toolbox stack for one rover (default rover_0).

    Reuses launch/nav2_stack.launch.py (namespaced bringup) plus a tf_relay
    that mirrors the sim's global /tf + /tf_static into the rover's namespace.
    """
    rover_description_dir = get_package_share_directory('rover_description')

    rover_namespace = LaunchConfiguration('rover_namespace')
    num_rovers = LaunchConfiguration('num_rovers')
    teleop = LaunchConfiguration('teleop')
    slam = LaunchConfiguration('slam')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')

    sim_launch = os.path.join(rover_description_dir, 'launch', 'simulation.launch.py')
    stack_launch = os.path.join(rover_description_dir, 'launch', 'nav2_stack.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rover_namespace', default_value='rover_0',
            description='Namespace of the rover to run Nav2 on'),

        DeclareLaunchArgument(
            'num_rovers', default_value='1',
            description='Number of rovers to spawn in the simulation'),

        DeclareLaunchArgument(
            'teleop', default_value='true',
            description='Open the Fleet Teleop terminal (idle-silent; Nav2 autonomy '
                        'still reaches the rover via cmd_vel_arbiter)'),

        DeclareLaunchArgument(
            'slam', default_value='True',
            description='Whether to run slam_toolbox (map built on the fly) or load a map'),

        DeclareLaunchArgument(
            'autostart', default_value='True',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'use_composition', default_value='False',
            description='Use composed bringup if True (separate processes are easier to debug)'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                'num_rovers': num_rovers,
                'teleop': teleop,
            }.items()),

        # Mirror the global sim TF into the rover namespace for the nav2 stack.
        Node(
            package='rover_description',
            executable='tf_relay.py',
            name='tf_relay',
            arguments=['--namespaces', rover_namespace],
            parameters=[{'use_sim_time': True}],
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stack_launch),
            launch_arguments={
                'rover_namespace': rover_namespace,
                'slam': slam,
                'autostart': autostart,
                'use_composition': use_composition,
            }.items()),
    ])
