import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):

    return [
        DeclareLaunchArgument('bag_file', default_value='', description='Path to a bag produced by kitti2rosbag2 (topics: /velodyne_points, /imu/data, /gps/fix).'),
        DeclareLaunchArgument('namespace', default_value='/r0', description=''),
        DeclareLaunchArgument('rate', default_value='0.2', description=''),
        DeclareLaunchArgument('bag_start_delay', default_value='5.0', description=''),
        TimerAction(
            period=LaunchConfiguration('bag_start_delay'),
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'bag', 'play',
                        LaunchConfiguration('bag_file').perform(context),
                        '-r', LaunchConfiguration('rate'),
                        # No --clock/use_sim_time: `ros2 bag play --clock` in this ROS 2
                        # distro doesn't scale /clock by `-r`, so at rate != 1.0 it drifts
                        # away from the bag's own message timestamps. The static TF
                        # broadcaster instead stamps directly off incoming sensor data
                        # (see periodic_static_tf.py) - no clock in the loop to drift.
                        # Camera topics are skipped entirely (no decode/transport cost).
                        # /tf is also skipped: kitti2rosbag2 now records real extrinsics
                        # (odom->velodyne->imu_link, rigid mounts) plus the per-frame
                        # ground truth on map->odom, but its "odom" is the vehicle body
                        # frame, not icp_odometry's dead-reckoning "odom" - playing it back
                        # would fight icp_odometry's own odom->base_link broadcast. We
                        # substitute fixed identity mounts instead (see the dataset launch).
                        '--topics', '/velodyne_points', '/imu/data', '/gps/fix',
                        '--remap',
                        '/velodyne_points:=' +
                        LaunchConfiguration('namespace').perform(context) +
                        '/pointcloud',
                        '/imu/data:=' +
                        LaunchConfiguration('namespace').perform(context) +
                        '/imu/data',
                        '/gps/fix:=' +
                        LaunchConfiguration('namespace').perform(context) +
                        '/gps/fix',
                    ],
                    name='bag',
                    output='screen',
                )
            ]),
    ]


def generate_launch_description():

    return LaunchDescription([OpaqueFunction(function=launch_setup)])
