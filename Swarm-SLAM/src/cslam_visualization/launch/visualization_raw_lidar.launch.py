"""RViz view of ONE robot's raw odometry and lidar, in its own `odom` frame.

Why this is separate from visualization_lidar.launch.py
------------------------------------------------------
There are two disconnected TF trees in a cslam lidar run and no transform
ever links them:

  global /tf   robotX_map -> robotY_map -> robotN_latest_optimized_pose
                          -> robotN_current_pose       (decentralized_pgo)
  /rN/tf       odom -> base_link -> velodyne / imu_link (icp_odometry +
                                                        periodic_static_tf)

decentralized_pgo only ever broadcasts inside the robotN_* family, so
`odom`/`velodyne` can never resolve against `robot0_map`. RViz has a single
fixed frame, so one view cannot show both trees - lidar.rviz shows the
optimized map, and this one shows the raw sensor stream. In lidar.rviz the raw
displays are therefore present but disabled; enabling them there only produces
"discarding message because the queue is full" forever.

The per-robot local frames are also deliberately identical across robots
(`odom`, `base_link`, `velodyne`), which is exactly why the dataset launch
isolates them onto /rN/tf. That means this view is one robot at a time: pick
which with `robot:=r1`.

Usage
-----
  ros2 launch cslam_visualization visualization_raw_lidar.launch.py
  ros2 launch cslam_visualization visualization_raw_lidar.launch.py robot:=r1

Can be run alongside visualization_lidar.launch.py to see both at once.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    ns = robot if robot.startswith('/') else '/' + robot
    rviz_config = os.path.join(
        get_package_share_directory('cslam_visualization'),
        'config', LaunchConfiguration('rviz_config_file').perform(context))

    return [
        Node(
            package='rviz2',
            executable='rviz2',
            # The config uses relative topic names ("odom", "pointcloud",
            # "odom_filtered_input_scan") so the node's namespace selects the
            # robot without needing a config file per robot.
            namespace=ns,
            name='rviz2_raw',
            arguments=['-d', rviz_config],
            # tf2 always uses the absolute /tf and /tf_static regardless of
            # namespace, so the per-robot tree has to be remapped explicitly.
            remappings=[('/tf', ns + '/tf'),
                        ('/tf_static', ns + '/tf_static')],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot', default_value='r0',
            description="Which robot's raw data to view (r0, r1, ...). The local "
                        "frames are identically named across robots, so only one "
                        "can be shown at a time."),
        DeclareLaunchArgument('rviz_config_file', default_value='raw_lidar.rviz',
                              description=''),
        OpaqueFunction(function=launch_setup),
    ])
