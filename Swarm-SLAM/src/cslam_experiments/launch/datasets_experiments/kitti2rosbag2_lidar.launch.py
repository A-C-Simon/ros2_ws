import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import TimerAction, OpaqueFunction, PushLaunchConfigurations, PopLaunchConfigurations, DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    config_path = os.path.join(
        get_package_share_directory("cslam_experiments"), "config/")
    config_file = LaunchConfiguration('config_file').perform(context)

    max_nb_robots = int(LaunchConfiguration('max_nb_robots').perform(context))
    robot_delay_s = LaunchConfiguration('robot_delay_s').perform(context)
    launch_delay_s = LaunchConfiguration('launch_delay_s').perform(context)
    rate = float(LaunchConfiguration('rate').perform(context))

    # One bag per robot, comma-separated (defaults to a single bag for robot 0).
    bag_files = [
        b.strip() for b in LaunchConfiguration('bag_files').perform(context).split(',')
        if b.strip() != ''
    ]
    if len(bag_files) != max_nb_robots:
        raise RuntimeError(
            f"'bag_files' must list exactly {max_nb_robots} comma-separated bag path(s) "
            f"(one per robot), got {len(bag_files)}: {bag_files}")

    robot_delay_s = float(robot_delay_s) / rate
    launch_delay_s = float(launch_delay_s) / rate

    cslam_processes = []
    bag_processes = []
    odom_processes = []
    tf_processes = []

    for i in range(max_nb_robots):
        proc = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("cslam_experiments"),
                             "launch", "cslam", "cslam_lidar.launch.py")),
            launch_arguments={
                "config_path": config_path,
                "config_file": config_file,
                "robot_id": str(i),
                "namespace": "/r" + str(i),
                "max_nb_robots": str(max_nb_robots),
                "enable_simulated_rendezvous": LaunchConfiguration('enable_simulated_rendezvous'),
                "rendezvous_schedule_file": os.path.join(get_package_share_directory("cslam_experiments"),
                             "config", "rendezvous", LaunchConfiguration('rendezvous_config').perform(context)),
            }.items(),
        )
        cslam_processes.append(proc)

        bag_proc = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("cslam_experiments"),
                    "launch",
                    "sensors",
                    "bag_kitti2rosbag2.launch.py",
                )),
            launch_arguments={
                "namespace": "/r" + str(i),
                "bag_file": bag_files[i],
                "rate": str(rate)
            }.items(),
        )
        bag_processes.append(bag_proc)

        # kitti2rosbag2's own /tf carries the real rigid mounts (odom->velodyne->imu_link),
        # but we don't play that back (see bag_kitti2rosbag2.launch.py) since its "odom" is
        # the vehicle body frame, not icp_odometry's dead-reckoning "odom". Instead, mirror
        # upstream kitti_lidar.launch.py's own simplification: treat velodyne and imu_link
        # as coincident with base_link (identity offsets) rather than threading the real
        # extrinsics through - upstream doesn't bother with precise KITTI extrinsics either.
        #
        # base_link is the parent of both (not the reverse, as upstream's own
        # velo_link->base_link / imu_link->base_link pair does) - a frame can only have
        # one parent, so publishing base_link as a *child* of two different sensor frames
        # makes it structurally ambiguous which sensor base_link is actually attached to,
        # and tf2 reports "two or more unconnected trees" for lookups against the other
        # one. Rooting both sensors under base_link avoids that outright.
        #
        # A one-shot tf2_ros static_transform_publisher (latched on /tf_static) stamps
        # once at startup; icp_odometry's TF lookups at later bag-time timestamps then
        # report "extrapolation" against that single, now-stale sample. Re-broadcasting
        # on every pointcloud message, stamped with THAT message's own header.stamp, keeps
        # the TF within icp_odometry's wait_for_transform tolerance without relying on any
        # clock: `ros2 bag play --clock` doesn't scale /clock by `-r` in this ROS 2 distro
        # (it advances at wall-clock pace regardless of playback rate), so at rate != 1.0 a
        # use_sim_time-driven broadcaster drifted out of sync and every lookup failed.
        #
        # One instance per robot, each stamped off *that* robot's own pointcloud and
        # remapped onto that robot's own private "/rN/tf": tf2_ros always publishes to the
        # absolute "/tf"/"/tf_static" topics regardless of node namespace, so with a single
        # shared instance keyed to robot 0's pointcloud (as before), robot 1's base_link
        # mount would go stale/extrapolate the moment robot 1's timestamps drifted from
        # robot 0's - and even a naive per-robot instance sharing the *global* "/tf" would
        # still collide, since both robots' bags carry the same literal "velodyne"/
        # "imu_link" frame_ids (baked in by kitti2rosbag2's recorder) and icp_odometry's
        # own odom->base_link broadcast is unnamespaced by default too. Isolating each
        # robot's local sensor/odom TF onto its own private topic sidesteps all of that
        # without needing to rename any frame: cslam's own frontend never reads this local
        # TF at all (lidar_handler_node.py takes odometry off the `odom` topic message, not
        # TF), and the cross-robot map frames (robotX_map, etc.) are broadcast separately by
        # decentralized_pgo straight onto the global /tf - only rtabmap's internal ICP
        # registration needs this mount, so it can stay fully private per robot.
        tf_process = Node(package="cslam_experiments",
                          executable="periodic_static_tf.py",
                          namespace="/r" + str(i),
                          parameters=[{
                              "mounts": ["base_link:velodyne", "base_link:imu_link"],
                              "stamp_topic": "/r" + str(i) + "/pointcloud",
                          }],
                          remappings=[
                              ("/tf", "/r" + str(i) + "/tf"),
                              ("/tf_static", "/r" + str(i) + "/tf_static"),
                          ])
        tf_processes.append(tf_process)

        odom_proc = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('cslam_experiments'), 'launch',
                             'odometry', 'rtabmap_kitti_lidar_odometry.launch.py')),
            launch_arguments={
                "namespace": "/r" + str(i),
                "robot_id": str(i),
                # This never actually reached the node until the '--ros-args'
                # concatenation bug in rtabmap_kitti_lidar_odometry.launch.py was
                # fixed, so icp_odometry has always run at INFO in practice. Kept
                # at "info" rather than restored to "fatal": fatal suppresses
                # icp_odometry's own "TF of received scan cloud ... is not set"
                # errors and the dropped-scan warnings, which are the main signal
                # that odometry is silently discarding most of the lidar stream.
                # Raise to "warn" for quiet runs - that still keeps both.
                "log_level": "info",
                # kitti2rosbag2 now records real IMU (oxts orientation/accel/gyro),
                # remapped to <namespace>/imu/data by the bag launch below.
                "wait_imu_to_init": LaunchConfiguration('wait_imu_to_init'),
                "wait_for_transform": LaunchConfiguration('wait_for_transform'),
                # Same private TF topic as this robot's tf_process above, so its
                # base_link/odom frames never collide with another robot's.
                "tf_topic": "/r" + str(i) + "/tf",
                "tf_static_topic": "/r" + str(i) + "/tf_static",
            }.items(),
        )
        odom_processes.append(odom_proc)

    schedule = []

    for i in range(max_nb_robots):
        schedule.append(PushLaunchConfigurations())
        schedule.append(
            TimerAction(period=float(robot_delay_s) * i,
                        actions=[cslam_processes[i]]))
        schedule.append(PopLaunchConfigurations())
        schedule.append(PushLaunchConfigurations())
        schedule.append(
            TimerAction(period=float(robot_delay_s) * i,
                        actions=[odom_processes[i], tf_processes[i]]))
        schedule.append(PopLaunchConfigurations())

    for i in range(max_nb_robots):
        schedule.append(PushLaunchConfigurations())
        schedule.append(
            TimerAction(period=float(robot_delay_s) * i + float(launch_delay_s),
                        actions=[bag_processes[i]]))
        schedule.append(PopLaunchConfigurations())

    return schedule


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('tune_malloc', default_value='true', description='Pin glibc\'s mmap threshold for the nodes started below. The lidar pipeline moves ~2 MB PointCloud2 messages at ~8/s; glibc raises M_MMAP_THRESHOLD dynamically once a large mmap\'d block is freed, after which those buffers come from arenas and are never returned to the OS. Pinning it measurably cut peak RSS on a full KITTI run: lidar_handler 1174 -> 955 MB, icp_odometry 234 -> 184 MB, total 1958 -> 1681 MB, with cslam_loop_closure_detection unchanged (it never touches the big clouds - the control that confirms where the saving comes from). Set false to A/B it: odom updates were 1771 vs 1615 across the two runs, which is inside the 1583-1814 spread already seen between otherwise identical runs, so any throughput cost is unconfirmed.'),
        # Must precede the OpaqueFunction: these mutate the launch context's
        # environment, which is inherited by every process started afterwards
        # (including the included cslam/odometry launch files and ros2 bag play).
        SetEnvironmentVariable('MALLOC_MMAP_THRESHOLD_', '131072',
                               condition=IfCondition(LaunchConfiguration('tune_malloc'))),
        SetEnvironmentVariable('MALLOC_TRIM_THRESHOLD_', '262144',
                               condition=IfCondition(LaunchConfiguration('tune_malloc'))),
        DeclareLaunchArgument('bag_files', default_value='', description='Comma-separated list of kitti2rosbag2 bag paths, one per robot.'),
        DeclareLaunchArgument('max_nb_robots', default_value='1'),
        DeclareLaunchArgument('robot_delay_s', default_value='20', description="Delay between launching each robot. Adjust depending on the computing power of your machine."),
        DeclareLaunchArgument('launch_delay_s', default_value='10', description="Delay between launching the bag and the robot, so the robot initializes before bag data starts flowing."),
        DeclareLaunchArgument('config_file', default_value='kitti_lidar.yaml', description=''),
        DeclareLaunchArgument('rate', default_value='0.2'),
        DeclareLaunchArgument('wait_imu_to_init', default_value='true', description='kitti2rosbag2 bags carry real IMU data; set false only for bags recorded without imu:=True.'),
        DeclareLaunchArgument('wait_for_transform', default_value='0.9', description='icp_odometry TF lookup tolerance (s). periodic_static_tf and icp_odometry are independently-scheduled subscribers on the same pointcloud topic, and icp_odometry can query a stamp before its own /tf listener has ingested the matching sample - measured lag is always a whole number of scan periods, up to 8 at 10 Hz. Measured over full KITTI runs: 0.2 -> 232 "TF ... is not set" aborts, 0.9 -> 10. Raise further if aborts persist; the broadcaster itself publishes within ~21 ms (p99 67 ms) so this only ever covers the consumer-side lag.'),
        DeclareLaunchArgument('enable_simulated_rendezvous', default_value='false'),
        DeclareLaunchArgument('rendezvous_config', default_value='kitti00_2robots_lidar.config'),
        OpaqueFunction(function=launch_setup)
    ])
