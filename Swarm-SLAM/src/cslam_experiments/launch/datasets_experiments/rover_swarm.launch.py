"""Launch the AiRover fleet simulation + one Swarm-SLAM stack per rover.

This is the single entry point that merges the two repos into one launch
argument. `max_nb_robots` is the one knob that drives BOTH sides:

    max_nb_robots := N
        rover_description/simulation.launch.py  -> num_rovers := N
        cslam (cslam_rover_lidar.launch.py)     -> N x (3 nodes in /r0../r{N-1})
        nav2 (rover_description/nav2_stack)     -> N x Nav2+slam_toolbox in
                                                  /rover_0../rover_{N-1}

Per-robot wiring (rover_i  <->  cslam robot i):
    /rover_i/lidar/points  ->  /r{i}/pointcloud   (lidar_handler PointCloud2)
    /rover_i/odom          ->  /r{i}/odom         (lidar_handler Odometry)

cslam's frontend takes odometry off the `odom` topic message (not TF), so those
two remaps are the whole sensor bridge. All cslam nodes run on sim time (set in
config/rover_lidar.yaml) so they stay in sync with Gazebo's /clock.

Nav2: each rover also gets its own full Nav2 + slam_toolbox stack running in
/rover_i (actions/services like /rover_i/navigate_to_pose, per-rover /map). Its
velocity smoother writes /rover_i/cmd_vel — the cmd_vel_arbiter autonomy input —
and tf_relay mirrors the sim's global TF into each /rover_i/tf. Disable with
nav2:=false.

Sensors: minimal_sensors:=true drops the GPU-heavy camera + all 6 sonars on
every rover and keeps the SLAM essentials only (lidar scan + points, IMU,
odometry) — cslam, slam_toolbox and Nav2 keep working. Use on weak machines
to reclaim real-time factor.

Example:
    ros2 launch cslam_experiments rover_swarm.launch.py max_nb_robots:=3

Then drive the fleet with the "Fleet Teleop" terminal (1..N switch active
rover, 'b' broadcasts to all). cslam builds a pose graph per rover and merges
them across /cslam/* topics as rovers revisit the same places. Nav2 goals go to
/rover_i/navigate_to_pose (e.g. ros2 action send_goal ...).
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


FLEET_DEFAULTS = {
    'namespace_prefix': 'rover_',
    'first_index': 0,
}


def _rover_fleet():
    """Read rover_description's fleet block so the rover_i <-> r{i} mapping
    always matches the rover's own namespace_prefix / first_index settings."""
    try:
        pkg = get_package_share_directory('rover_description')
    except Exception:
        raise RuntimeError(
            "rover_description package not found. Build it first: "
            "colcon build --packages-up-to rover_description && source install/setup.bash")
    with open(os.path.join(pkg, 'config', 'swarm.yaml')) as f:
        data = yaml.safe_load(f) or {}
    fleet = (data.get('fleet') or {}).get('ros__parameters') or {}
    prefix = fleet.get('namespace_prefix', FLEET_DEFAULTS['namespace_prefix'])
    first = int(fleet.get('first_index', FLEET_DEFAULTS['first_index']))
    return prefix, first


def launch_setup(context, *args, **kwargs):
    max_nb_robots = int(LaunchConfiguration('max_nb_robots').perform(context))
    max_nb_robots = max(1, max_nb_robots)

    prefix, first = _rover_fleet()

    # --- Rover fleet simulation (num_rovers synced to max_nb_robots) ---
    rover_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rover_description'),
                         'launch', 'simulation.launch.py')),
        launch_arguments={
            'num_rovers': str(max_nb_robots),
            'world': LaunchConfiguration('world'),
            'full_sensors_all': LaunchConfiguration('full_sensors_all'),
            'minimal_sensors': LaunchConfiguration('minimal_sensors'),
            'rviz': LaunchConfiguration('rviz'),
            'ground_truth': LaunchConfiguration('ground_truth'),
        }.items(),
    )

    # --- One cslam stack per rover ---
    config_path = os.path.join(
        get_package_share_directory('cslam_experiments'), 'config/')
    config_file = LaunchConfiguration('config_file').perform(context)

    enable_rdv = LaunchConfiguration('enable_simulated_rendezvous').perform(context).lower()
    rendezvous_schedule_file = ''
    if enable_rdv in ('1', 'true', 'yes'):
        rendezvous_schedule_file = os.path.join(
            get_package_share_directory('cslam_experiments'),
            'config', 'rendezvous',
            LaunchConfiguration('rendezvous_config').perform(context))

    cslam_processes = []
    for i in range(max_nb_robots):
        rover_ns = f'{prefix}{first + i}'
        proc = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('cslam_experiments'),
                             'launch', 'cslam', 'cslam_rover_lidar.launch.py')),
            launch_arguments={
                'config_path': config_path,
                'config_file': config_file,
                'robot_id': str(i),
                'namespace': '/r' + str(i),
                'rover_namespace': rover_ns,
                'max_nb_robots': str(max_nb_robots),
                'enable_simulated_rendezvous': LaunchConfiguration('enable_simulated_rendezvous'),
                'rendezvous_schedule_file': rendezvous_schedule_file,
            }.items(),
        )
        cslam_processes.append(proc)

    # --- One Nav2 + slam_toolbox stack per rover (in /rover_i) ---
    nav2_processes = []
    rover_ns_list = [f'{prefix}{first + i}' for i in range(max_nb_robots)]
    if LaunchConfiguration('nav2').perform(context).lower() not in ('0', 'false', 'no'):
        nav2_processes.append(Node(
            package='rover_description',
            executable='tf_relay.py',
            name='tf_relay',
            arguments=['--namespaces', *rover_ns_list],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ))
        # Relay global /initialpose (where RViz's "2D Pose Estimate" publishes)
        # into each rover's namespace so namespaced Nav2 receives it.
        for rover_ns in rover_ns_list:
            nav2_processes.append(Node(
                package='topic_tools',
                executable='relay',
                name=f'initialpose_relay_{rover_ns}',
                arguments=['/initialpose', f'/{rover_ns}/initialpose'],
                parameters=[{'use_sim_time': True}],
                output='screen',
            ))
        # Relay global /goal_pose (where RViz's "2D Nav Goal" tool publishes)
        # into each rover's namespace for the namespaced bt_navigator.
        for rover_ns in rover_ns_list:
            nav2_processes.append(Node(
                package='topic_tools',
                executable='relay',
                name=f'goalpose_relay_{rover_ns}',
                arguments=['/goal_pose', f'/{rover_ns}/goal_pose'],
                parameters=[{'use_sim_time': True}],
                output='screen',
            ))
        nav2_stack_launch = os.path.join(
            get_package_share_directory('rover_description'),
            'launch', 'nav2_stack.launch.py')
        for rover_ns in rover_ns_list:
            nav2_processes.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_stack_launch),
                launch_arguments={
                    'rover_namespace': rover_ns,
                    'slam': LaunchConfiguration('slam'),
                    'autostart': LaunchConfiguration('autostart'),
                    'use_composition': LaunchConfiguration('use_composition'),
                }.items(),
            ))

        # Workaround: a bt_navigator only transitions to active once it has
        # been queried (first `ros2 lifecycle get` reports "node not found",
        # the retry reports active[3]). Last course of action after everything
        # else is up: 30 s after launch, query each rover's bt_navigator twice,
        # 4 s apart, and print both results so the operator can verify.
        def _lifecycle_get(ns):
            return ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'get', f'/{ns}/bt_navigator'],
                output='screen',
            )

        nav2_processes.append(TimerAction(
            period=30.0,
            actions=[_lifecycle_get(ns) for ns in rover_ns_list]))
        nav2_processes.append(TimerAction(
            period=34.0,
            actions=[_lifecycle_get(ns) for ns in rover_ns_list]))

    return [rover_launch] + cslam_processes + nav2_processes


def generate_launch_description():
    default_world = os.path.join(
        get_package_share_directory('rover_description'),
        'worlds', 'test_station.sdf')

    return LaunchDescription([
        DeclareLaunchArgument(
            'max_nb_robots', default_value='1',
            description='Number of rovers AND cslam robots (1-10). Synced: '
                        'rover num_rovers == cslam max_nb_robots == this value.'),
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='Absolute path to a .sdf world for the rover sim.'),
        DeclareLaunchArgument(
            'config_file', default_value='rover_lidar.yaml',
            description='cslam config file in cslam_experiments/config/.'),
        DeclareLaunchArgument(
            'full_sensors_all', default_value='',
            description='true = every rover gets full sensors; false = only the '
                        'first rover is a scout (lite rovers have NO lidar, so '
                        'their cslam robot gets no pointcloud). For SLAM testing '
                        'leave this true/empty. Empty = use rover swarm.yaml.'),
        DeclareLaunchArgument(
            'minimal_sensors', default_value='false',
            description='true = every rover keeps ONLY lidar + IMU (scan + '
                        'points, joint states, odometry) and drops the '
                        'GPU-heavy camera + all 6 sonars. cslam, slam_toolbox '
                        'and Nav2 keep working. Use on weak machines to '
                        'reclaim real-time factor.'),
        DeclareLaunchArgument(
            'enable_simulated_rendezvous', default_value='false',
            description='Simulate limited inter-robot communication windows.'),
        DeclareLaunchArgument(
            'rendezvous_config', default_value='',
            description='Rendezvous schedule file (config/rendezvous/) used only '
                        'when enable_simulated_rendezvous is true.'),
        DeclareLaunchArgument(
            'nav2', default_value='true',
            description='true = also run a Nav2 + slam_toolbox stack per rover '
                        '(actions at /rover_i/navigate_to_pose etc.).'),
        DeclareLaunchArgument(
            'slam', default_value='True',
            description='Whether the per-rover nav2 stacks run slam_toolbox '
                        '(map built on the fly) or load a map.'),
        DeclareLaunchArgument(
            'autostart', default_value='True',
            description='Automatically startup the nav2 stacks'),
        DeclareLaunchArgument(
            'use_composition', default_value='False',
            description='Use composed bringup for the nav2 stacks if True '
                        '(separate processes are easier to debug)'),
        DeclareLaunchArgument(
            'ground_truth', default_value='true',
            description='true (default) = rover odometry is Gazebo ground truth '
                        '(map + Nav2 + cslam run on the true pose). false = '
                        'realistic encoder odometry with wheel slip — use this '
                        'for SLAM benchmarking.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='true (default) = open the generated RViz config. '
                        'false = run headless (no rviz2 window) — reclaims ~50% '
                        'of a core on CPU-only/no-GPU machines and keeps the sim '
                        'at real speed.'),
        OpaqueFunction(function=launch_setup),
    ])
