import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def _substitute_namespace(context):
    """Build the params file for one rover namespace and launch its Nav2 stack.

    Two nav2_bringup quirks are worked around here:

    1. navigation_launch.py's RewrittenYaml re-roots every block of the
       params file under '<namespace>:' itself. '<robot_namespace>' is
       replaced with the bare namespace ('rover_0') before the file is
       written, so tf FRAME values keep no leading slash (tf2 rejects frame
       ids starting with '/').

    2. slam_toolbox is launched HERE instead of via bringup_launch.py:
       humble's slam_launch.py never passes use_respawn down to the actual
       node (online_sync_launch.py builds it without respawn), so a killed
       slam_toolbox would never come back. With respawn=True the teleop R
       key can kill the process and the launch restarts it with an empty
       pose graph — a clean SLAM map reset. slam_toolbox reads its params
       from the '<namespace>.slam_toolbox' section of the emitted file
       (plus the dummy top-level 'slam_toolbox:' key so HasNodeParams-style
       checks pass the file through). All other blocks keep top-level keys:
       navigation_launch.py re-roots them under '<namespace>:' via
       RewrittenYaml, and map_saver is wrapped in its own RewrittenYaml.

    The whole group is wrapped in PushRosNamespace (as bringup_launch.py
    does): the nav2 nodes carry no namespace= themselves, so without the
    push every node comes up global and reads the wrong params section.
    """
    rover_description_dir = get_package_share_directory('rover_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('rover_namespace').perform(context)
    slam = LaunchConfiguration('slam').perform(context)
    autostart = LaunchConfiguration('autostart').perform(context)
    use_composition = LaunchConfiguration('use_composition').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    autostart_bool = autostart.lower() in ('true', '1', 'yes')

    params_source = os.path.join(rover_description_dir, 'config', 'nav2_rover.yaml')
    with open(params_source, 'r') as f:
        content = f.read()
    content = content.replace('<robot_namespace>', namespace)

    data = yaml.safe_load(content)
    slam_block = data.pop('slam_toolbox', {})
    data_out = {'slam_toolbox': {'ros__parameters': {}}}  # dummy top-level key
    data_out[namespace] = {'slam_toolbox': slam_block}
    data_out.update(data)

    with tempfile.NamedTemporaryFile(
            mode='w', suffix='.yaml', prefix=f'nav2_{namespace}_', delete=False) as f:
        yaml.safe_dump(data_out, f)
        params_file = f.name

    if slam.lower() not in ('true', '1', 'yes'):
        return []

    navigation_launch = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')

    # Everything runs inside the rover namespace. bringup_launch.py achieves
    # this with PushRosNamespace around its includes; the nav2 nodes carry no
    # namespace= themselves, so this wrapper must be replicated here or every
    # node comes up global and reads the wrong params file section.
    actions = [GroupAction([
        PushRosNamespace(namespace=namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': autostart,
                'use_composition': use_composition,
                'use_respawn': 'True',
                'log_level': 'info',
            }.items()),

        # slam_toolbox, respawned by the launch: the teleop R key kills this
        # process and it comes back with an empty pose graph (fresh map,
        # identity map -> odom). Requires composition disabled, like the rest
        # of the stack. use_sim_time must be a REAL bool here — the string
        # form crashes slam_toolbox with InvalidParameterTypeException.
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[params_file, {'use_sim_time': True}]),

        # map_saver + its lifecycle manager, replicating slam_launch.py.
        Node(
            package='nav2_map_server',
            executable='map_saver_server',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[ParameterFile(
                RewrittenYaml(
                    source_file=params_file,
                    root_key=namespace,
                    param_rewrites={'use_sim_time': use_sim_time},
                    convert_types=True),
                allow_substs=True)]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': autostart_bool},
                        {'node_names': ['map_saver']}]),
    ])]
    return actions


def generate_launch_description():
    """Launch one full Nav2 + slam_toolbox stack inside a rover namespace.

    Include this once per rover (e.g. from nav2_rover.launch.py or
    rover_swarm.launch.py). The stack runs inside /<rover_namespace> so every
    topic/action/service is scoped there and the velocity smoother's output
    lands on /<rover_namespace>/cmd_vel — exactly the cmd_vel_arbiter
    autonomy input for that rover. No cmd_vel relay needed.

    Slam node is owned by this launch with respawn=True (humble's bringup
    chain cannot respawn it), so the teleop R reset can kill slam_toolbox to
    clear the map: the launch restarts it empty.

    TF: the sim publishes on the global /tf; the parent launch must add one
    scripts/tf_relay.py (--namespaces <all rovers>) so the namespaced stack
    sees the rover's transforms on /<ns>/tf and /<ns>/tf_static.

    Frames must be <rover_namespace>/xxx WITHOUT a leading slash, so this
    launch substitutes '<robot_namespace>' -> namespace itself instead of
    letting nav2_bringup prepend '/' (which tf2 rejects).
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            'rover_namespace', default_value='rover_0',
            description='Rover namespace this Nav2 stack runs in '
                        '(topics/actions/services all get /<rover_namespace> prefixed)'),

        DeclareLaunchArgument(
            'slam', default_value='True',
            description='Run slam_toolbox (True) or launch no nav2 stack at all '
                        '(False — AMCL localization is not supported here)'),

        DeclareLaunchArgument(
            'autostart', default_value='True',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'use_composition', default_value='False',
            description='Use composed bringup if True (separate processes are '
                        'easier to debug and respawn)'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Use simulated (Gazebo) clock'),

        OpaqueFunction(function=_substitute_namespace),
    ])