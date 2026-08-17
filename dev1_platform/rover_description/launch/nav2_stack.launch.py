import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _substitute_namespace(context):
    """Build the params file for one rover namespace.

    Two nav2_bringup quirks are worked around here:

    1. bringup_launch.py's ReplaceString expands '<robot_namespace>' to
       '/'+namespace, which puts a leading slash on tf FRAME values
       ('/rover_0/odom') — and tf2 rejects frame ids starting with '/'
       ('Invalid frame ID ... cannot start with a '/'). So the token is
       replaced with the bare namespace instead ('rover_0').

    2. slam_toolbox is launched by slam_launch.py with the *unwrapped*
       params file (slam_params_file), so a namespaced node reads its
       params from the '<namespace>.slam_toolbox' section — which the
       top-level-key file doesn't have. We emit a '<namespace>.'-prefixed
       copy of the slam_toolbox block (plus a dummy top-level
       'slam_toolbox:' key so HasNodeParams passes the file through).
       All other blocks keep top-level keys: navigation_launch.py re-roots
       them under '<namespace>:' itself via RewrittenYaml, and map_saver
       is wrapped by slam_launch.py's own RewrittenYaml.
    """
    rover_description_dir = get_package_share_directory('rover_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('rover_namespace').perform(context)
    slam = LaunchConfiguration('slam').perform(context)
    autostart = LaunchConfiguration('autostart').perform(context)
    use_composition = LaunchConfiguration('use_composition').perform(context)

    params_source = os.path.join(rover_description_dir, 'config', 'nav2_rover.yaml')
    with open(params_source, 'r') as f:
        content = f.read()
    content = content.replace('<robot_namespace>', namespace)

    data = yaml.safe_load(content)
    slam_block = data.pop('slam_toolbox', {})
    data_out = {'slam_toolbox': {'ros__parameters': {}}}  # HasNodeParams needs the key
    data_out[namespace] = {'slam_toolbox': slam_block}
    data_out.update(data)

    with tempfile.NamedTemporaryFile(
            mode='w', suffix='.yaml', prefix=f'nav2_{namespace}_', delete=False) as f:
        yaml.safe_dump(data_out, f)
        params_file = f.name

    bringup_launch = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': 'True',
            'slam': slam,
            'map': '',
            'use_sim_time': 'True',
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': 'False',
            'log_level': 'info',
        }.items())]


def generate_launch_description():
    """Launch one full Nav2 + slam_toolbox stack inside a rover namespace.

    Include this once per rover (e.g. from nav2_rover.launch.py or
    rover_swarm.launch.py). The stack runs with use_namespace:=True so every
    topic/action/service is scoped to /<rover_namespace> and the velocity
    smoother's output lands on /<rover_namespace>/cmd_vel — exactly the
    cmd_vel_arbiter autonomy input for that rover. No cmd_vel relay needed.

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
            description='Whether to run slam_toolbox (map built on the fly) or load a map'),

        DeclareLaunchArgument(
            'autostart', default_value='True',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'use_composition', default_value='False',
            description='Use composed bringup if True (separate processes are easier to debug)'),

        OpaqueFunction(function=_substitute_namespace),
    ])
