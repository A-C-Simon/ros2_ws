# Dev 1 Platform: multi-rover simulation and sim-to-real interface

Simulation platform for the AiRover swarm. It spawns a fleet of 1 to 10 namespaced
rovers from a single configuration file, with the full sensor set, docking, battery,
emergency stop and fleet teleoperation, and it publishes a fixed topic and TF contract
so that mapping and navigation code written against simulation runs unchanged against
the physical rover.

## Environment

| Item | Version |
|---|---|
| OS | Ubuntu 22.04 LTS |
| ROS 2 | Humble |
| Simulator | Ignition Gazebo Fortress |
| Build tool | colcon |

## Layout

```
dev1_platform/
├── rover_description/
│   ├── urdf/rover.urdf          rover model, namespace template (see below)
│   ├── launch/simulation.launch.py   spawns the whole fleet
│   ├── config/swarm.yaml        THE fleet knob: size, poses, sensor policy
│   ├── config/sensor_qos.yaml   BEST_EFFORT sensor QoS overrides
│   ├── worlds/crop_field.sdf    crop row world with docking station
│   ├── scripts/                 sonar, livox, battery, dock, estop, diagnostics, teleop
│   ├── rviz/rover.rviz          RViz template, cloned per rover at launch
│   └── platform_docs/           full documentation, start here
└── livox_msgs/                  Livox CustomMsg definition, needed to build
```

## Build

```bash
# from the workspace root containing this folder
colcon build --symlink-install
source install/setup.bash
```

`livox_msgs` must build before `rover_description`, which colcon handles automatically.

## Run

```bash
# default fleet size, taken from config/swarm.yaml
ros2 launch rover_description simulation.launch.py

# override fleet size on the command line
ros2 launch rover_description simulation.launch.py num_rovers:=5
```

This brings up Gazebo, one RViz configured for the whole fleet, and a teleop terminal.

### Changing the fleet

Everything scales from `config/swarm.yaml`. Set `num_rovers` and, if you want explicit
placement, add entries under `spawn_poses`. Rovers without an explicit pose fall back to
a line layout with collision avoidance. Nothing else needs editing: topics, TF frames,
teleop slots, dock, battery and diagnostics rosters and the generated RViz config all
follow.

`full_sensors_all: false` drops non-scout rovers to a lite sensor set (IMU, camera,
front sonar) if the machine cannot sustain full sensors at high fleet counts.

## Interface

Every rover publishes under `/rover_i/`. Frames are prefixed `rover_i/`. `/clock` and
`/tf` are deliberately global, because simulation time is one clock and TF is one tree.

| Topic | Type | Rate |
|---|---|---|
| `/rover_i/cmd_vel` | `geometry_msgs/Twist` | 20 Hz |
| `/rover_i/odom` | `nav_msgs/Odometry` | 30 Hz |
| `/rover_i/lidar` | `sensor_msgs/LaserScan` | 5 Hz |
| `/rover_i/lidar/points` | `sensor_msgs/PointCloud2` | 5 Hz |
| `/rover_i/livox/lidar` | `livox_msgs/CustomMsg` | 5 Hz |
| `/rover_i/camera/image` | `sensor_msgs/Image` | 10 Hz |
| `/rover_i/imu` | `sensor_msgs/Imu` | 100 Hz |
| `/rover_i/sonar/DIR/range` | `sensor_msgs/Range` | 5 Hz |
| `/rover_i/battery_state` | `sensor_msgs/BatteryState` | 1 Hz |

Fleet wide, no namespace: `/clock`, `/tf`, `/tf_static`, `/emergency_stop`,
`/dock_0/status`, `/diagnostics`.

**The full contract is `rover_description/platform_docs/TOPIC_PARITY.md`. Read it before
writing any subscriber.** It fixes topic names, message types and frame ids so that the
simulated rover can be swapped for the physical one without changing subscriber code.

## Network setup for integration

The simulation defaults to an isolated ROS graph:

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
```

That isolation is intentional for solo development. For integration across machines both
must change:

- All machines that need to see each other must use the **same** `ROS_DOMAIN_ID`. Different
  ids means the graphs cannot see each other at all, which looks like a broken driver but
  is only configuration.
- `ROS_LOCALHOST_ONLY` must be `0`, otherwise nothing leaves the machine.


## Documentation

| Document | Contents |
|---|---|
| `platform_docs/ARCHITECTURE.md` | system diagram and data flow, tier by tier |
| `platform_docs/PARAMETER_CONFIG_GUIDE.md` | every tunable value: geometry, motion limits, sensor rates, fleet, dock, battery, QoS |
| `platform_docs/TOPIC_PARITY.md` | the sim-to-real topic and frame contract |

## Namespacing

The URDF is a template. Every topic, frame and plugin reference contains a `__NS__`
sentinel, and the launch file substitutes the namespace per rover before spawning, so
one rover costs one line of configuration rather than a code change. Three layers have to
agree and are handled together: Ignition topic strings, ROS 2 namespaces, and TF frame
prefixes via `frame_prefix` on `robot_state_publisher`.

The Ignition `Sensors` plugin is hosted once at world level rather than per model. Per
model it multiplies the render pipeline and fails at two or more sensor carrying rovers.

## Verified

- 3 rovers demonstrated, 5 verified under test, architecture supports 10 without redesign
- Namespace isolation confirmed at every fleet size, no topic or TF collisions
- Docking approach, docking and charging states, battery drain and charge
- Fleet wide emergency stop, diagnostics aggregation, single terminal fleet teleoperation
- RViz configuration generated at launch for arbitrary fleet size

## Known gaps

1. Sensor noise models are documented in the parameter guide but not yet applied.
2. Simulated odometry is perfect, with no drift or slip. Real wheel odometry drifts, and
   closing that gap is the main sim-to-real item.
3. The simulated Livox emits a uniform scan grid. The real Mid-360 sweeps a
   non-repeating rosette, so message format matches but point distribution does not.
4. The bridge is RELIABLE underneath while converter nodes publish BEST_EFFORT, so
   subscribers should use `SensorDataQoS`.
5. Camera runs at 320x240 and 10 Hz to hold real time factor above 1.0 with three
   rovers on a CPU bound machine. Raise it on hardware with a GPU.

## Media

`media/` holds screenshots and screen recordings of the fleet running, docking, and the
namespaced topic list, taken from the runs described above.

`media/rosgraph_nodes.png` is the ROS 2 computation graph captured with `rqt_graph` from a
live 2-rover run. It shows the per-rover node groups, the bridges, the converter nodes and
the fleet-wide nodes, with topic names on the edges. `rqt_graph` shows structure only; the
publish rates for each topic are in the table above and in `platform_docs/ARCHITECTURE.md`.
