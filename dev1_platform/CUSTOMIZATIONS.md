# Customizations — original platform, development notes

Unlike the other packages in this workspace, **dev1_platform has no upstream
repository** — it is an original platform built for the AiRover mesh-rover
swarm project ("Dev 1 (Platform/Hardware + Simulation)"). This file records
what it is, what it vendors, and how it plugs into the rest of the workspace.
The package's own `README.md` covers usage, layout, build and the sim-to-real
interface; `rover_description/platform_docs/` holds the detailed reference
(`ARCHITECTURE.md`, `PARAMETER_CONFIG_GUIDE.md`, `TOPIC_PARITY.md`).

---

## What it is

A Gazebo Fortress simulation platform that spawns a fleet of **1–10
namespaced rovers** (`/rover_0`, `/rover_1`, …) from a single configuration
file (`rover_description/config/swarm.yaml`), with a **fixed topic/TF
contract** so that mapping and navigation code written against simulation runs
unchanged against the physical rover. Per rover: Livox-style lidar (5 Hz),
camera (320×240 @ 10 Hz), IMU (100 Hz), 6 sonars, diff-drive odometry
(30 Hz), `cmd_vel` teleop, battery, docking and fleet-wide emergency stop.

## Vendored: `livox_msgs`

A standalone package holding just the Livox custom message definitions
(`msg/CustomMsg.msg`, `msg/CustomPoint.msg`). The field layout is
**byte-compatible with `livox_ros_driver2`** (verified by diff — only comments
differ), so downstream Livox-optimised SLAM (FAST-LIO2, Point-LIO, LIO-SAM
Livox variant) can subscribe without code changes. It is vendored here so the
sim does not require a full `livox_ros_driver2` install; the rover's
`livox_publisher.py` emits `livox_msgs/CustomMsg` alongside the
`PointCloud2` used by the mapping stack.

## `rover_description` — the platform package

| Area | Contents |
|---|---|
| `urdf/rover.urdf` | Namespace **template**: every topic/frame/plugin reference carries a `__NS__` sentinel substituted per rover at launch. |
| `launch/simulation.launch.py` | Spawns the whole fleet from `config/swarm.yaml`; generates per-rover RViz configs, bridges, teleop/estop/dock/battery/diagnostics rosters. |
| `config/swarm.yaml` | **The fleet knob**: `num_rovers`, `namespace_prefix`, `first_index`, `full_sensors_all`, spawn poses / line layout, motion & sensor-rate reference values (comments). |
| `config/sensor_qos.yaml` | BEST_EFFORT QoS overrides for sensor topics (used by converter nodes, not the bridge — see README note). |
| `worlds/` | `test_station.sdf` (indoor test zone with docking station), `crop_field.sdf` (crop rows). |
| `scripts/` | `livox_publisher.py`, `sonar_to_range.py`, `sensor_frame_aliases.py`, `battery_publisher.py`, `dock_monitor.py`, `estop_manager.py`, `diagnostics_aggregator.py`, `rover_teleop.py` (fleet teleop: `1..N` select, `b` broadcast), `recenter_wheels.py`, `rover_session_watchdog.py`, `kill_rover_session.sh`. |
| `rviz/rover.rviz` | RViz template, cloned per rover at launch. |
| `platform_docs/` | `ARCHITECTURE.md` (tier 1→6 data flow), `PARAMETER_CONFIG_GUIDE.md` (every tunable value), `TOPIC_PARITY.md` (the sim-to-real contract), `Rover Architecture.pdf`. |
| `meshes/`, `models/` | Rover meshes and the `test_zone` building model. |

## Design decisions worth knowing

- **Namespacing**: one `__NS__` sentinel drives Ignition topic strings, ROS 2
  namespaces and TF `frame_prefix` together — one rover costs one config line.
- **`/clock` and `/tf` are deliberately global** (one clock, one TF tree);
  everything else is per-rover under `/rover_i/`.
- **QoS split**: `ros_gz_bridge` publishes RELIABLE; converter nodes republish
  BEST_EFFORT — subscribers should use `SensorDataQoS` (see TOPIC_PARITY.md).
- **Known gaps** (documented in README): perfect simulated odometry (no drift),
  uniform Livox scan grid vs the Mid-360's rosette sweep, camera downscaled to
  hold real-time factor > 1.0, sensor noise models documented but not applied.

## Workspace integration

- **Swarm-SLAM** — `Swarm-SLAM/src/cslam_experiments` depends on this package
  (`rover_description` exec_depend). `rover_swarm.launch.py` is the single
  entry point: `max_nb_robots:=N` spawns N rovers **and** N cslam stacks,
  reading `namespace_prefix`/`first_index` live from `config/swarm.yaml`.
  `config/rover_lidar.yaml` (in cslam_experiments) tunes Swarm-SLAM's lidar
  pipeline to this rover's sensor profile. See
  `Swarm-SLAM/ROVER_SWARM_INTEGRATION.md` for the full integration record.
- **Topic contract** — the mapping stack subscribes to `/rover_i/lidar/points`
  (PointCloud2) and `/rover_i/odom` (Odometry); both are pinned by
  `TOPIC_PARITY.md` so the physical rover can replace the sim without code
  changes.
- **ROS_DOMAIN_ID 42 / ROS_LOCALHOST_ONLY=1** are the documented network
  settings for multi-machine fleet runs.
