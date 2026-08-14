# Swarm-SLAM × AiRover Integration — Chat Log & Solution

> Session: 2026-08-13 — integrating `/home/ac/ros2_ws/src/Swarm-SLAM` (C-SLAM) with
> `/home/ac/ros2_ws/src/dev1_platform` (AiRover simulation) so both scale from a
> single `max_nb_robots` launch argument.

---

## 1. Request

1. Make the rover launchable from a launch argument inside the Swarm-SLAM repo.
2. Sync the swarm size: `max_nb_robots:=N` in Swarm-SLAM must also spawn `N` rovers.
3. Wire topics/TF internally so the two stacks actually talk.
4. Provide usable launch commands.

---

## 2. What was found (rover side)

The rover package is `rover_description` (a Gazebo Fortress diff-drive rover). Its
main launch file is `launch/simulation.launch.py`, which reads a `fleet:` block from
`config/swarm.yaml` and spawns `num_rovers` rovers namespaced `rover_0`, `rover_1`, …

Relevant per-rover topics (from `platform_docs/TOPIC_PARITY.md` + the launch/URDF):

| Rover topic (`/rover_i/...`) | Type | Rate | cslam consumer |
|---|---|---|---|
| `lidar/points` | `sensor_msgs/PointCloud2` | 5 Hz | `lidar_handler_node.py` `pointcloud` |
| `odom` | `nav_msgs/Odometry` | ~30 Hz | `lidar_handler_node.py` `odom` |
| `imu` | `sensor_msgs/Imu` | 100 Hz | (not used by lidar pipeline) |
| `camera/image` | `sensor_msgs/Image` | 10 Hz | (not used by lidar pipeline) |
| `cmd_vel` | `geometry_msgs/Twist` | — | teleop → sim |

Key facts:

- **cslam's lidar front-end reads odometry off the `odom` topic MESSAGE, not TF** —
  so the *only* wiring needed is `pointcloud` + `odom`. No rtabmap odometry node, no
  `periodic_static_tf` shim, no TF aliases are required for cslam itself.
- **QoS matches**: `ros_gz_bridge` publishes RELIABLE by default (depth 10), and
  `lidar_handler_node.py` already subscribes RELIABLE. (Do *not* apply the rover's
  `config/sensor_qos.yaml` BEST_EFFORT override to the bridge, or the pointcloud
  subscriber would silently stop connecting.)
- **Clock**: every rover node runs on `use_sim_time` off Gazebo `/clock`, so cslam
  must do the same.
- **TF**: rover frames hang off `world` (`world → rover_i/odom → rover_i/base_link →
  sensors`). cslam broadcasts its own separate tree (`robot{i}_map`, etc.), which is
  fine — cslam never reads rover TF.

---

## 3. Files created / modified (all inside Swarm-SLAM)

| File | Purpose |
|---|---|
| `src/cslam_experiments/config/rover_lidar.yaml` | cslam config: `use_sim_time: true`, lidar pipeline tuned for the indoor rover world (voxel 0.3, ScanContext threshold 0.8, 60 ICP inliers). |
| `src/cslam_experiments/launch/cslam/cslam_rover_lidar.launch.py` | Per-robot cslam stack (3 nodes) with `pointcloud → /rover_i/lidar/points` and `odom → /rover_i/odom` remappings. |
| `src/cslam_experiments/launch/datasets_experiments/rover_swarm.launch.py` | Single entry point. `max_nb_robots` → rover `num_rovers` AND N × cslam. |
| `src/cslam_experiments/package.xml` | Added `rover_description` + `python3-yaml` exec depends. |

### The sync logic

```
rover_swarm.launch.py  max_nb_robots:=N
 ├─ rover_description/simulation.launch.py     num_rovers:=N      (rover_0 .. rover_N-1)
 └─ N × cslam_rover_lidar.launch.py            robot_id=i, /r{i}  (reads rover_i sensors)
```

The `rover_i ↔ r{i}` mapping is read live from the rover's own `swarm.yaml`
(`namespace_prefix`, `first_index`), so it stays correct if those change.

---

## 4. Build + source (performed)

The initial `--packages-up-to rover_description cslam_experiments` hit a
**pre-existing** failure in `ros_gz_sim` (missing `absl::*` targets — a linuxbrew
abseil conflict, unrelated to this task). Adapted to build only the package that
actually changed:

```bash
cd /home/ac/ros2_ws
colcon build --symlink-install --packages-select cslam_experiments
```

Result:

```
Starting >>> cslam_experiments
Finished <<< cslam_experiments [1.77s]
Summary: 1 package finished [2.16s]
```

Then sourced and verified:

```bash
source install/setup.bash
ros2 pkg prefix cslam_experiments   # → /home/ac/ros2_ws/install/cslam_experiments
ros2 pkg prefix rover_description   # → /home/ac/ros2_ws/install/rover_description
```

Confirmed all three new files are installed, and both launch files load cleanly:

```bash
ros2 launch cslam_experiments rover_swarm.launch.py --show-args
ros2 launch cslam_experiments cslam_rover_lidar.launch.py --show-args
```

---

## 5. Usable launch commands

```bash
cd /home/ac/ros2_ws
source install/setup.bash

# 1 rover  ==  1 cslam robot
ros2 launch cslam_experiments rover_swarm.launch.py max_nb_robots:=1

# 3 rovers  ==  3 cslam robots  (single synced argument)
ros2 launch cslam_experiments rover_swarm.launch.py max_nb_robots:=3

# custom world
ros2 launch cslam_experiments rover_swarm.launch.py \
    max_nb_robots:=3 world:=/path/to/world.sdf
```

Drive the fleet from the **"Fleet Teleop"** terminal: keys `1..N` switch active
rover, `b` broadcasts a twist to all, arrow keys drive. As rovers revisit the same
places, cslam detects intra/inter-robot loop closures and fuses the pose graphs over
`/cslam/*` topics.

---

## 6. Caveats

- **`full_sensors_all:=false` breaks SLAM on follower rovers** — "lite" rovers have
  their lidar stripped, so their cslam robot gets no pointcloud. Keep it `true`/empty
  for SLAM testing.
- **GUI required** — the rover launch brings up Gazebo + RViz + gnome-terminal
  teleop/e-stop. It is an interactive test rig, not headless.
- **Sim odometry is perfect** (no drift/noise) — good for validating the cslam
  pipeline; real drift needs the rover team's IMU fusion or re-added sensor noise.
- **cslam merged-map TF** (`robot{i}_map`, etc.) is a separate tree from the rover's
  `world` frame; the rover RViz shows the fleet, while merged-map visualization would
  use `cslam_visualization` separately.
- The `ros_gz_sim` abseil build error is pre-existing and unrelated — `rover_description`
  was already installed, so only `cslam_experiments` needed rebuilding.
