# Swarm-SLAM Usage Notes

Local notes on how to run this package, covering single- and multi-robot setups for both
simulation (KITTI lidar dataset) and real sensors. See `README.md` for the official project
description/citation and the [hosted documentation](https://lajoiepy.github.io/cslam_documentation/html/index.html)
for the authoritative reference.

## Architecture

Swarm-SLAM (`cslam`) runs **three ROS 2 nodes per robot**, each in a `/rN` namespace:

- `loop_closure_detection_node.py` — front-end: computes global descriptors, finds intra/inter-robot loop closure candidates
- `lidar_handler_node.py` (or `rgbd_handler_node.py` / stereo equivalent) — front-end map/keyframe manager for the given sensor type
- `pose_graph_manager` — back-end: pose graph optimization (GTSAM), publishes the optimized map/TF

Robots communicate loop-closure/pose-graph data over ROS 2 topics. For real multi-robot deployments each robot typically runs its **own `ROS_DOMAIN_ID`** and the graphs are bridged together with **Zenoh** (`zenoh-bridge-ros2dds`), which is the whole point of the "sparse/decentralized" design — bandwidth stays low. An optional `cslam_visualization` node + RViz can run on a base station to monitor progress.

Everything is driven by ROS 2 launch files in `src/cslam_experiments/launch/`, split into:
- `launch/cslam/` — the 3 core nodes for one robot, per sensor type (lidar/rgbd/stereo)
- `launch/datasets_experiments/` — spins up N robots + dataset-bag playback + odometry, for **simulation**
- `launch/robot_experiments/` — spins up cslam + real sensor drivers + Zenoh bridge, for **real hardware**
- `launch/sensors/`, `launch/odometry/` — building blocks included by the above

Every launch file exposes `max_nb_robots` and `robot_id`/`namespace` — that's the single vs. multi-robot switch throughout.

## SLAM algorithm / pipeline

Swarm-SLAM isn't a single algorithm — it's a pipeline of existing components glued together, split across the three nodes above:

- **Front-end odometry**: wraps **RTAB-Map** (`rtabmap`/`rtabmap_msgs`/`rtabmap_conversions`) for local visual/LiDAR odometry and keyframe/map management. Supports stereo, RGB-D, and LiDAR sensing (`front_end/rgbd_handler.cpp`, `stereo_handler.cpp`).
- **Inter-robot place recognition** (`global_descriptor_technique` config, `cslam/vpr/`, `cslam/lidar_pr/`):
  - Visual: **NetVLAD** or **CosPlace** learned global descriptors.
  - LiDAR: **ScanContext** descriptors + ICP for geometric verification.
- **Outlier rejection / robust registration**: **TEASER++** (vendored in `src/TEASER-plusplus`) for certifiable outlier-robust point cloud registration, plus **MAC** (Maximal Clique, `cslam/mac/`) to prune spurious inter-robot loop closures via graph-theoretic consistency.
- **Back-end**: decentralized pose-graph optimization in **GTSAM**, using its `GncOptimizer` (Graduated Non-Convexity) for robustness (`back_end/decentralized_pgo.cpp`, `gtsam_utils.cpp`). Each robot optimizes its own graph, exchanging only sparse loop-closure data — the "sparse decentralized" in the name.

## Build

```bash
cd /home/ac/ros2_ws/src          # colcon workspace root in this setup
colcon build --symlink-install --packages-up-to cslam_experiments cslam_visualization
source install/setup.bash
```
(cslam depends on GTSAM ≥4.1 and TEASER-plusplus, which are vendored/prebuilt in `src/Swarm-SLAM`; `requirements.txt` pip deps — torch, open3d, scikit-learn, etc. — must be installed too.)

## 1. Simulation, single robot (KITTI)

```bash
ros2 launch cslam_experiments kitti_lidar.launch.py \
    max_nb_robots:=1 sequence:=00 config_file:=kitti_lidar.yaml rate:=0.2
```
This launches 1× (cslam nodes + rtabmap lidar-odometry + bag playback) plus the KITTI `velo_link`/`imu_link`→`base_link` static transforms.

## 2. Simulation, multiple robots (KITTI)

```bash
ros2 launch cslam_experiments kitti_lidar.launch.py \
    max_nb_robots:=2 sequence:=00 config_file:=kitti_lidar.yaml \
    rate:=0.2 robot_delay_s:=260 launch_delay_s:=10
```
This spawns `max_nb_robots` copies of the cslam stack (namespaces `/r0`, `/r1`, …), staggered by `robot_delay_s` so each robot's stack initializes before its bag starts playing. Optional: `enable_simulated_rendezvous:=true rendezvous_config:=kitti00_2robots_lidar.config` to simulate limited communication windows (format: `robot_id,start_s,end_s,...`, see `config/rendezvous/`).

**Important gap to fix before this runs**: the bag player (`launch/sensors/bag_kitti.launch.py`) expects each robot to have its **own bag file** at:
```
src/cslam_experiments/data/KITTI<seq>_<N>robots/KITTI<seq>-<robot_id>
```
e.g. for 2-robot sequence 00: `data/KITTI00_2robots/KITTI00-0` and `KITTI00-1`. `data/download.sh` is empty in this checkout, so **these bags aren't provided** — you need to produce them yourself, one bag per robot, typically by splitting a KITTI sequence into time segments (that's what the "2robots"/"5robots" rendezvous configs assume — each robot gets a different portion of the trajectory).

Also note the bag player remaps topics from the **raw KITTI rosbag naming convention** (`/kitti/velo/pointcloud`, `/kitti/oxts/gps/fix`, `/kitti/oxts/imu`, `/kitti/camera_color/...`) — this is the format produced by tools like `kitti2bag`, **not** the `kitti2rosbag2` package already in this workspace (which publishes `velodyne_points`, `car/base/odom`, etc.). If you plan to feed bags from `kitti2rosbag2`, either add matching remaps or adjust `bag_kitti.launch.py`.

### 1b. Simulation, single robot, from a `kitti2rosbag2` bag (lidar-only, no camera)

This is now wired up (`launch/sensors/bag_kitti2rosbag2.launch.py` + `launch/datasets_experiments/kitti2rosbag2_lidar.launch.py`), bridging the gap noted above for bags produced by the `kitti2rosbag2` package already in this workspace:

```bash
ros2 launch cslam_experiments kitti2rosbag2_lidar.launch.py \
    max_nb_robots:=1 bag_files:=/path/to/kitti2rosbag2_bag rate:=0.2
```

`kitti2rosbag2` was later re-engineered (see `src/kitti2rosbag2/config/params.yaml`, `imu:=True`/`raw_dir`) to also publish real IMU (`/imu/data`: orientation+accel+gyro, from the KITTI raw dataset's OXTS files) and GPS (`/gps/fix`), not just `/velodyne_points`. The launch files below were updated accordingly — this section describes the current (IMU/GPS-aware) behavior.

What it does differently from `kitti_lidar.launch.py`, and why:
- Plays back `/velodyne_points`, `/imu/data`, `/gps/fix` (`--topics`), remapped to `/r0/pointcloud`, `/r0/imu/data`, `/r0/gps/fix`. Camera topics are never played — no decoding/transport cost on limited hardware.
- Skips the bag's own `/tf`. `kitti2rosbag2` now records real rigid-mount extrinsics there (`odom`→`velodyne`→`imu_link`, constant per sequence) plus the per-frame ground truth on `map`→`odom` — but that `odom` is the *vehicle body frame*, a different thing from `icp_odometry`'s dead-reckoning `odom`; playing it back would fight `icp_odometry`'s own `odom`→`base_link` broadcast.
- Publishes fixed identity mounts `base_link`→`velodyne` and `base_link`→`imu_link` instead (`cslam_experiments/periodic_static_tf.py`, see below) — mirroring upstream `kitti_lidar.launch.py`'s own simplification of treating sensors as coincident with `base_link` rather than threading through the real extrinsics (upstream doesn't bother with precise KITTI extrinsics either). **Note the direction**: `base_link` is the *parent* of both sensor frames. Upstream's own pattern (`velo_link`→`base_link` *and* `imu_link`→`base_link`, i.e. `base_link` as the child of two different frames) is structurally invalid — a frame can only have one parent — and reproducing it here made `tf2` report "two or more unconnected trees" for whichever sensor lost the race.
- `wait_imu_to_init` now defaults to `true` (exposed as a launch arg) — real IMU data is available, and `icp_odometry` uses it for gravity/roll/pitch initialization. Set it to `false` only for bags recorded with `imu:=False`.
- No `--clock` / `use_sim_time`. That was an earlier approach (still visible in git history) that broke down at any `rate` other than `1.0`: `ros2 bag play --clock` in this ROS 2 distro doesn't scale `/clock` by `-r` — it advances at wall-clock pace regardless of playback rate — so a sim-time-driven broadcaster silently drifted out of sync with the bag's own (KITTI-relative, near-zero) message timestamps and every TF lookup failed. `periodic_static_tf.py` instead subscribes to `/r0/pointcloud` and re-broadcasts the mounts on every message, stamped with **that message's own `header.stamp`** — no clock involved, so nothing to drift. Both mounts are published from one node/callback; a second rclpy process here was enough added CPU contention on constrained hardware to start missing frames and reintroduce TF errors intermittently, so don't split it back out.

Tuning note: `rate:=0.2` (5× slower than real-time) is the upstream default for a reason — `icp_odometry`'s scan queue is depth 1, so on constrained hardware a `rate:=1.0` playback drops frames faster than ICP can register them, producing runaway pose "guesses" and `libpointmatcher` registration failures. Slow the rate down further if you still see `Registration failed` / `Odom tracking failed` messages.

Verified against `/home/ac/Documents/kitti_dataset/my_bag_imu1` (IMU+GPS bag): at `rate:=0.2` odometry tracks cleanly (ICP inlier ratio ~0.5–0.6, no registration/tracking failures, only a single transient TF miss at startup), and keyframes/ScanContext descriptors flow on `/r0/cslam/keyframe_odom` and `/cslam/global_descriptors`; `/r0/gps/fix` also confirmed flowing (unused by default — `kitti_lidar.yaml`'s `evaluation.enable_gps_recording` is `false`; flip it to feed GPS into pose-graph optimization).

Caveat: `periodic_static_tf.py`'s `stamp_topic` is hardcoded to `/r0/pointcloud`, and TF frame names (`base_link`, `velodyne`, `imu_link`) aren't robot-namespaced — same as upstream's own static-TF nodes. This is only correct for `max_nb_robots:=1`; multi-robot would need per-robot frame names, which neither this nor the upstream lidar launch currently does.

## 3. Reality, single robot, real sensors

Per-robot config already exists for a few sensor rigs (`experiment_lidar.launch.py` for a generic lidar + Zenoh only, `experiment_ouster_realsense.launch.py` for Ouster lidar + RealSense + VectorNav IMU, `experiment_oak-d_rgbd.launch.py`, `experiment_realsense.launch.py`). On the robot:

```bash
ros2 launch cslam_experiments experiment_ouster_realsense.launch.py \
    robot_id:=0 max_nb_robots:=1 cslam_config_file:=ouster_lidar.yaml
```
This sets `ROS_DOMAIN_ID=0`, starts the Zenoh DDS bridge, the sensor drivers (`ros2_ouster`, `vectornav`, RealSense), rtabmap lidar-odometry, and the 3 cslam nodes, plus static TFs for the sensor mounting. For a bare lidar without the Ouster/RealSense rig, `experiment_lidar.launch.py` gives you just cslam + Zenoh — you'd add your own driver launch alongside it (this file doesn't start any sensor driver itself).

## 4. Reality, multiple robots, real sensors

Same launch file, run **independently on each robot**, each with its own `robot_id` (which also becomes its `ROS_DOMAIN_ID` — this is what isolates each robot's local DDS traffic) and the same `max_nb_robots`:

```bash
# robot 0
ros2 launch cslam_experiments experiment_ouster_realsense.launch.py robot_id:=0 max_nb_robots:=3
# robot 1
ros2 launch cslam_experiments experiment_ouster_realsense.launch.py robot_id:=1 max_nb_robots:=3
# robot 2
ros2 launch cslam_experiments experiment_ouster_realsense.launch.py robot_id:=2 max_nb_robots:=3
```
The Zenoh bridges on each robot (config: `config/zenoh/zenoh_cslam.json5`) need to reach each other over the network (Wi-Fi/mesh) — that's what carries the sparse inter-robot loop-closure/pose-graph traffic. Optionally, on a base station:
```bash
ros2 launch cslam_visualization visualization_lidar.launch.py
```
to monitor the merged map/pose-graphs live in RViz.

---

Sensor-type configs live in `src/cslam_experiments/config/*.yaml` (`kitti_lidar.yaml`, `ouster_lidar.yaml`, `realsense_rgbd.yaml`, `kitti_stereo.yaml`, …) — pick/copy one matching your sensor and pass it as `config_file`/`cslam_config_file`. Key tunables inside: `frontend.sensor_type`, `similarity_threshold`, `inter_robot_loop_closure_budget`, `global_descriptor_technique` (`scancontext` for lidar, `netvlad` for camera).
