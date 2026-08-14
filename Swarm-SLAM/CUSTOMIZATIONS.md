# Customizations (vs. upstream Swarm-SLAM)

This document records every change made to this copy of
[Swarm-SLAM](https://github.com/MISTLab/Swarm-SLAM) (MISTLab, MIT License) for
the AiRover / dev1_platform workflow. **Diff basis:** upstream `main`, cloned
shallow 2026-08-14. `cslam.repos`, `README.md`, `LICENSE.md`,
`build-everything.sh` and `requirements.txt` are unmodified upstream files.

Regenerate the diff:

```bash
cd /home/ac/ros2_ws/src
git clone --depth 1 https://github.com/MISTLab/Swarm-SLAM.git /tmp/upstream-Swarm-SLAM
diff -rq /tmp/upstream-Swarm-SLAM Swarm-SLAM -x .git -x src -x media
```

The populated `src/` sub-packages diff against their own upstreams
(`lajoiepy/cslam*`, `MIT-SPARK/TEASER-plusplus`).

---

## Top-level additions

| File | What it is |
|---|---|
| `USAGE.md` | Personal runbook: the 3-nodes-per-robot architecture, build steps, single- and multi-robot KITTI simulation commands, real-hardware / Zenoh-bridge notes. |
| `ROVER_SWARM_INTEGRATION.md` | Chat-log/solution record of the rover-swarm integration session. Read this first — it explains the reasoning behind all `rover_*` files below. |

## src/cslam_experiments — most of the work

### New: `config/rover_lidar.yaml` — cslam config for the rover sim

Lidar pipeline config for `rover_description`'s Livox-style gpu_lidar
(360° × 16 rings @ 5 Hz, ~5760 pts/scan) and diff-drive odometry. Topic names
are relative (`pointcloud`, `odom`); the launch file remaps them per rover.

| Parameter | rover_lidar.yaml | KITTI default | Why |
|---|---|---|---|
| `use_sim_time` | `true` | false | rover sim drives everything off Gazebo `/clock` |
| `voxel_size` | `0.3` | `0.5` | indoor-scale world — finer voxel keeps keyframe clouds dense for ICP |
| `registration_min_inliers` | `60` | `100` | mid-size clouds; more forgiving, still rejects garbage |
| `similarity_threshold` | `0.8` | `0.9` | small repeatable indoor env — more ScanContext candidates |
| `keyframe_generation_ratio_distance` | `0.5` | — | denser keyframes for a slow-moving diff-drive rover |
| `global_descriptor_technique` | `scancontext` | — | lidar-only front end |

Also sets `enable_intra_robot_loop_closures: true` and keeps the
backend/visualization blocks from the KITTI config.

### New: `launch/cslam/cslam_rover_lidar.launch.py`

The 3-node cslam stack for **one** rover (`loop_closure_detection_node.py`,
`lidar_handler_node.py` as `cslam_map_manager`, `pose_graph_manager`) in
namespace `/r{i}`. Only real difference from `cslam_lidar.launch.py`: two
remappings on the map manager — `pointcloud → /rover_i/lidar/points` and
`odom → /rover_i/odom`. cslam reads odometry off the odom topic *message*
(not TF), so those two remaps are the entire sensor wiring.

### New: `launch/datasets_experiments/rover_swarm.launch.py`

The single entry point merging the two repos into one launch argument:
`max_nb_robots:=N` spawns `N` rovers **and** `N` cslam stacks. It reads the
fleet block (`namespace_prefix`, `first_index`) live from
`rover_description/config/swarm.yaml`, so the `rover_i ↔ r{i}` mapping stays
correct if those change. Extra args: `world` (default
`rover_description/worlds/test_station.sdf`), `config_file`,
`full_sensors_all`, `enable_simulated_rendezvous`.

### New: `launch/datasets_experiments/kitti2rosbag2_lidar.launch.py`

Multi-robot **dataset** variant: `bag_files` takes N comma-separated
kitti2rosbag2 bag paths (one per robot), spins up N × `cslam_lidar.launch.py`
plus bag playback and rtabmap odometry, all synced by `max_nb_robots`.

### New: `launch/sensors/bag_kitti2rosbag2.launch.py`

Bag player for kitti2rosbag2 bags. Plays only `/velodyne_points`, `/imu/data`,
`/gps/fix`, remapped into `/rN/...`. Deliberately:
- **no `--clock`** — `ros2 bag play --clock` doesn't scale `/clock` by `-r` in
  this distro, so at rate ≠ 1.0 it drifts from the bag timestamps;
- **no `/tf`** — kitti2rosbag2's recorded `odom` is the vehicle-body frame, not
  `icp_odometry`'s dead-reckoning odom; playing it back would fight the
  odometry node's own broadcast. Fixed identity mounts are substituted via
  `periodic_static_tf.py` instead.

### New: `cslam_experiments/periodic_static_tf.py`

Re-broadcasts fixed identity transforms on `/tf` stamped to match **incoming
sensor messages** instead of the node clock — sidesteps the clock-domain
drift above entirely. Subscription depth 1 on purpose (keeps it serving the
newest cloud, matching rtabmap's `topic_queue_size=1`); measures in the
docstring show this fixed 1464 extrapolation gaps over two full runs.

### Modified

- `package.xml` — added `exec_depend`: `rover_description`, `python3-yaml`.
- `CMakeLists.txt` — install `periodic_static_tf.py`.
- `launch/odometry/rtabmap_kitti_lidar_odometry.launch.py` — (1) new
  `tf_topic`/`tf_static_topic` args (default `/tf`, `/tf_static`) remapped onto
  the rtabmap node so multi-robot setups can isolate TF per namespace
  (e.g. `/r0/tf`); (2) fixed a bug where `'--ros-args'` + `'--log-level'`
  silently concatenated into one token, so `log_level` was never applied.
- `launch/robot_experiments/experiment_ouster_realsense.launch.py` — comments
  only: notes for switching to `cslam_lidar.launch.py` / commenting out the
  camera block for lidar-only runs.

## src/cslam — performance & reliability fixes

- **`cslam/lidar_handler_node.py`** —
  - QoS `RELIABLE` (upstream: `BEST_EFFORT`). A KITTI velodyne cloud is ~2 MB;
    UDP fragments it into ~64 kB datagrams that overrun the default
    `net.core.rmem_max` socket buffer, and one lost fragment silently drops the
    whole message under BEST_EFFORT. Measured on KITTI seq 02 @ rate 0.8: 13%
    intake → full 7.7 msg/s, odom→keyframe conversion 34% → ~99.7%.
  - Keyframe storage switched from Open3D clouds to compact float32 arrays
    (`downsample_ros_pointcloud_to_array`): the `local_descriptors_map` holds
    every keyframe for the whole run (loop closures reach 880–1265 keyframes
    back), and `extract_fpfh()` attaches in-place float64 normals to any cloud
    it touches — arrays avoid ~2× memory plus the normals bloat.
- **`cslam/lidar_pr/icp_utils.py`** — new `points_to_open3d()`,
  `points_to_ros()`, `downsample_ros_pointcloud_to_array()` (the
  `open3d_to_ros`/`ros_to_open3d` helpers are kept for compatibility);
  explicit `import rclpy.logging` so `solve_teaser()`'s failure-path logging
  doesn't raise `AttributeError`.
- **`cslam/lidar_pr/scancontext_matching.py`** — `query()` now returns the
  **top-k** candidates *with* similarity scores instead of a single best match
  (callers skip candidates too close in time to be useful loop closures); also
  caps the KD-tree query at `min(num_candidates, nb_items)` — scipy pads
  out-of-range queries with the preallocated all-zero descriptors.
- **`cslam/lidar_pr/scancontext_utils.py`** — `ptcloud2sc()` vectorised
  (per-point Python loop was ~330 ms/keyframe, the 2nd largest cost in the
  loop-closure detector); bit-identical output verified against the original
  on random/KITTI-like/degenerate inputs.
- **`src/back_end/decentralized_pgo.cpp`** — `RCLCPP_WARN` logged when an
  inter-robot loop closure is accepted, so merges are visible in the console
  without a debugger.

## src/cslam_interfaces — unchanged

## src/cslam_visualization — split RViz view

- `config/lidar.rviz` — reworked panel layout (cosmetic).
- New `config/raw_lidar.rviz` + `launch/visualization_raw_lidar.launch.py` —
  a second RViz showing **one** robot's raw odometry/lidar in its own `odom`
  frame (`robot:=r1` to pick). Needed because cslam runs two disconnected TF
  trees (`robotX_map` family vs `/rN` `odom → base_link → velodyne`) that no
  transform ever links, and RViz has a single fixed frame. Can run alongside
  `visualization_lidar.launch.py` to see both at once.

## src/TEASER-plusplus — vendored, unmodified

Third-party dependency (MIT-SPARK) vendored per upstream's `cslam.repos`
workflow; prebuilt locally. No local edits.

---

## Theme summary

1. **One-knob rover swarm**: `max_nb_robots` drives fleet size AND the number
   of cslam stacks, with the `rover_i ↔ r{i}` mapping read live from the
   rover's own config.
2. **Rover-tuned SLAM params** for an indoor, slow, lidar-only diff-drive
   platform (`rover_lidar.yaml`).
3. **Bag-driven multi-robot testing** for both the rover sim and KITTI
   (kitti2rosbag2 bags), including TF isolation per namespace.
4. **Robustness/perf work in cslam**: RELIABLE QoS fix, memory-efficient
   keyframe storage, vectorised ScanContext, top-k loop-closure candidates.
5. **Diagnosability**: inter-robot closure logging, split RViz views, and a
   personal USAGE runbook.
