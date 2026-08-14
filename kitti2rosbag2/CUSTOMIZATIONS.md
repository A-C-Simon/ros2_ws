# Customizations (vs. upstream kitti2rosbag2)

This document records every change made to this copy of
[kitti2rosbag2](https://github.com/bharadwajsirigadi/kitti2rosbag2)
(bharadwajsirigadi, Apache-2.0) for the KITTI → ROS 2 bag pipeline used with
Swarm-SLAM. **Diff basis:** upstream `main`, cloned shallow 2026-08-14.

Regenerate the diff:

```bash
cd /home/ac/ros2_ws/src
git clone --depth 1 https://github.com/bharadwajsirigadi/kitti2rosbag2.git /tmp/upstream-kitti2rosbag2
diff -rq /tmp/upstream-kitti2rosbag2 kitti2rosbag2 -x .git -x __pycache__ -x '*.pyc'
```

---

## New features (the big ones)

### 1. Velodyne point clouds — `/velodyne_points`

Upstream only converted images + ground-truth odometry. This copy adds
`velodyne: true` support:

- `KITTIOdometryDataset.velodyne_scans()` / `read_velodyne_scan()` — reads
  `data_odometry_velodyne` `.bin` files (new `velodyne_dir` param; leave `''`
  to look under `data_dir/sequences/XX/velodyne`).
- `rec_pointcloud_msg()` publishes `sensor_msgs/PointCloud2` with fields
  `x, y, z, intensity`, frame `velodyne`, stamped from KITTI `times.txt`.
- Scans and images are index-aligned; the recorder stops at whichever runs out
  first (`counter_limit = min(...)`).

### 2. IMU + GPS from the raw dataset — `/imu/data`, `/gps/fix`

The odometry benchmark ships no IMU/GPS — that data only exists in KITTI's
separate **raw (synced+rectified)** drives, as per-frame OXTS files. New
`imu: true` / `raw_dir` params:

- `RAW_SEQUENCE_MAP` in `kitti_utils.py` — the devkit's odometry-sequence →
  raw-drive mapping (covers sequences 00–10; 11–21 have no mapping).
- `oxts_files()` / `read_oxts()` — reads `oxts/data/*.txt` per the devkit's
  `dataformat.txt` (`OXTS_FIELDS`), frame `i` of the sequence = raw frame
  `start + i`.
- `rec_imu_msg()` publishes `sensor_msgs/Imu` (frame `imu_link`): roll/pitch/
  yaw are absolute ENU-convention, so `Rz(yaw)·Ry(pitch)·Rx(roll)` carries over
  directly; accelerations/rates are already body-frame.
- `rec_gps_msg()` publishes `sensor_msgs/NavSatFix` (frame `imu_link`).
- Covariances left unknown (OXTS reports only pos/vel accuracy, not
  orientation/IMU accuracy).

### 3. TF tree in the bag — `/tf`

Upstream recorded no TF at all. This copy records real extrinsics:

- `odom → velodyne` — static mount, derived once from `calib.txt`'s
  `Tr` (velo→cam0) with the frame permutation below.
- `velodyne → imu_link` — from the raw archive's `calib_imu_to_velo.txt` when
  present (per-date file, treated as optional); otherwise IMU/GPS still publish
  without a TF link.
- `map → odom` — per-frame ground truth, so RViz can place the cloud.

### 4. Corrected vehicle-frame convention

Upstream's odometry had an ad-hoc sign-flip mapping
(`-x, -y, -z` position, wrong quaternion order). This copy applies a single
permutation matrix `R_perm` (KITTI camera axes x-right/y-down/z-forward → ROS
body x-forward/y-left/z-up) to **both** position and orientation of the ground
truth, so "local +X" actually matches the direction of travel and position and
attitude stay consistent with each other. Quaternions are consistently
`[w, x, y, z]` throughout (odom, path, TF).

## Reliability & lifecycle fixes

- `main()` now uses a `spin_once` loop with a `done` flag instead of
  `rclpy.spin` + `rclpy.shutdown()` from inside a callback — the old pattern
  hangs. `KeyboardInterrupt` closes the bag cleanly; a partial bag is still
  readable (`metadata.yaml` is written in `finally`).
- Bag-dir-already-exists is now an explicit error (`done = True` + message)
  instead of a silent `rclpy.shutdown()`.
- Image, camera_info, odom and path messages now carry proper `header.stamp`
  from KITTI timestamps (upstream wrote un-stamped messages).
- New `close_bag()` flushes the sqlite file and writes `metadata.yaml`.

## Bug fixes

- **Install paths** — upstream installed `launch/` and `config/` flat into
  `share/<pkg>/`, and the launch file looked for `share/<pkg>/params.yaml`
  (missing `config/`). `setup.py` now installs them under
  `share/<pkg>/launch` and `share/<pkg>/config`, and the launch file's path
  includes `'config'`.
- **`quaternion.py`** — `rotationmtx_to_quaternion()` rewritten with
  Shepperd's method: picks the largest diagonal element to avoid dividing by a
  near-zero term (upstream's `t > 0` branch could produce garbage for large
  rotations), always returns a normalized `[w, x, y, z]`.
- **New optional params** are declared with defaults (`velodyne: false`,
  `imu: false`, `velodyne_dir: ''`, `raw_dir: ''`) so configs written before
  this support still load.
- `package.xml` — added `sensor_msgs`, `cv_bridge`, `tf2_msgs` depends.

## New files

| File | Purpose |
|---|---|
| `kitti2rosbag2/patch_tf.py` | One-off migration: copies an existing bag into a new one adding `/tf` and corrected `/car/base/odom` + `/car/base/odom_path`. Images/clouds copy through as raw serialized bytes — a cheap sqlite copy + numpy pass over poses, no re-run of the recorder. |
| `kitti2rosbag2/split_overlapping_bag.py` | Splits one kitti2rosbag2 bag into **two overlapping halves** for multi-robot Swarm-SLAM (cslam) testing — both outputs cover the same middle stretch, guaranteeing genuine spatial overlap for inter-robot loop-closure validation (different KITTI sequences are different drives and may never cross paths). Copies only `/velodyne_points`, `/imu/data`, `/gps/fix`. |
| `scripts/download_kitti_raw_seq_0_1_2.sh` | Fetch helper for the raw drives (IMU/GPS) matching sequences 00–02. |

## Config & wiring changes

- `config/params.yaml` — example config with `velodyne: true`, `imu: true`,
  `velodyne_dir: ''`, and `raw_dir` documented in comments.
- `setup.py` — new console script `split_overlapping_bag`.
- `README.md` — documents the new `imu`/`raw_dir` params and the OXTS/raw
  dataset dependency.

## How this plugs into the workspace

These bags feed Swarm-SLAM directly:

- `Swarm-SLAM/src/cslam_experiments/launch/datasets_experiments/kitti2rosbag2_lidar.launch.py`
  — N robots from N comma-separated bags.
- `Swarm-SLAM/src/cslam_experiments/launch/sensors/bag_kitti2rosbag2.launch.py`
  — plays `/velodyne_points` + `/imu/data` + `/gps/fix` into `/rN/...` (TF is
  deliberately skipped there — see that file's comments).
- `split_overlapping_bag.py`'s docstring shows the exact two-bag cslam command.
