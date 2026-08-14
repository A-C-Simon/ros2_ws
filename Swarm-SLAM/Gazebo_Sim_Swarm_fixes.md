# Gazebo Sim Swarm Fixes

Session log for the Swarm-SLAM Gazebo simulation work on the AI Task2 Rover.

## Objective

Run the rover swarm C-SLAM stack in Gazebo and diagnose a reported mapping bug:
when the rover rotates, the map appears to duplicate the environment at different
angles matching the rover rotation.

## Launch statement

```
ros2 launch cslam_experiments rover_swarm.launch.py max_nb_robots:=1 config_file:=rover_lidar_kf02.yaml
```

No launch file was modified. Only the `config_file` argument changes, using two
new config files:

- `rover_lidar_kf01.yaml`: `keyframe_generation_ratio_distance` 0.5 -> 0.1
- `rover_lidar_kf02.yaml`: `registration_min_inliers` 60 -> 15, `similarity_threshold` 0.8 -> 0.6

Both live in
`/home/ac/ros2_ws/install/cslam_experiments/share/cslam_experiments/config/`.

## Root cause of the rotation map bug

The C-SLAM pipeline is rotation correct. Verified live: keyframe map poses
(position and orientation) exactly match the rover odometry at capture.

Example from the drive-through-turn test:

- straight leg: keyframes 0-6 at yaw 0.0
- after a +60 degree turn: keyframes 7-12 at yaw 58.3
- later legs: keyframes 13-23 at yaw -81.2

The pose graph preserves orientation end to end: odom -> Pose3 -> GTSAM ->
`PoseGraphValue.pose` -> keyframe TF `robot0_map -> robot0_keyframe{i}`.

The duplicated/rotating map the seen comes from the RViz LiDAR PointCloud
display, not from C-SLAM. Confirmed by test:

- RViz display with `Use Fixed Frame: false` and Decay Time 20
- after a 30 degree rover rotation, cloud pixels grew from 16,588 to 50,264
  (3x, the environment smeared across the rotation arc)
- correct display (`Use Fixed Frame: true`, Decay 0): 173 pixels before and
  after rotation, unchanged

The generated RViz config uses `Use Fixed Frame: true` and Decay 0, which is
correct. If a display is set to `Use Fixed Frame: false` (or Fixed Frame is a
robot frame) with a decay, the accumulated cloud rotates with the rover.

## Loop closure verification

ScanContext detected candidates but the default verification threshold
(`registration_min_inliers` 60) rejected them (17 inliers on a planar wall
scene). Lowered to 15 in `rover_lidar_kf02.yaml`.

With a longer out-and-back drive so revisit keyframes were at least 20 indices
apart (`intra_loop_min_inbetween_keyframes`):

- 27 intra-robot loop closures added to the pose graph, for example
  `New intra-robot loop closure (20, 0)` and `(46, 11)`
- final graph: 49 keyframes, 75 edges (48 odometry + 27 loop-closure edges)
- keyframe poses stay rotation correct

Full detection chain works: ScanContext descriptor -> candidate ->
TEASER++ geometric verification -> backend pose graph update.

## Other verified items

- Odom message pose and yaw track the rover correctly.
- TF chain correct: `world -> rover_0/odom` static at spawn, `rover_0/odom ->
  rover_0/base_link` tracks odom yaw.
- No odom position drift during in-place rotation.
- Cloud frame is `rover_0/base_link/lidar`.
- cslam_visualization publishes one marker per keyframe and broadcasts
  `robot0_map -> robot0_keyframe{i}` TFs. "Publishing 0 pointclouds" logs are
  idle gaps, not a fault.
- Loop closure detection output verified in run7.

## Known issues found

- `pointcloud_visualizer.py` never clears `tfs_to_publish`, only
  `markers_to_publish`. Unbounded growth of broadcast transforms.
- The visualization broadcasts each keyframe TF once with volatile QoS. An RViz
  started after the keyframes were created sees nothing until new keyframes
  arrive.
- The rover moves at about 0.01 m/s in the sim regardless of the command
  magnitude. Wheels spin at about 0.194 rad/s while commanded near 14.8 rad/s.
  Not teleop interference, not a collision, not e-stop. Unexplained sim physics
  anomaly, pre-existing.

## Logs and artifacts

- `/tmp/cslam_test/run7_swarm.log` (live run, 27 loop closures)
- `/tmp/cslam_test/run6_swarm.log` (previous run)
- `/tmp/cslam_test/x_base.png`, `/tmp/cslam_test/x_rot30.png` (RViz display test)
- `/tmp/cslam_test/drive_turn.log`, `/tmp/cslam_test/loop2.log` (drive evidence)
- `/home/ac/ros2_ws/install/cslam_experiments/share/cslam_experiments/config/rover_lidar_kf01.yaml`
- `/home/ac/ros2_ws/install/cslam_experiments/share/cslam_experiments/config/rover_lidar_kf02.yaml`
