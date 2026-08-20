# Ground-truth visualization fix (RViz ↔ Gazebo)

**Date:** 2026-08-19
**Scope:** `dev1_platform/rover_description` — propagates automatically to every
session type (`simulation.launch.py`, `nav2_rover.launch.py`, and Swarm-SLAM's
`cslam_experiments/rover_swarm.launch.py`, which all include the same base launch).

## The problem

When teleop-driving a rover into a wall in Gazebo, the simulated robot stops
(or flips), but the robot in RViz2 keeps driving — straight through walls.
Flips never appear in RViz at all.

## Root cause

RViz never received the robot's true pose. The TF it renders,

```
rover_i/map → rover_i/odom → rover_i/base_link
```

gets its second link from Gazebo's **DiffDrive** plugin
(`urdf/rover.urdf`), which has no `<odom_source>` element and therefore uses
the default **ENCODER** odometry: dead reckoning integrated from wheel
velocities, planar only (x, y, yaw).

- Robot blocked by a wall → wheels keep spinning → odometry keeps integrating
  → RViz robot walks through the wall.
- Robot flips → roll/pitch/z are never published by DiffDrive odometry →
  invisible in RViz by construction.

Gazebo knew the true pose all along; nothing bridged it to ROS. The only
correction in the stack was slam_toolbox's scan matching (`map → odom`), which
lags or is absent (bare sim sessions), so the dead reckoning was displayed
uncorrected.

## The fix (proper, not the quick one)

The quick fix (`<odom_source>world</odom_source>` on DiffDrive) was rejected:
it would make the odometry "too perfect" and destroy realistic wheel-slip
behavior that SLAM/Nav2/cslam development depends on.

Instead, ground truth is exported as a **separate, parallel data path** that
nothing in the existing stack consumes:

```
Gazebo physics
  └─ PosePublisher plugin (per rover, in URDF)
       gz: /model/<rover_i>/pose  (ignition.msgs.Pose, world frame, 30 Hz)
  └─ per-rover ros_gz_bridge
       ROS: /model/<rover_i>/pose (geometry_msgs/PoseStamped)
  └─ ground_truth_tf.py (ONE node for the whole fleet)
       TF: map → rover_i/gt_base_link   (on global /tf)
  └─ 2nd robot_state_publisher per rover (frame_prefix=<ns>/gt_)
       TF: rover_i/gt_base_link → rover_i/gt_<every URDF link>
           (driven by the same /rover_i/joint_states = real physics angles)
  └─ RViz: "RobotModel (Gazebo truth)" display per rover group = the primary,
     fully-articulated robot rendering; the belief-chain "RobotModel (odom
     belief)" is disabled by default (toggleable ghost)
```

Key design points:

- **gz world frame == ROS `map` frame.** Each rover's `rover_i/map` is anchored
  to `map` by a static TF at its gz spawn pose, and the gz world spawn
  coordinates equal that pose — so the PosePublisher's world pose is directly a
  pose in `map`. (`ground_truth_tf.py` deliberately replaces the incoming
  `frame_id`, which carries the gz world name, e.g. `test_station`.)
- **Frame named `rover_i/gt_base_link`** (prefix style, not `base_link_gt`) so
  the mirrored link tree hangs under one consistent `rover_i/gt_` prefix that
  the RViz RobotModel display can consume via its "TF Prefix" property.
- **The belief chain is untouched.** `rover_i/odom → rover_i/base_link` and
  everything below it still carry encoder odometry; slam_toolbox, Nav2, cslam,
  the cmd_vel arbiter and tf_relay all see exactly the same data as before.
  The GT path is additive — nothing existing consumes the `gt_` frames.
- **30 Hz**, matching the DiffDrive odom rate.

## Files changed

| File | Change |
|---|---|
| `urdf/rover.urdf:590-608` | Added model-level `ignition::gazebo::systems::PosePublisher` plugin (outside the `__LITE_STRIP__` markers, so lite rovers keep it too) |
| `urdf/rover.urdf` (DiffDrive block) | `<tf_topic>` is now the launch-substituted `__DIFF_TF_TOPIC__` token (v4 GT-diversion) |
| `launch/simulation.launch.py` | Per-rover bridge arg `/model/<ns>/pose@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose`; `/<ns>/odom` bridge only in encoder mode; second `robot_state_publisher` (`gt_state_publisher`, `frame_prefix=<ns>/gt_`); fleet-wide `ground_truth_tf.py` (`--publish-odom` in GT mode) and `gt_sensor_relay.py` nodes; `ground_truth` launch arg (default `true`) |
| `launch/nav2_rover.launch.py` | `ground_truth` arg + passthrough (v4) |
| `Swarm-SLAM/.../rover_swarm.launch.py` | `ground_truth` arg + passthrough (v4) |
| `scripts/ground_truth_tf.py` | **New.** PoseStamped → TF `map → <ns>/gt_base_link`; with `--publish-odom` also `<ns>/odom → <ns>/base_link` TF + `/<ns>/odom` topic (planar, spawn-relative, finite-diff twist); subscribes BEST_EFFORT so any bridge QoS works |
| `scripts/gt_sensor_relay.py` | **New.** Republishes lidar points + sonar ranges on `/<ns>/gt/...` with frames re-anchored to the gt_ links (RViz-only); RELIABLE out |
| `rviz/rover.rviz` | Template group: **"RobotModel (Gazebo truth)"** (TF Prefix `rover_0/gt_`, enabled = primary view); belief model renamed **"RobotModel (odom belief)"**, disabled by default; **"Ground Truth (Gazebo)"** Axes display at `rover_0/gt_base_link`; PointCloud2/Range displays re-pointed to the `/gt/` topics. `generate_rviz_config()` clones all per rover |
| `CMakeLists.txt` | Install `scripts/ground_truth_tf.py` + `scripts/gt_sensor_relay.py` |

### Gazebo Fortress (ign-gazebo6) quirks discovered

Verified against `PosePublisher.cc` on the `ign-gazebo6` branch (this machine
runs Gazebo Sim 6.18.0; note `/usr/bin/gz` here is Gazebo **Classic** 11 — the
sim actually runs via `ign gazebo`):

- The pose topic is **hardcoded** to `/model/<model_name>/pose` — there is no
  `<topic>` parameter (it is silently ignored).
- The top-level model pose is published only when **both**
  `publish_nested_model_pose` **and** `publish_model_pose` are true
  (backward-compatibility quirk in the code).
- `publish_link_pose` defaults to `true` — must be disabled or every link
  spams the same topic.
- Frame names come from entity names, not from `frame_id`/`child_frame_id`
  params (those don't exist in Fortress).

## Test-run evidence (2026-08-19, `simulation.launch.py teleop:=false`, 1 rover)

1. **GT topic flows:** `/model/rover_0/pose` → position (3.300, 0.400, 0.061),
   yaw π — exact match to the swarm.yaml spawn pose.
2. **TF live:** `tf2_echo map rover_0/gt_base_link` → same pose, updating at 30 Hz.
3. **Wall test (live reproduction of the original bug):** drove forward at
   0.4 m/s; the rover physically stopped against the charging station after
   **0.77 m** (GT: x 3.30 → 2.53, frozen) while the encoder odometry walked on
   to **1.41 m**. The discrepancy the user reported is now *visible* in RViz
   instead of hidden.
4. **Flip test:** `set_pose` with 90° roll → GT TF reports RPY (90°, 0, 0) and
   z settling as the body falls on its side; `/rover_0/odom` stays planar
   (z = 0, no roll) — old behavior preserved for SLAM realism, truth now shown
   alongside.
5. **RViz:** config generator verified offline for scout + lite rovers; rviz2
   ran healthy during the live session.
6. **No conflicts:** all pre-existing nodes (arbiter, battery, dock monitor,
   diagnostics, bridges) ran unchanged; cslam's `/rover_i/odom` +
   `/rover_i/lidar/points` inputs are untouched, so `rover_swarm.launch.py`
   sessions inherit the fix with zero changes.

## v2 (same day): ground truth DOMINATES the RViz rendering

First field test with Nav2 waypoint following showed the v1 layout was still
backwards for the operator: the big robot model followed the *belief* chain
(odometry + SLAM) while the actual Gazebo pose was only a small axes triad.
When the belief strayed, RViz "kept going the wrong way" visually.

v2 flips the dominance, RViz-side only:

- A second `robot_state_publisher` per rover republishes the full URDF link
  tree under the `rover_i/gt_` prefix, fed by the same `/rover_i/joint_states`
  (real physics wheel angles), and `ground_truth_tf.py` anchors it with
  `map → rover_i/gt_base_link` (frame renamed from `base_link_gt` to fit the
  RobotModel "TF Prefix" scheme).
- The rover group in RViz now has **"RobotModel (Gazebo truth)"** enabled as
  the primary rendering — the robot you see is where Gazebo actually has it,
  with wheels/sensors articulated from physics.
- **"RobotModel (odom belief)"** (the old model) is still in the group,
  disabled by default — enable it to see the belief ghost. The colored
  Odometry arrow trail, SLAM map, lidar points and sonar ranges remain on the
  belief chain, so "what the robot thinks" is still fully visible and
  self-consistent (points stay aligned with the SLAM map they built).
- Nav2/SLAM/cslam inputs are unchanged — this is purely how RViz draws.

v2 test-run evidence (2026-08-19, same session type):

1. `map → rover_0/gt_base_link` at exact spawn pose; `gt_base_link →
   gt_wheel_fl_link` offset (0.065, 0.065, −0.020) live from the 2nd RSP;
   `map → rover_0/gt_lidar_link` resolves through the GT chain.
2. Flip test: 90° roll `set_pose` → whole GT tree rotates (gt_base_link and
   gt_lidar_link both report RPY (90°, 0, 0); lidar link z drops 0.174 →
   0.095 as the body settles on its side).
3. Generated config verified offline for scout + lite: each rover group has
   GT model enabled with prefix `rover_i/gt_`, belief model disabled, axes at
   `rover_i/gt_base_link`.
4. rviz2 alive with zero `[rviz2]` error lines in the launch log; session
   teardown clean.

## v3 (same day): sensor overlays moved onto the GT robot

Field test of v2 confirmed the GT model tracks Gazebo exactly — but the lidar
points and sonar cones still rendered through the belief chain, so when the
belief strayed (worst during skid-steer rotation) the sensor visuals went with
the wrong robot.

v3 adds `scripts/gt_sensor_relay.py` (one fleet-wide node): it republishes
`/<ns>/lidar/points` and the six `/<ns>/sonar/*/range` streams on
`/<ns>/gt/...` topics with `header.frame_id` rewritten to the GT mirror links
(`<ns>/gt_lidar_link`, `<ns>/gt_sonar_front_link`, … — identity-equivalent to
the original frames, same equivalence table as `sensor_frame_aliases.py`).
The RViz template's PointCloud2 display (renamed "LiDAR PointCloud (GT)") and
all six Range displays now subscribe to the `/gt/` topics.
`generate_rviz_config()` re-namespaces them per rover as usual; the lite-rover
filter (front sonar only) is untouched.

**The originals are deliberately NOT moved.** `/rover_i/lidar/points` and the
raw sonar scans keep their belief-chain frames, and slam_toolbox / Nav2 /
cslam keep consuming exactly those. Only RViz's view moved. See "Reality
check" below for why.

Files (v3): `scripts/gt_sensor_relay.py` (new), one node block in
`launch/simulation.launch.py`, install entry in `CMakeLists.txt`, six Range
topics + one PointCloud2 topic in `rviz/rover.rviz`.

v3 test-run evidence (2026-08-19, same session type, rotation-first motion as
requested):

1. **Frames:** `/rover_0/gt/lidar/points` carries `rover_0/gt_lidar_link`;
   `/rover_0/gt/sonar/*/range` carry `rover_0/gt_sonar_*_link`; originals
   verified unchanged (`rover_0/base_link/lidar`, `rover_0/sonar_front_link`).
2. **Arc phase** (12 s, v=0.25 + ω=0.7) then straight (3 s, v=0.4): truth
   ended at (2.78, −0.65) having travelled ~1.2 m; belief thought (−0.30,
   0.38) — wrong distance AND wrong direction.
3. **Rotation-in-place** (10 s, ω=1.0 — worst-case skid): true yaw swept
   **199.5°** while the encoders registered only **46.5°**, and the chassis
   scrubbed ~0.66 m sideways that the odometry attributed differently
   (0.36 m). This is exactly the rotation-slip divergence the user reported —
   now visible: GT model + GT sensor overlays stay on the true robot.
4. **Health:** GT points flowing (~2 Hz effective under full sim load — the
   relay republishes 1:1 whatever the gpu_lidar produces, no throttle),
   rviz2 alive with zero error lines, clean teardown.

## Reality check: what v1–v3 fix and what they cannot

The map warping and Nav2 waypoint drift observed in the field were **not**
caused by RViz wiring — they came from wheel slip corrupting the encoder
odometry that slam_toolbox and Nav2 legitimately consume. v1–v3 made that
divergence *visible*; v4 (below) makes the autonomy itself run on ground
truth by default.

## v4 (2026-08-20): autonomy runs on ground truth BY DEFAULT

New launch arg `ground_truth` on `simulation.launch.py` (default `true`),
passed through by `nav2_rover.launch.py` and cslam's `rover_swarm.launch.py`:

    ros2 launch rover_description simulation.launch.py                 # GT (default)
    ros2 launch rover_description simulation.launch.py ground_truth:=false   # realistic slip
    ros2 launch cslam_experiments rover_swarm.launch.py ground_truth:=false  # same for cslam

How it works (Gazebo **Fortress**, ign-gazebo6, has **no** `<odom_source>`
param — verified against `DiffDrive.cc` on the ign-gazebo6 branch; that
feature, like TurtleBot3-classic's `<odometry_source>world</...>`, only
arrived in gz-sim7/Garden — and its DiffDrive **always** publishes TF, so it
can't simply be switched off):

- The URDF's DiffDrive `<tf_topic>` is a launch-substituted token
  (`__DIFF_TF_TOPIC__`): in GT mode it becomes `__NS__/encoder_tf`, diverting
  the encoder TF to a dead-end gz topic that is never bridged to ROS; in
  encoder mode it stays `/tf` (original behavior).
- In GT mode the `/<ns>/odom` bridge arg is dropped (otherwise the bridge's
  encoder odometry and the GT odometry would interleave on one topic).
- `ground_truth_tf.py --publish-odom` (started in GT mode) then publishes
  both `<ns>/odom → <ns>/base_link` TF **and** the `/<ns>/odom`
  nav_msgs/Odometry topic from the true pose — planar and spawn-relative
  (identity at spawn), i.e. the exact interface encoder odometry had, so
  slam_toolbox / Nav2 / cslam / dock_monitor need zero changes. The twist is
  finite-differenced at the 30 Hz pose rate and expressed in the child frame,
  matching the DiffDrive convention.
- The full 6-DOF truth (`map → <ns>/gt_base_link` + the gt_ link tree + GT
  sensor overlays) stays available for RViz in BOTH modes; in encoder mode
  it remains the "where the rover actually is" reference.

v4 test-run evidence (2026-08-20, `teleop:=false`, 1 rover):

- **GT mode, at rest:** `/rover_0/odom` ≈ identity, publisher count 1 (the
  GT node; no interleaving); gz topic `/rover_0/encoder_tf` carries the
  diverted encoder TF; gz `/tf` carries nothing.
- **GT mode, rotation-in-place** (10 s, ω=1.0): GT yaw +88.5°, odom yaw
  +89.8° (≤1.4° sampling lag) — the same maneuver under encoder mode
  previously registered 46.5°–287° (run- and friction-dependent) against
  89°–199.5° of truth. Simultaneous snapshot: 0.1 cm / 0.9° error.
- **GT mode, wall test** (0.4 m/s into the charging station): truth stopped
  at 0.94 m from spawn and the odom stopped with it (0.90 m) — encoder mode
  previously walked on to 1.41 m.
- **Encoder mode** (`ground_truth:=false`): `/rover_0/odom` publisher = the
  bridge (count 1), identity at rest, rotation-in-place over-integrated to
  287° vs 89° true — realistic skid-slip preserved; GT truth tree still
  available for RViz.
- **Both modes:** rviz2 healthy, zero QoS warnings (the GT pointcloud relay
  now publishes RELIABLE after RViz was observed requesting RELIABLE despite
  the saved Best Effort policy), zero rviz errors, clean teardown.

One XML gotcha found while editing: Gazebo's URDF parser rejects `--` inside
XML comments — don't write flag names like `--publish-odom` inside URDF
comments.

## What you see in RViz now

With the default `ground_truth:=true`, belief == truth: the robot model, the
odometry trail, the SLAM map and the sensor overlays all agree, because the
autonomy itself is fed ground truth (v4). With `ground_truth:=false` the
layout below becomes a diagnostic view — divergence between the GT
robot/sensor overlays and the trail/map IS the localization error:

- **Robot model ("RobotModel (Gazebo truth)")** — where the rover *actually*
  is in Gazebo, full 6-DOF: blocked-at-wall stops it, flips show it.
- **LiDAR PointCloud (GT) + sonar cones** — the sensor readings, drawn at the
  true sensor poses (via the `/<ns>/gt/...` relayed topics).
- **RGB axes triad ("Ground Truth (Gazebo)")** — same truth frame, handy when
  the model is hidden.
- **Colored odometry arrow trail + SLAM map** — where the rover *thinks* it
  is (encoder odometry ± SLAM correction). Divergence between the GT
  robot/sensor overlays and the trail/map = localization error, visible at a
  glance.
- **"RobotModel (odom belief)"** — disabled ghost of the belief-chain model;
  toggle it on to compare.
- The TF display discovers all `rover_i/gt_*` frames live (frame list is
  stripped at config generation).

## Known limitations (pre-existing, unchanged)

- RViz runs namespaced under the first rover and reads `/<first_rover>/tf`.
  The GT TF is published on **global** `/tf` (same as the DiffDrive TF), so in
  a bare `simulation.launch.py` session (no `tf_relay.py`) RViz's TF feed has
  the same gap it already had for odom TF. Sessions via `nav2_rover.launch.py`
  or `rover_swarm.launch.py` (which run `tf_relay.py`) see everything.
- `tf_relay.py` mirrors cross-rover frames into other namespaces with double
  prefixes (e.g. `rover_1/rover_0/gt_base_link`) — pre-existing inert behavior
  for all global frames, not introduced by this change. With a single-rover
  fleet (the swarm.yaml default) this never triggers.
- A stale `ground_truth_tf.py` symlink existed in `install/` from an earlier
  abandoned attempt (source file was missing); this change finally provides
  the source file, so the symlink is valid again.

## Housekeeping note found during testing

When `simulation.launch.py` dies from a launch-time exception (e.g. a missing
executable), already-started children (gz, bridges, rviz, arbiter) can leak —
the session watchdog does not always reap them. If a session fails at startup,
check `pgrep -af 'ign gazebo|parameter_bridge|cmd_vel_arbiter'` before
relaunching. (Observed once during this fix's test-run; cleaned up manually.)
