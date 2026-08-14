# Rover platform — parameter configuration guide
**Dev 1 (Platform/Hardware + Simulation) — mesh rover swarm project**

This is the reference for every tunable value that defines how the rover platform is built, moves, senses, and talks to the fleet. It's meant to be read alongside `TOPIC_PARITY.md`, which is the sim-to-real topic contract.

---

## 1. Physical geometry (from `urdf/rover.urdf`)

| Parameter | Value | Notes |
|---|---|---|
| Wheel radius | 0.04045 m | Matches collision cylinder and DiffDrive plugin |
| Wheel separation (track width) | 0.13 m | Left pair vs right pair |
| Wheelbase (front–back spacing) | ~0.097 m | From `wheel_fr` x=0.065092 to `wheel_br` x=-0.031906 |
| Wheel naming | `fr / fl / br / bl` | Continuous joints, axis `0 1 0` |
| Base link mass | 5.0 kg | Raised from 1.2 kg to lower CoG and stop pitch on accel/brake |
| Base link inertial origin | z = 0.05 m | Lowered from 0.08 m for the same reason |
| Wheel mass | 0.1 kg each | |

### Sensor mount offsets (relative to `base_link`)

| Sensor | xyz (m) | rpy (rad) |
|---|---|---|
| LiDAR (Livox Mid-360) | -0.0298, -0.008, 0.1131 | 0, 0, 0 |
| Camera (LeTMC-520) | 0.1135, -0.0022, 0.0597 | 0, 0, 0 |
| Camera optical frame | 0, 0, 0 (child of camera_link) | -1.5708, 0, -1.5708 |
| Sonar front | 0.125, 0.0022, 0.0344 | yaw 0 |
| Sonar front-right | 0.11, -0.084, 0.0344 | yaw -0.7854 (-45°) |
| Sonar front-left | 0.111, 0.086, 0.0344 | yaw +0.7854 (+45°) |
| Sonar rear | -0.119, 0.004, 0.0344 | yaw 3.1416 (180°) |
| Sonar rear-right | -0.095, -0.085, 0.0344 | yaw -2.3562 (-135°) |
| Sonar rear-left | -0.095, 0.092, 0.0344 | yaw +2.3562 (+135°) |

---

## 2. Drivetrain / motion limits (Gazebo `DiffDrive` plugin)

| Parameter | Value |
|---|---|
| max_linear_velocity | 1.0 m/s |
| max_linear_acceleration | 1.0 m/s² |
| max_angular_velocity | 1.0 rad/s |
| max_angular_acceleration | 1.0 rad/s² |
| odom_publish_frequency | 30 Hz |
| cmd_vel real-world rate | 20 Hz (per TOPIC_PARITY.md) |

**Hard rule:** these four motion limits must exactly match `linear_accel` / `angular_accel` in `config/swarm.yaml` (`fleet_teleop` block), which now override the old hard-coded constants in `rover_teleop.py`. A mismatch causes double-filtering — visible as constant jerk during accel/decel.

---

## 3. Sensor rates and specs (actual URDF/Gazebo values — source of truth)

| Sensor | Rate | Spec | Real driver |
|---|---|---|---|
| LiDAR (Livox Mid-360, `gpu_lidar`) | 5 Hz | 360 horizontal samples, 16 vertical, range 0.1–40 m | livox_ros_driver2 |
| Camera (LeTMC-520) | 10 Hz | 320×240, R8G8B8, FOV 1.396 rad | LeTMC-520 |
| IMU | 100 Hz | On base_link, no noise modeled yet | MPU-9250 / BMI088 class |
| Sonars (×6) | 5 Hz | 25° cone (5 samples × 5°), range 0.02–2.0 m | HC-SR04 |

> Note: camera was downgraded from 640×480 @ 15 Hz to keep real-time factor above 1.0 with 3 rovers running. Bump back up per-project on a beefier machine.

### Reference noise values (not yet applied — for later SLAM tuning)

| Sensor | Noise model | Value |
|---|---|---|
| LiDAR range | Gaussian | 0.02 m stddev |
| RGB camera | Gaussian | 0.007 normalized |
| IMU gyro | Gaussian + bias walk | 0.001 rad/s, bias 0.0001 rad/s² |
| IMU accel | Gaussian + bias walk | 0.01 m/s², bias 0.001 m/s³ |
| Sonar range | Gaussian | 0.003 m stddev |

---

## 4. Fleet configuration (`config/swarm.yaml`)

| Parameter | Value | Purpose |
|---|---|---|
| num_rovers | 3 (1–10 supported) | Fleet size |
| namespace_prefix | `rover_` | Topic/TF prefix |
| first_index | 0 | Numbering start |
| full_sensors_all | true | false = only rover_0 is full "scout"; rest run lite (IMU + camera + front sonar) |
| spawn_y_spacing | 0.8 m | Line-layout fallback spacing |
| spawn_z | 0.061 m | Tuned so wheels just touch the ground plane |

### Docking station (`dock_monitor_0`)

| Parameter | Value |
|---|---|
| dock_x, dock_y | -2.0, 0.0 |
| contact_offset | 0.15 m |
| charging_radius | 0.10 m |
| charging_heading_tol | 0.35 rad (~20°) |
| docking_radius | 0.30 m |
| docking_heading_tol | 0.60 rad |
| approach_radius | 1.0 m |

### Battery model (`battery_publisher`)

| Parameter | Value |
|---|---|
| capacity_wh | 100.0 |
| nominal_voltage | 24.0 V |
| drive_current | 3.0 A |
| idle_current | 0.5 A |
| charge_current | 10.0 A |
| publish_rate | 1.0 Hz |
| initial_percentage | 0.85 |

### E-stop and teleop

| Parameter | Value | Notes |
|---|---|---|
| estop_manager.stop_topic | /emergency_stop | |
| fleet_teleop.speed_init | 0.8 m/s | initial linear set-point |
| fleet_teleop.turn_init | 0.8 rad/s | initial angular set-point |
| fleet_teleop.speed_min | 0.0 m/s | removes the old 0.56 floor |
| fleet_teleop.speed_max | 1.0 m/s | matches URDF max_linear_velocity |
| fleet_teleop.turn_min | 0.0 rad/s | |
| fleet_teleop.turn_max | 1.0 rad/s | matches URDF max_angular_velocity |
| fleet_teleop.linear_accel | 1.0 m/s² | matches URDF max_linear_acceleration |
| fleet_teleop.angular_accel | 1.0 rad/s² | matches URDF max_angular_acceleration |
| fleet_teleop.publish_period | 0.05 s | 20 Hz cmd_vel loop |
| fleet_teleop.release_window | 0.12 s | coast-down delay after arrow key release |

---

## 5. Topic and namespace scheme

All per-rover topics live under `/rover_i/...`. Full contract in `TOPIC_PARITY.md` — key ones:

| Topic | Type | Rate |
|---|---|---|
| /rover_i/cmd_vel | geometry_msgs/Twist | 20 Hz |
| /rover_i/odom | nav_msgs/Odometry | ~50 Hz |
| /rover_i/lidar/points | sensor_msgs/PointCloud2 | 5 Hz |
| /rover_i/livox/lidar | livox_msgs/CustomMsg | 5 Hz |
| /rover_i/camera/image | sensor_msgs/Image | 10 Hz |
| /rover_i/imu | sensor_msgs/Imu | 100 Hz |
| /rover_i/sonar/DIR/range | sensor_msgs/Range | 5 Hz |
| /rover_i/battery_state | sensor_msgs/BatteryState | 1 Hz |

Fleet-wide (no namespace): `/clock`, `/tf`, `/tf_static`, `/emergency_stop`, `/dock_0/status`, `/diagnostics`.

### TF frame layout

```
world
└ rover_i/odom (static, set at spawn)
  └ rover_i/base_link (from DiffDrive)
    ├ rover_i/lidar_link
    ├ rover_i/camera_link
    ├ rover_i/imu_link
    ├ rover_i/sonar_DIR_link (six of these)
    └ rover_i/wheel_XX_link (four of these)
```

---

## 6. QoS overrides (`config/sensor_qos.yaml`)

All sensor topics (lidar, camera, IMU, sonar range) use `BEST_EFFORT` + `KEEP_LAST(5)`. This matches what Nav2, SLAM Toolbox, Cartographer, and FAST-LIO2 expect on the consumer side.

---

## 7. Network / environment

| Variable | Value |
|---|---|
| ROS_DOMAIN_ID | 42 (isolates this workspace from LAN / other tasks) |
| ROS_LOCALHOST_ONLY | 1 |

---

## 8. Known sim-to-real gaps (from TOPIC_PARITY.md)

1. Livox sim emits a uniform scan grid; real Mid-360 sweeps a non-repeating rosette — same message format, different point density.
2. Bridge QoS defaults to RELIABLE underneath even though converter nodes publish BEST_EFFORT — subscribing with SensorDataQoS handles both.
3. Perfect odometry in sim (no drift/slip) vs real wheel odometry, which drifts significantly — this is the core Week 6 sim-to-reality gap to address.
