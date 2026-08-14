# Rover platform — system architecture

End-to-end data flow for the mesh rover swarm sim, numbered tier 1 → 6, from the physics
and sensor sim down to the fleet nodes and the Week 4+ SLAM handoff. Tiers 1–3 (Gazebo,
bridge, per-rover nodes) are spawned N times, one namespace `/rover_i` per rover, off a
single `config/swarm.yaml`. Read alongside `PARAMETER_CONFIG_GUIDE.md` (the values) and
`TOPIC_PARITY.md` (the sim-to-real topic contract).

```mermaid
%%{init: {'theme':'base','themeVariables':{'fontFamily':'Segoe UI, Helvetica, Arial, sans-serif','fontSize':'14px','lineColor':'#5b6472','clusterBkg':'#f8fafc','clusterBorder':'#cbd5e1'},'flowchart':{'curve':'basis','nodeSpacing':30,'rankSpacing':52}}}%%
flowchart TB
  cfg[/"config/swarm.yaml<br/>num_rovers · spawn poses · namespaces"/]:::cfg
  cfg -->|read at launch| launch["simulation.launch.py"]:::cfg

  subgraph SIM["1 · Gazebo — Ignition Fortress · world: crop_field.sdf · ×N rovers"]
    direction LR
    lx["Livox Mid-360<br/>5 Hz"]:::sim
    cam["Camera LeTMC-520<br/>320×240 @ 10 Hz"]:::sim
    imu["IMU · 100 Hz"]:::sim
    son["6× Sonar · 5 Hz"]:::sim
    dd["DiffDrive plugin"]:::sim
    js["JointState plugin"]:::sim
    sp["World Sensors plugin"]:::sim
  end

  subgraph BRIDGE["2 · ros_gz parameter_bridge"]
    direction LR
    gb["Global bridge<br/>/clock · /tf"]:::bridge
    pb["Per-rover bridge ×N<br/>/rover_i/*"]:::bridge
  end

  subgraph NODES["3 · Per-rover ROS 2 nodes · ×N"]
    direction LR
    lp["livox_publisher"]:::ros
    sr["sonar_to_range"]:::ros
    bp["battery_publisher"]:::ros
    rsp["robot_state_publisher"]:::ros
    fa["sensor_frame_aliases"]:::ros
    stf["static_tf"]:::ros
  end

  subgraph TOPICS["4 · Namespaced topics · /rover_i/*"]
    direction LR
    tctl["cmd_vel · odom"]:::topic
    tlid["lidar/points · livox/lidar"]:::topic
    tcam["camera/image · imu"]:::topic
    tson["sonar/DIR/range · battery_state"]:::topic
    ttf["/tf · /tf_static"]:::topic
  end

  subgraph FLEET["5 · Fleet nodes · single instance"]
    direction LR
    tel["fleet_teleop"]:::fleet
    dock["dock_monitor"]:::fleet
    est["estop_manager"]:::fleet
    diag["diagnostics_aggregator"]:::fleet
    rviz["RViz"]:::fleet
  end

  subgraph DOWN["6 · Downstream · Week 4+ handoff"]
    direction LR
    slam["SLAM — Dev 2<br/>FAST-LIO2 / slam_toolbox"]:::down
    nav["Nav2 (planned)"]:::down
  end

  launch -. spawns ×N .-> SIM
  SIM ==>|Ignition msgs · /odom| BRIDGE
  BRIDGE ==>|raw ROS 2 topics| NODES
  NODES ==>|converted msgs · TF| TOPICS
  TOPICS ==>|subscribed by| FLEET
  TOPICS ==>|lidar · camera · tf| DOWN
  FLEET -.->|cmd_vel · e-stop control| SIM
  slam --> nav
  lx ~~~ cam ~~~ imu ~~~ son ~~~ dd ~~~ js ~~~ sp
  gb ~~~ pb
  lp ~~~ sr ~~~ bp ~~~ rsp ~~~ fa ~~~ stf
  tctl ~~~ tlid ~~~ tcam ~~~ tson ~~~ ttf
  tel ~~~ dock ~~~ est ~~~ diag ~~~ rviz

  classDef cfg fill:#fff2cc,stroke:#d6b656,color:#3a3210;
  classDef sim fill:#dae8fc,stroke:#6c8ebf,color:#1a2b3c;
  classDef bridge fill:#ffe6cc,stroke:#d79b00,color:#3a2a10;
  classDef ros fill:#d5e8d4,stroke:#82b366,color:#1f3320;
  classDef topic fill:#fff2cc,stroke:#d6b656,color:#332a0a;
  classDef fleet fill:#e1d5e7,stroke:#9673a6,color:#2e2138;
  classDef down fill:#f8cecc,stroke:#b85450,color:#3a1512;
```

## How to read it

- **Follow the numbers, 1 → 6, and the labels on each arrow.** The thick arrows are the
  spine of the data flow: sim → bridge → per-rover nodes → topics → downstream. The label
  on each says what crosses it (`Ignition msgs · /odom`, `raw ROS 2 topics`,
  `converted msgs · TF`, `lidar · camera · tf`).
- **Colors reinforce the tiers.** Blue = Gazebo sim, orange = ros_gz bridge, green =
  per-rover nodes, yellow = namespaced topics, purple = single fleet nodes, red =
  downstream consumers.
- **The control loop.** Fleet teleop and the e-stop feed back into the sim
  (`cmd_vel · e-stop control`, the dashed arrow), so `/rover_i/cmd_vel` reaches the
  DiffDrive plugin. Everything else flows one way, down the spine.
- **× N means replicated per rover.** Tiers 1–3 and the per-rover bridge are spawned once
  per rover at namespace `/rover_i`, set by `num_rovers` in `swarm.yaml` (the dashed
  `spawns × N` arrow from the launch file). The fleet nodes and the global bridge run once
  for the whole swarm.
- **`/tf` and `/clock` are deliberately shared, not namespaced:** sim time is one global
  clock and TF is one tree with per-rover prefixed frames (`rover_i/…`).
