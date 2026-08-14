#!/usr/bin/env bash
# Session cleanup helper for rover_description simulation.launch.py.
# When any of the three terminals (main launch, Fleet Teleop, E-Stop) is
# killed, launch fires an OnShutdown event that runs this script. It kills
# Gazebo, RViz2, and every rover node so a single Ctrl+C always tears down
# the whole session.

pkill -f "rover_teleop\.py" || true
pkill -f "estop_manager\.py" || true
pkill -f "dock_monitor\.py" || true
pkill -f "diagnostics_aggregator\.py" || true
pkill -f "livox_publisher\.py" || true
pkill -f "sonar_to_range\.py" || true
pkill -f "sensor_frame_aliases\.py" || true
pkill -f "battery_publisher\.py" || true
pkill -f "robot_state_publisher" || true
pkill -f "parameter_bridge" || true
pkill -f "static_transform_publisher" || true
pkill -f "rviz2" || true
pkill -f "ign gazebo" || true
pkill -f "gz sim" || true
pkill -f "gzserver" || true
pkill -f "gzclient" || true
