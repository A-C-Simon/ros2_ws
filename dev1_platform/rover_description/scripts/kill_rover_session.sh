#!/usr/bin/env bash
# Session cleanup helper for rover_description simulation.launch.py.
# When any of the three terminals (main launch, Fleet Teleop, E-Stop) is
# killed, launch fires an OnShutdown event that runs this script. It kills
# Gazebo, RViz2, the Nav2 stack, cslam, and every rover node so a single
# Ctrl+C always tears down the whole session (no stale ROS processes left
# crowding the DDS domain).

pkill -f "rover_teleop\.py" || true
pkill -f "estop_manager\.py" || true
pkill -f "dock_monitor\.py" || true
pkill -f "diagnostics_aggregator\.py" || true
pkill -f "livox_publisher\.py" || true
pkill -f "sonar_to_range\.py" || true
pkill -f "sensor_frame_aliases\.py" || true
pkill -f "battery_publisher\.py" || true
pkill -f "cmd_vel_arbiter\.py" || true
pkill -f "tf_relay\.py" || true
pkill -f "topic_tools/relay" || true
pkill -f "install/cslam/lib/cslam" || true
pkill -f "/opt/ros/humble/lib/nav2_" || true
pkill -f "/opt/ros/humble/lib/slam_toolbox" || true
pkill -f "robot_state_publisher" || true
pkill -f "parameter_bridge" || true
pkill -f "static_transform_publisher" || true
pkill -f "rviz2" || true
pkill -f "ign gazebo" || true
pkill -f "gz sim" || true
pkill -f "gzserver" || true
pkill -f "gzclient" || true
pkill -9 -f "ros2cli.daemon.daemonize" || true
