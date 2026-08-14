
# Autonomous Navigation with Nav2 in Gazebo

<p align="center">
 <img src="https://github.com/user-attachments/assets/407f35a5-722f-4b15-85be-66c02939ad46">
</p>


## Requirements
 - ROS 2 Humble
 - Gazebo Igntion Fortress
 - [Navigation 2](https://navigation.ros.org/build_instructions/index.html#install)

## Setup and build
```
cd ros2_ws/src

git clone https://github.com/mohittalwar23/autonomous_nav2_ignition_humble

cd ..

sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Import source dependencies
pip3 install vcstool
vcs import --input deps.repos src

# Install rosrep dependencies
rosdep install -y -r -i  --from-paths . 

# Make sure ROS2 is sourced (assuming bash, please replace extension as needed)
source /opt/ros/humble/setup.bash

# Build
colcon build

# Make sure the app is sourced (assuming bash, please replace extension as needed)
source install/setup.bash
```

## Run examples
```
# Launch Gazebo, RViz, and Navigation2
ros2 launch sam_bot_nav2_gz complete_navigation.launch.py

# Set goal poses in RViz or run a navigation example:
ros2 run sam_bot_nav2_gz follow_waypoints.py
ros2 run sam_bot_nav2_gz reach_goal.py
```

## File Structure 
```
ros2_ws
 - build
 - install
 - src/gz_ros2_control
 - src/ros_gz
 - src/sam_bot_nav2_gz



