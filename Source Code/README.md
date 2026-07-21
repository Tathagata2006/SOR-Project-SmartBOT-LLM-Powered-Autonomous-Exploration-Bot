# LLM-Controlled-Bot

This package contains the source code for a ROS 2 Jazzy based SmartBot implementing autonomous navigation, environment exploration, obstacle tracking, and natural language navigation using a Large Language Model (LLM).

## Contents

### `config/`
Configuration files for Nav2, SLAM Toolbox, AMCL, and other ROS 2 nodes.

### `diff_drive_robot/`
Python source code implementing the robot functionalities, including:
- Keyboard teleoperation
- Goal-based autonomous navigation
- Obstacle avoidance
- Frontier exploration
- Dynamic obstacle tracking
- LLM-based navigation interface

### `launch/`
ROS 2 launch files for starting the robot, simulation environment, SLAM, navigation, and other project components.

### `maps/`
Occupancy grid maps generated during SLAM and used for localization and autonomous navigation.

### `resource/`
ROS 2 package resource files required for package discovery.

### `rviz/`
RViz2 configuration files for visualization of the robot, map, laser scans, and obstacle markers.

### `worlds/`
Gazebo simulation worlds used for testing navigation, exploration, and obstacle avoidance.

## Dependencies

- ROS 2 Jazzy
- Gazebo
- Nav2
- SLAM Toolbox
- RViz2
- Python 3
- Ollama (for LLM-based navigation)

## Build

```bash
cd ~/smartbot_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

This package was developed as part of a ROS 2 SmartBot project demonstrating autonomous navigation, mapping, obstacle tracking, and LLM-assisted robot control.
