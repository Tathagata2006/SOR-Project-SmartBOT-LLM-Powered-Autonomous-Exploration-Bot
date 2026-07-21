# SOR-Project-SmartBOT-LLM-Powered-Autonomous-Exploration-Bot

A ROS 2 Jazzy-based autonomous mobile robot project implementing manual control, autonomous navigation, SLAM-based exploration, obstacle tracking, and Large Language Model (LLM) based natural language navigation.

---

**Name:** Tathagata Roy <br>
**Roll No.:** 25B3954 <br>

---

## Features

- 🚗 Keyboard Teleoperation
- 🎯 Autonomous Goal Navigation
- 🚧 Obstacle Avoidance using LaserScan
- 🗺️ Frontier-Based Exploration (SLAM)
- 🔴 Dynamic Obstacle Tracking with RViz Visualization
- 🤖 LLM-Based Natural Language Navigation (Ollama)

---

## Technologies Used

- ROS 2 Jazzy
- Gazebo
- RViz2
- Nav2
- SLAM Toolbox
- Python
- Ollama (phi3:mini)

---

## Package Structure

```
diff_drive_robot/
├── launch/
├── config/
├── maps/
├── rviz/
├── worlds/
├── resource/
├── package.xml
├── setup.py
└── ...
```

---

## Implemented Modules

### Keyboard Teleoperation

Allows manual control of the robot using the keyboard.

---

### Autonomous Navigation

Implements goal-based navigation with local obstacle avoidance using LiDAR data.

---

### Frontier Exploration

Automatically explores an unknown environment and constructs an occupancy grid map using frontier-based exploration.

---

### Obstacle Tracker

Detects and tracks moving obstacles using LaserScan data and visualizes them as markers in RViz.

---

### LLM Navigation

Uses Ollama to interpret natural language navigation commands and converts them into navigation goals for Nav2.

Example commands:

```
go to room_a
```

```
go to hallway
```

```
go to charging_dock
```

---

## Build

```bash
cd ~/smartbot_ws

source /opt/ros/jazzy/setup.bash

colcon build

source install/setup.bash
```

---

## Demonstration

The project demonstrates:

- Keyboard Teleoperation
- Goal Navigation with Obstacle Avoidance
- Frontier Exploration and Map Generation
- Dynamic Obstacle Tracking
- Natural Language Navigation using an LLM

---

