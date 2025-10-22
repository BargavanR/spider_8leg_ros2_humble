# 🕷️ Spider 8-Leg Robot (ROS2 Humble)

A modular ROS2-based simulation and control framework for an **8-legged spider robot**, developed on **ROS 2 Humble (Ubuntu 22.04)**.  
This repository contains robot description, control nodes, GUI interface, SLAM modules, and warehouse navigation tasks.

---

## 📦 Repository Overview

| Package | Description |
|----------|-------------|
| `spider_description` | URDF/Xacro model, robot description files, and display launch. |
| `spider_control` | ROS2 control interfaces and controller configurations. |
| `spider_gui` | Python-based GUI for individual joint control. |
| `spider_rviz` | RViz visualization package and display configuration. |
| `spider_slam` | SLAM setup for mapping and localization. |
| `spider_warehouse` | High-level warehouse simulation / navigation. |

---

## 🧩 Prerequisites

Make sure you have the following installed:

1. **Ubuntu 22.04 LTS**
2. **ROS 2 Humble Hawksbill**

   👉 [ROS2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)

3. **Build Tools**
   ```bash
   sudo apt update
   sudo apt install -y python3-colcon-common-extensions python3-pip build-essential
   pip3 install -U setuptools
   ```

4. **ROS Control & Common Packages**
   ```bash
   sudo apt install -y      ros-humble-rclpy      ros-humble-ros2-control      ros-humble-ros2-controllers      ros-humble-xacro      ros-humble-joint-state-controller      ros-humble-controller-manager      ros-humble-gazebo-ros-pkgs
   ```

---

## ⚙️ Workspace Setup

```bash
# Create a workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repository
git clone https://github.com/BargavanR/spider_8leg_ros2_humble.git

# Go to workspace root
cd ~/ros2_ws
```

---

## 🧱 Build Instructions

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build all packages
colcon build --symlink-install

# Source workspace after successful build
source install/setup.bash
```

> ⚠️ Every new terminal must run:
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/ros2_ws/install/setup.bash
> ```

---

## 🚀 Running the System

Below are the main steps to launch and test each part of the robot system.

---

### 🦴 1. Robot Description

Visualize the robot model in RViz.

```bash
ros2 launch spider_description display.launch.py
```

If your launch file name differs, find it using:
```bash
ros2 pkg prefix spider_description
ls $(ros2 pkg prefix spider_description)/share/spider_description/launch
```

---

### 🧠 2. Controller Setup

Bring up controllers for the spider robot.

```bash
ros2 launch spider_control bringup.launch.py
```

If needed, manually spawn controllers:

```bash
ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner position_trajectory_controller
```

---

### 🖥️ 3. Joint Control GUI

Start the **custom GUI** to manually control robot joints.

```bash
ros2 run spider_gui joint_gui.py
```

> 💡 If you get a `ZeroDivisionError` from `sine_wave_motion`, ensure the `duration` parameter is not `0` inside the script.

To check available executables:
```bash
ros2 pkg executables spider_gui
```

---

### 🌍 4. Visualization in RViz

Use the RViz config for the spider model:

```bash
ros2 launch spider_rviz view_spider.launch.py
```

or manually:

```bash
rviz2 -d $(ros2 pkg prefix spider_rviz)/share/spider_rviz/rviz/spider.rviz
```

---

### 🗺️ 5. SLAM Module

Run the SLAM system for mapping/localization.

```bash
ros2 launch spider_slam slam_launch.py
```

---

### 🏭 6. Warehouse Task Demo

Simulate the spider robot performing warehouse navigation or object detection.

```bash
ros2 launch spider_warehouse warehouse_launch.py
```

---

## 🧩 Full System Example (Multi-Terminal Setup)

| Terminal | Command |
|-----------|----------|
| **1** | `ros2 launch spider_description display.launch.py` |
| **2** | `ros2 launch spider_control bringup.launch.py` |
| **3** | `ros2 run spider_gui joint_gui.py` |
| **4** | `ros2 launch spider_slam slam_launch.py` |

This sequence runs the full robot stack — model, control, GUI, and SLAM mapping.

---

## 🔍 Debugging & Tips

### Check available packages
```bash
ros2 pkg list | grep spider
```

### Find launch files
```bash
ros2 pkg prefix spider_control
ls $(ros2 pkg prefix spider_control)/share/spider_control/launch
```

### If `ros2 run` can’t find your node
- Ensure the script is executable:
  ```bash
  chmod +x path/to/script.py
  ```
- Rebuild and re-source your workspace:
  ```bash
  colcon build --symlink-install
  source install/setup.bash
  ```

---

## ⚠️ Common Issues

| Error | Cause | Fix |
|-------|--------|-----|
| `ZeroDivisionError` in `joint_gui.py` | Invalid `duration = 0` | Set valid duration in code |
| `Package not found` | Not sourced or build failed | Run `colcon build` and `source install/setup.bash` |
| `Launch file not found` | Wrong file name | List files in the `launch/` folder |
| Missing dependency | Uninstalled ROS pkg | Install `ros-humble-*` dependency via apt |

---

## 🧰 Developer Notes

You can inspect all installed packages and their executables using:
```bash
ros2 pkg executables spider_*
```

To directly run Python scripts for debugging:
```bash
python3 ~/ros2_ws/src/spider_8leg_ros2_humble/spider_gui/scripts/joint_gui.py
```

---

## 🏁 License

This project is released under the **MIT License** (or update as required).

---

## 👨‍💻 Author

**Bargavan R**  
Robotics & Automation Engineer  
Madras Institute of Technology, Anna University  
📧 [GitHub Profile](https://github.com/BargavanR)

---

> _“Built to explore, adapt, and learn — just like the spider itself.”_ 🕷️
