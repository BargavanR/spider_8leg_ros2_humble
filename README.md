# 🕷️ Spider 8-Leg Robot - ROS2 Humble & Ignition Fortress

A comprehensive ROS2-based simulation and control framework for an **8-legged spider robot** with 24 degrees of freedom. Built on **ROS 2 Humble (Ubuntu 22.04)** and **Ignition Gazebo Fortress**, this project provides complete robot description, motion control, GUI interfaces, SLAM capabilities, and warehouse navigation demonstrations.

---

## 📦 Package Architecture

| Package | Purpose |
|---------|---------|
| **spider_description** | URDF/Xacro robot models, Gazebo plugins, launch files, and mesh assets |
| **spider_control** | Gait algorithms, inverse kinematics, and motion primitives |
| **spider_gui** | PyQt-based joint control interface and joystick teleoperation |
| **spider_rviz** | RViz2 visualization configurations and display tools |
| **spider_slam** | Mapping and localization modules for autonomous navigation |
| **spider_warehouse** | Warehouse environment simulation and task demonstrations |

---

## 🔧 System Requirements

### Operating Environment
- **OS**: Ubuntu 22.04 LTS
- **ROS**: ROS 2 Humble Hawksbill
- **Simulator**: Ignition Gazebo Fortress

### Installation Prerequisites

**1. ROS 2 Humble Installation**
```bash
# Follow official installation guide
# https://docs.ros.org/en/humble/Installation.html

# Verify installation
ros2 --version
```

**2. Development Tools**
```bash
sudo apt update
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    build-essential

pip3 install -U setuptools
```

**3. ROS 2 Control & Simulation Packages**
```bash
sudo apt install -y \
    ros-humble-rclpy \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-joint-state-controller \
    ros-humble-controller-manager \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim
```

**4. Ignition Gazebo Fortress**
```bash
sudo apt-get update
sudo apt-get install lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install ignition-fortress
```

---

## 🚀 Quick Start Guide

### Step 1: Workspace Setup

```bash
# Create ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone repository
git clone https://github.com/BargavanR/spider_8leg_ros2_humble.git

# Return to workspace root
cd ~/ros2_ws
```

### Step 2: Dependency Resolution

```bash
# Initialize rosdep (first time only)
sudo rosdep init
rosdep update

# Install package dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### Step 3: Build Workspace

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build all packages
colcon build --symlink-install

# Source workspace overlay
source install/setup.bash
```

### Step 4: Configure Gazebo Resources

```bash
# Add to ~/.bashrc for persistent configuration
echo "export IGN_GAZEBO_RESOURCE_PATH=~/ros2_ws/src:\$IGN_GAZEBO_RESOURCE_PATH" >> ~/.bashrc
source ~/.bashrc
```

### Step 5: Verify Installation

```bash
# Check installed packages
ros2 pkg list | grep spider

# Expected output:
# spider_control
# spider_description
# spider_gui
# spider_rviz
# spider_slam
# spider_warehouse
```

---

## 🎮 Operating the Robot

### Launch Simulation Environment

**Terminal 1: Start Ignition Gazebo**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch spider_description spider_gazebo.launch.py
```

This command:
- Launches Ignition Gazebo Fortress with the configured world
- Loads the spider robot URDF from `spider_description.xacro`
- Publishes robot description to `/robot_description` topic
- Spawns the 8-legged spider entity with all 24 joints
- Initializes `ros2_control` interface via `IgnitionROS2ControlPlugin`

**Terminal 2: Activate Controllers**
```bash
source ~/ros2_ws/install/setup.bash

# Spawn joint state publisher
ros2 run controller_manager spawner joint_state_broadcaster

# Spawn position controller for all joints
ros2 run controller_manager spawner position_controller
```

Controller functions:
- `joint_state_broadcaster` → Publishes real-time joint positions and velocities
- `position_controller` → Accepts trajectory commands for individual joint control

---

### Control Methods

#### 1. **Joint Control GUI**

Manual control interface for testing individual joints:

```bash
ros2 run spider_gui joint_gui.py
```

Features:
- Individual joint angle adjustment with sliders
- Real-time position feedback
- Reset to home position
- Joint limit visualization

#### 2. **Gait Execution Scripts**

Pre-programmed motion primitives for autonomous movement:

```bash
# Forward walking gait
ros2 run spider_control Forward_perfect.py

# Backward walking gait
ros2 run spider_control Backward_perfect.py

# Left turning maneuver
ros2 run spider_control Left_Turn.py

# Right turning maneuver
ros2 run spider_control Right_Turn.py

# Clockwise circular motion
ros2 run spider_control Circle_C.py

# Counter-clockwise circular motion
ros2 run spider_control Circle_CC.py

# Dance demonstration routine
ros2 run spider_control Dancer_spider.py
```

#### 3. **Inverse Kinematics Control**

Direct endpoint control for leg positions:

```bash
ros2 run spider_control INVERSE_KINEMATICS.py
```

Enables Cartesian space control for precise foot placement and body posture adjustments.

#### 4. **Joystick Teleoperation**

Real-time control using game controller:

```bash
ros2 run spider_gui joystick_control.py
```

Supported controllers: Xbox, PlayStation, Logitech gamepads

---

### Visualization Options

#### RViz2 Visualization

```bash
# Launch with custom configuration
ros2 launch spider_rviz launch.py

# Or start manually
rviz2 -d $(ros2 pkg prefix spider_rviz)/share/spider_rviz/rviz/default.rviz
```

#### Combined Gazebo + RViz

```bash
ros2 launch spider_description spider_gz_rviz.launch.py
```

Launches both Ignition Gazebo simulation and RViz2 visualization simultaneously for comprehensive monitoring.

---

## 🗺️ Advanced Features

### SLAM and Mapping

```bash
ros2 launch spider_slam slam_launch.py
```

Enables:
- Real-time environment mapping
- Localization in known environments
- Path planning integration

### Warehouse Navigation Demo

```bash
ros2 launch spider_warehouse warehouse.launch.py
```

Demonstrates:
- Autonomous navigation in structured environments
- Object detection and avoidance
- Multi-goal waypoint navigation
- Task execution in warehouse scenarios

---

## 📂 Workspace Structure

```
~/ros2_ws/
├── src/
│   └── spider_8leg_ros2_humble/
│       ├── spider_description/
│       │   ├── package.xml
│       │   ├── CMakeLists.txt
│       │   ├── launch/
│       │   │   ├── spider_gazebo.launch.py
│       │   │   └── spider_gz_rviz.launch.py
│       │   ├── urdf/
│       │   │   ├── spider_description.xacro
│       │   │   └── spider.gazebo
│       │   ├── config/
│       │   │   ├── controller.yaml
│       │   │   ├── bridge.yaml
│       │   │   └── gaz_ros2_ctl_use_sim.yaml
│       │   ├── meshes/
│       │   │   ├── visual/
│       │   │   └── collision/
│       │   └── worlds/
│       │       └── empty.world
│       │
│       ├── spider_control/
│       │   └── src/
│       │       ├── Forward_perfect.py
│       │       ├── Backward_perfect.py
│       │       ├── Left_Turn.py
│       │       ├── Right_Turn.py
│       │       ├── Circle_C.py
│       │       ├── Circle_CC.py
│       │       ├── INVERSE_KINEMATICS.py
│       │       └── Dancer_spider.py
│       │
│       ├── spider_gui/
│       │   └── src/
│       │       ├── joint_gui.py
│       │       └── joystick_control.py
│       │
│       ├── spider_rviz/
│       │   ├── launch/
│       │   │   └── launch.py
│       │   └── rviz/
│       │       ├── default.rviz
│       │       └── rviz_launch.rviz
│       │
│       ├── spider_slam/
│       │   └── launch/
│       │       └── slam_launch.py
│       │
│       └── spider_warehouse/
│           ├── launch/
│           │   └── warehouse.launch.py
│           ├── worlds/
│           │   └── spider_warehouse.world
│           ├── models/
│           └── meshes/
├── build/
├── install/
└── log/
```

---

## 🛠️ Troubleshooting

### Common Issues and Solutions

| Issue | Cause | Solution |
|-------|-------|----------|
| **Package not found** | Workspace not sourced | Run `source ~/ros2_ws/install/setup.bash` |
| **Build failures** | Missing dependencies | Execute `rosdep install --from-paths src --ignore-src -r -y` |
| **Mesh files not loading** | IGN_GAZEBO_RESOURCE_PATH not set | Add path to `~/.bashrc` as shown in Step 4 |
| **Controllers not spawning** | ros2_control node not running | Ensure `spider_gazebo.launch.py` completed successfully |
| **ZeroDivisionError in GUI** | Invalid duration parameter | Check script for `duration = 0` and set valid value |
| **Joint limits exceeded** | Invalid trajectory commands | Verify joint limits in URDF file |

### Debugging Commands

```bash
# List all active nodes
ros2 node list

# Check controller status
ros2 control list_controllers

# Monitor joint states
ros2 topic echo /joint_states

# View available launch files
ros2 pkg prefix spider_description
ls $(ros2 pkg prefix spider_description)/share/spider_description/launch

# Check executables in package
ros2 pkg executables spider_gui

# Direct Python script execution (debugging)
python3 ~/ros2_ws/src/spider_8leg_ros2_humble/spider_gui/src/joint_gui.py
```

### Gazebo Bridge Issues

If topics aren't bridged between ROS 2 and Ignition:

```bash
# Check bridge configuration
cat ~/ros2_ws/src/spider_8leg_ros2_humble/spider_description/config/bridge.yaml

# List active bridges
ros2 topic list | grep gz
```

---

## 🔄 Development Workflow

### Making Changes

1. **Edit Python scripts** (with `--symlink-install`, no rebuild needed):
   ```bash
   nano ~/ros2_ws/src/spider_8leg_ros2_humble/spider_control/src/Forward_perfect.py
   ```

2. **Modify URDF/Xacro files** (requires rebuild):
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select spider_description
   source install/setup.bash
   ```

3. **Update controller configurations**:
   ```bash
   nano ~/ros2_ws/src/spider_8leg_ros2_humble/spider_description/config/controller.yaml
   colcon build --packages-select spider_description
   ```

### Adding New Gait Patterns

1. Create new Python script in `spider_control/src/`
2. Use existing scripts as templates
3. Update `CMakeLists.txt` to install new script
4. Rebuild workspace:
   ```bash
   colcon build --packages-select spider_control
   ```

---

## 📝 Multi-Terminal Launch Sequence

For complete system operation, use this terminal layout:

| Terminal | Command | Purpose |
|----------|---------|---------|
| **1** | `ros2 launch spider_description spider_gazebo.launch.py` | Simulation environment |
| **2** | `ros2 run controller_manager spawner joint_state_broadcaster && ros2 run controller_manager spawner position_controller` | Controller activation |
| **3** | `ros2 run spider_gui joint_gui.py` | Manual control interface |
| **4** | `ros2 launch spider_rviz launch.py` | Visualization |
| **5** | `ros2 run spider_control Forward_perfect.py` | Gait execution |

---

## 🎯 Project Features

- **24 DOF articulation**: 3 joints per leg × 8 legs
- **Real-time control**: ros2_control integration with Ignition Gazebo
- **Multiple gaits**: Forward, backward, turning, circular motion
- **Inverse kinematics**: Cartesian space control for precise positioning
- **GUI control**: Intuitive interface for testing and debugging
- **SLAM ready**: Mapping and localization capabilities
- **Warehouse navigation**: Autonomous task execution demonstrations
- **Extensible architecture**: Modular design for easy customization

---

## 📚 Additional Resources

- **ROS 2 Documentation**: [docs.ros.org](https://docs.ros.org/en/humble/)
- **Ignition Gazebo**: [gazebosim.org](https://gazebosim.org/)
- **ros2_control**: [control.ros.org](https://control.ros.org/)
- **Project Repository**: [github.com/BargavanR/spider_8leg_ros2_humble](https://github.com/BargavanR/spider_8leg_ros2_humble)

---

## 📄 License

This project is released under the **MIT License**.

---

## 👨‍💻 Author

**Bargavan R**  
Robotics & Automation Engineer  
Madras Institute of Technology, Anna University

📧 [GitHub: @BargavanR](https://github.com/BargavanR)

---

## 🙏 Acknowledgments

Built with ROS 2 Humble and Ignition Gazebo Fortress for advanced legged robot simulation and control research.

---

> *"Engineered for versatility, precision, and autonomous exploration — inspired by nature's most adaptive creatures."* 🕷️
