# Module 1: The Robotic Nervous System (ROS 2)

## Module Overview

Welcome to Module 1, where you'll learn the foundational communication framework that powers modern robots. Just as the human nervous system coordinates billions of sensory inputs and motor outputs, **ROS 2 (Robot Operating System 2)** serves as the nervous system for robots, enabling sensors, planning algorithms, and actuators to work together seamlessly.

**Philosophy:** Code-first approach - you'll build and run ROS 2 nodes from Day 1. ROS 2 isn't just theory; it's the nervous system that connects sensors, planning, and actuation throughout this entire course.

---

## Complete Beginner Setup Guide (Start Here!)

### What You'll Need

Before diving into ROS 2, let's get your development environment ready. Don't worry - we'll walk through every step!

**Required Hardware:**

- 💻 **Computer**: Laptop/desktop with 8GB+ RAM (16GB recommended)
- 🖥️ **OS**: Ubuntu 22.04 LTS (or Windows with WSL2)
- 💾 **Storage**: 20GB free space for ROS 2 and tools

**Optional Hardware (for later labs):**

- 🤖 **Robot**: Unitree Go2, G1, or any ROS 2-compatible robot
- 📷 **Camera**: Intel RealSense D435i ($300)
- 🧭 **IMU**: BNO055 breakout board ($30)
- 🔌 **Compute**: NVIDIA Jetson Orin Nano ($499) for deployment

**Don't have a robot?** No problem! Labs 1-3 work on your laptop. Labs 4-6 use Isaac Sim (virtual robot).

---

### Step-by-Step Installation (Ubuntu 22.04)

#### Step 1: Install Ubuntu 22.04 (If You Don't Have It)

**Option A: Native Ubuntu (Best Performance)**

1. Download Ubuntu 22.04 Desktop: https://ubuntu.com/download/desktop
2. Create bootable USB with Rufus (Windows) or Etcher (Mac)
3. Boot from USB, follow installation wizard
4. Choose "Install Ubuntu alongside Windows" for dual-boot

**Option B: WSL2 on Windows (Good for Learning)**

```powershell
# Open PowerShell as Administrator
wsl --install -d Ubuntu-22.04
# Follow prompts to create username/password
```

**Option C: Virtual Machine (Slower but Safe)**

1. Download VirtualBox: https://www.virtualbox.org/
2. Create VM with 4GB+ RAM, 40GB disk
3. Install Ubuntu 22.04 ISO

---

#### Step 2: Install ROS 2 Humble (30 minutes)

Open a terminal (Ctrl+Alt+T) and run these commands one by one:

```bash
# 1. Set up sources
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu \
  $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 2. Install ROS 2 Humble (this takes 10-15 minutes)
sudo apt update
sudo apt install -y ros-humble-desktop

# 3. Install development tools
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  git

# 4. Initialize rosdep (dependency manager)
sudo rosdep init
rosdep update

# 5. Add ROS 2 to your shell (so you don't have to source every time)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Verify installation:**

```bash
# This should print: humble
ros2 --version

# This should list available commands
ros2 --help
```

✅ **Success!** If you see "humble" and a list of commands, ROS 2 is installed!

---

#### Step 3: Create Your First Workspace (5 minutes)

A **workspace** is a folder where you'll put your robot code. Let's create one:

```bash
# 1. Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# 2. Build the empty workspace (this creates build/ install/ log/ folders)
colcon build

# 3. Source the workspace (tells ROS 2 where your code is)
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 4. Verify workspace
echo $ROS_PACKAGE_PATH
# Should include /home/YOUR_USERNAME/ros2_ws/install
```

**What just happened?**

```
ros2_ws/               # Your workspace root
├── src/               # Put your code here (source files)
├── build/             # Temporary build files (ignore this)
├── install/           # Compiled executables (ROS 2 looks here)
└── log/               # Build logs (for debugging)
```

---

#### Step 4: Install VS Code with ROS Extension (10 minutes)

**Why VS Code?** It has excellent ROS 2 integration (syntax highlighting, auto-complete, launch file debugging).

```bash
# 1. Download and install VS Code
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
sudo sh -c 'echo "deb [arch=amd64] https://packages.microsoft.com/repos/code stable main" \
  > /etc/apt/sources.list.d/vscode.list'

sudo apt update
sudo apt install -y code

# 2. Open VS Code
code ~/ros2_ws
```

**Install ROS Extensions (in VS Code):**

1. Click Extensions icon (left sidebar) or press `Ctrl+Shift+X`
2. Search and install:
   - **ROS** (by Microsoft) - ROS 2 support
   - **Python** (by Microsoft) - Python IntelliSense
   - **CMake** (by twxs) - CMake syntax highlighting
   - **YAML** (by Red Hat) - Launch file editing

---

#### Step 5: Test Your Setup (5 minutes)

Let's verify everything works by running a demo:

```bash
# Terminal 1: Start a publisher (sends messages)
ros2 run demo_nodes_cpp talker

# You should see:
# [INFO] [talker]: Publishing: 'Hello World: 1'
# [INFO] [talker]: Publishing: 'Hello World: 2'
```

Open a **second terminal** (Ctrl+Shift+T):

```bash
# Terminal 2: Start a subscriber (receives messages)
ros2 run demo_nodes_cpp listener

# You should see:
# [INFO] [listener]: I heard: [Hello World: 1]
# [INFO] [listener]: I heard: [Hello World: 2]
```

**What's happening?**

- `talker` node publishes messages to `/chatter` topic
- `listener` node subscribes to `/chatter` topic
- They communicate through ROS 2 middleware (you didn't write any networking code!)

Stop both with `Ctrl+C`.

---

### Troubleshooting Common Issues

**Problem: `ros2: command not found`**

```bash
# Solution: Source ROS 2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Make it permanent:
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

**Problem: `colcon: command not found`**

```bash
# Solution: Install colcon
sudo apt install python3-colcon-common-extensions
```

**Problem: `No module named 'rclpy'`**

```bash
# Solution: Install rclpy
sudo apt install python3-rclpy
```

**Problem: Build fails with "package not found"**

```bash
# Solution: Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y
```

**Still stuck?** Check the [ROS 2 documentation](https://docs.ros.org/en/humble/) or ask in the [ROS Discourse](https://discourse.ros.org/).

---

## Learning Outcomes

By the end of this module, you will be able to:

1. ✅ **Build and run ROS 2 nodes** using rclpy (publishers, subscribers, services, actions)
2. ✅ **Create and validate URDF/XACRO** humanoid robot descriptions
3. ✅ **Acquire, calibrate, and fuse sensor data** (IMU, encoders, RealSense cameras)
4. ✅ **Deploy ROS 2 stacks** to Jetson Orin Nano/NX edge devices
5. ✅ **Understand real-time considerations**, QoS policies, and latency budgeting

## Hardware Context

**Development Environment:**

- **OS:** Ubuntu 22.04 LTS (ROS 2 Humble native)
- **IDE:** VS Code with ROS extensions

**Deployment Target:**

- **Edge Compute:** Jetson Orin Nano (8GB, 100 TOPS, $499) or Orin NX (16GB, 275 TOPS, $899)
- **Sensors:** Intel RealSense D435i depth camera, BNO055 IMU
- **Robot Platform:** Unitree Go2 (quadruped) or G1 (humanoid)

## Module Structure

This module contains 3 chapters covering 12 pages:

### **Chapter 1.1: ROS 2 Fundamentals — Code First (5 pages)**

Learn the architecture, coding patterns, and developer workflow for ROS 2. You'll write your first publisher and subscriber nodes, understand DDS middleware, and master QoS settings.

**Sections:**

- 1.1.1 ROS 2 Architecture & Developer Workflow
- 1.1.2 rclpy Patterns & Example Nodes
- 1.1.3 Parameters, Launch Files & Package Layout
- 1.1.4 Real-Time Considerations & QoS Tuning

**🧪 Lab 1:** Your First ROS 2 Node - Build a heartbeat publisher that announces your robot is ready

### **Chapter 1.2: URDF, Xacro & Robot Description for Humanoids (4 pages)**

Build humanoid robot descriptions using URDF/XACRO, attach sensors and actuators, validate kinematics, and visualize in RViz.

**Sections:**

- 1.2.1 URDF Basics & Xacro Macros
- 1.2.2 Sensors in URDF & Plugin Wiring
- 1.2.3 Validating Kinematics & Joint Limits
- 1.2.4 Package Testing & CI Basics

**🧪 Lab 2:** Create a Simplified Humanoid URDF with Upper Body

### **Chapter 1.3: Sensors & Proprioception — Practical (3 pages)**

Acquire and fuse proprioceptive (encoders, IMU) and exteroceptive (RealSense) sensor data in ROS 2 for reliable state estimation.

**Sections:**

- 1.3.1 IMU & Encoder Basics + Calibration
- 1.3.2 RealSense Camera Integration
- 1.3.3 Sensor Fusion for State Estimation

**🧪 Lab 3:** IMU + Camera Fusion for Robot Orientation

## Why ROS 2 Matters for Physical AI

Traditional software runs on a single computer with predictable timing. **Physical AI systems** are different:

- **Distributed:** Sensors, planning, and actuators run on separate processors
- **Real-Time:** Control loops must execute at 100Hz+ for balance, 500Hz+ for force control
- **Lossy Networks:** WiFi drops packets, but robots can't afford to wait for retransmissions
- **Hardware Diversity:** Same code must run on x86 workstations, ARM edge devices, and FPGA controllers

**ROS 2 solves these challenges** with:

- **DDS Middleware:** Quality of Service (QoS) policies let you choose reliability vs. latency
- **Component Architecture:** Hot-swap nodes without restarting the entire system
- **Security:** Built-in authentication and encryption (critical for commercial deployments)
- **Cross-Platform:** Write once, deploy to Ubuntu workstations and Jetson edge devices

## Real-World Applications (2025)

- **Unitree G1 Humanoid:** Uses ROS 2 for secondary development SDK (1.3m tall, 120 Nm knee torque, $16K)
- **Boston Dynamics Spot:** Internal ROS 2 integration for custom payloads
- **Figure 02 (BMW Factory):** ROS 2 nodes handle navigation and manipulation in production lines
- **NASA VIPER Rover:** ROS 2 drives lunar exploration missions

## Prerequisites

Before starting this module, you should have:

- **Linux Basics:** Command line navigation, file permissions, package management
- **Python Fundamentals:** Classes, functions, imports, virtual environments
- **Git:** Cloning repositories, committing changes
- **Basic Robotics Concepts:** What sensors and actuators do (we'll teach the details)

**Don't worry if you're new to ROS!** This module assumes zero ROS experience and builds from first principles.

## Next Steps

Ready to build your first ROS 2 node? Let's dive into **Chapter 1.1: ROS 2 Fundamentals** where you'll understand the architecture and write executable code within minutes.

🚀 [Start Chapter 1.1: ROS 2 Fundamentals →](./ros2-fundamentals/1.1.1-architecture.md)

---

## Quick Reference: ROS 2 Cheat Sheet

### Essential CLI Commands

```bash
# Node Management
ros2 run <package> <executable>           # Run a node
ros2 node list                            # List running nodes
ros2 node info /node_name                 # Show node details

# Topic Operations
ros2 topic list                           # List all topics
ros2 topic echo /topic_name               # Print messages
ros2 topic hz /topic_name                 # Measure frequency
ros2 topic info /topic_name               # Show topic details
ros2 topic pub /topic std_msgs/String "data: 'hello'"  # Publish once

# Service Operations
ros2 service list                         # List services
ros2 service call /service_name std_srvs/Trigger  # Call service

# Parameter Operations
ros2 param list                           # List all parameters
ros2 param get /node_name param_name      # Get parameter value
ros2 param set /node_name param_name value  # Set parameter

# Package Management
ros2 pkg create <name> --build-type ament_python  # Create package
colcon build --packages-select <name>    # Build one package
colcon build                              # Build workspace
source install/setup.bash                 # Source workspace
```

### Common Message Types

| Type         | Package         | Fields                             | Example Use       |
| ------------ | --------------- | ---------------------------------- | ----------------- |
| `String`     | `std_msgs`      | `data`                             | Simple text       |
| `Int32`      | `std_msgs`      | `data`                             | Integer           |
| `Float64`    | `std_msgs`      | `data`                             | Floating point    |
| `Bool`       | `std_msgs`      | `data`                             | True/False        |
| `Image`      | `sensor_msgs`   | `width`, `height`, `data`          | Camera            |
| `Imu`        | `sensor_msgs`   | `orientation`, `angular_velocity`  | IMU               |
| `LaserScan`  | `sensor_msgs`   | `ranges`, `angle_min`, `angle_max` | LiDAR             |
| `Twist`      | `geometry_msgs` | `linear`, `angular`                | Velocity commands |
| `Pose`       | `geometry_msgs` | `position`, `orientation`          | Robot position    |
| `JointState` | `sensor_msgs`   | `name`, `position`, `velocity`     | Joint data        |

### Code Templates

**Minimal Publisher:**

```python
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('publisher')
        self.pub = self.create_publisher(String, '/topic', 10)
        self.timer = self.create_timer(1.0, self.callback)

    def callback(self):
        msg = String()
        msg.data = 'Hello'
        self.pub.publish(msg)
```

**Minimal Subscriber:**

```python
class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.sub = self.create_subscription(
            String, '/topic', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
```

### Troubleshooting Guide

| Problem                   | Solution                                              |
| ------------------------- | ----------------------------------------------------- |
| `ros2: command not found` | `source /opt/ros/humble/setup.bash`                   |
| `package not found`       | `rosdep install --from-paths src --ignore-src -r -y`  |
| `No module named 'rclpy'` | `sudo apt install python3-rclpy`                      |
| Messages not received     | Check QoS compatibility (`ros2 topic info -v /topic`) |
| Build fails               | `rm -rf build/ install/ log/` then rebuild            |
| Node doesn't appear       | `ros2 daemon stop && ros2 daemon start`               |

---

**Ready? Let's start coding!** 🚀
