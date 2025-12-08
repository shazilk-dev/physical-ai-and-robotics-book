---
sidebar_position: 1
title: Labs & Hands-On Exercises
---

# 🔬 Labs & Hands-On Exercises

## Learning Through Practice

This textbook includes **comprehensive hands-on labs** that progress from basic ROS 2 nodes to voice-controlled autonomous humanoid robots. Each lab is designed to reinforce concepts from the corresponding module through practical, real-world implementation.

:::tip Philosophy
The best way to learn robotics is by **building real systems**. These labs provide starter code, automated tests, and clear success criteria so you can focus on learning, not setup.
:::

---

## 📚 Lab Structure

Every lab follows a consistent format for ease of use:

```
lab-XX-name/
├── README.md          # Instructions, objectives, prerequisites
├── starter/           # Template code with TODOs
├── solutions/         # Reference implementation
├── tests/             # Automated validation scripts
└── assets/            # Expected outputs, diagrams
```

### What's Included

- **Starter Code:** Working template with clearly marked `TODO` sections
- **Step-by-Step Instructions:** Detailed guidance for each task
- **Automated Tests:** Validate your implementation (`colcon test`)
- **Reference Solutions:** Complete working code (check after attempting!)
- **Expected Outputs:** Screenshots and sample data for comparison

---

## 🎯 Complete Labs Index

### Module 1: The Robotic Nervous System (ROS 2)

Build the foundation of robot communication using ROS 2 middleware.

| Lab   | Title                                                 | Difficulty      | Duration | Topics Covered                           |
| ----- | ----------------------------------------------------- | --------------- | -------- | ---------------------------------------- |
| **1** | [Your First ROS 2 Node](/docs/labs/lab01-ros2-basics) | 🟢 Beginner     | 30 min   | Publishers, timers, CLI tools, debugging |
| **2** | Build a URDF Humanoid (Coming Soon)                   | 🟡 Intermediate | 45 min   | URDF/Xacro, sensors, RViz, kinematics    |

**Prerequisites:** Ubuntu 22.04, ROS 2 Humble/Iron installed, basic Python knowledge.

**Hardware:** None required - simulation only.

---

### Module 2: The Digital Twin (Simulation) ⏳ Coming Soon

Master physics simulation and environment building before deploying to real hardware.

| Lab   | Title                   | Difficulty      | Duration | Topics Covered                                      |
| ----- | ----------------------- | --------------- | -------- | --------------------------------------------------- |
| **3** | Gazebo Simulation Setup | 🟡 Intermediate | 60 min   | World files, physics tuning, spawning models        |
| **4** | Sensor Fusion Pipeline  | 🔴 Advanced     | 90 min   | IMU + Camera fusion, Kalman filters, noise modeling |

**Prerequisites:** Module 1 completion, understanding of coordinate frames.

**Hardware:** Workstation with 16GB+ RAM.

---

### Module 3: The AI-Robot Brain (Isaac Sim) ⏳ Coming Soon

Deploy AI perception and navigation systems for autonomous operation.

| Lab   | Title                     | Difficulty  | Duration | Topics Covered                                     |
| ----- | ------------------------- | ----------- | -------- | -------------------------------------------------- |
| **5** | Isaac Sim Perception      | 🔴 Advanced | 90 min   | Synthetic data generation, VSLAM, Nav2 integration |
| **6** | Edge Deployment to Jetson | 🔴 Advanced | 120 min  | Model optimization, TensorRT, Orin deployment      |

**Prerequisites:** Module 1-2 completion, NVIDIA GPU (RTX 3060+).

**Hardware:** NVIDIA Jetson Orin Nano/NX (recommended), RTX-capable workstation.

---

### Module 4: Vision-Language-Action (VLA) ⏳ Coming Soon

Integrate large language models with robotic control for natural language interaction.

| Lab   | Title                  | Difficulty  | Duration | Topics Covered                                    |
| ----- | ---------------------- | ----------- | -------- | ------------------------------------------------- |
| **7** | Voice-Controlled Robot | 🔴 Advanced | 120 min  | Whisper ASR, GPT-4 planning, ROS action execution |

**Prerequisites:** All previous modules, OpenAI API access.

**Hardware:** USB microphone, Jetson Orin (optional).

---

## 🛠️ Hardware Requirements

Labs are designed to work with **minimal hardware investment**, progressively introducing more advanced equipment.

### Tier 1: Simulation Only (Labs 1-4)

**Cost:** $0 (use existing computer)

- Ubuntu 22.04 workstation or VM
- 16GB RAM minimum, 32GB recommended
- No GPU required for Labs 1-2
- GPU (GTX 1660+) recommended for Labs 3-4

**What You'll Learn:** ROS 2 patterns, URDF, Gazebo simulation, sensor fusion.

---

### Tier 2: Edge AI Kit (Labs 5-7)

**Cost:** ~$700

Build the "robot brain" without needing a full robot. This kit replicates the compute and sensors of a real humanoid's head.

| Component   | Model                               | Price | Purpose               |
| ----------- | ----------------------------------- | ----- | --------------------- |
| **Compute** | NVIDIA Jetson Orin Nano Super (8GB) | $249  | Edge AI inference     |
| **Vision**  | Intel RealSense D435i               | $349  | RGB-D camera + IMU    |
| **Audio**   | ReSpeaker USB Mic Array v2.0        | $69   | Far-field voice input |
| **Storage** | 128GB microSD (high-endurance)      | $30   | OS and data           |

**What You'll Learn:** Model optimization, real-time perception, voice control, deployment constraints.

**Alternative:** Cloud GPU instances (AWS g5.2xlarge) ~$1.50/hr if local hardware unavailable.

---

### Tier 3: Physical Robot (Optional)

**Cost:** $1,800 - $16,000

For those who want to deploy to real hardware.

**Budget Option:** Unitree Go2 Edu (~$1,800)

- Quadruped robot with ROS 2 SDK
- Excellent proxy for learning locomotion and navigation
- Shares software stack with humanoids

**Humanoid Option:** Unitree G1 (~$16,000)

- Full-sized humanoid (1.3m, 23-43 DOF)
- High-torque actuators (120 N·m knees)
- NVIDIA Jetson Orin NX onboard
- Production-ready for research/education

:::note
**You do NOT need a physical robot to complete this course!** All labs work in simulation. Physical robots are optional for those pursuing advanced projects or research.
:::

---

## 📖 How to Use Labs

### 1. Read the Theory First

Each lab references specific book sections. Complete those readings before starting the lab.

**Example workflow for Lab 1:**

1. Read Section 1.1.1: ROS 2 Architecture
2. Read Section 1.1.2: rclpy Patterns
3. **Then** start Lab 1: Your First ROS 2 Node

### 2. Work in the Starter Code

Copy the `starter/` directory to your workspace and fill in the `TODO` sections.

```bash
# Copy starter code
cp -r labs/lab01-ros2-basics/starter ~/ros2_ws/src/heartbeat_package

# Navigate to workspace
cd ~/ros2_ws

# Build
colcon build --packages-select heartbeat_package

# Source
source install/setup.bash

# Run
ros2 run heartbeat_package heartbeat_node
```

### 3. Test Your Implementation

Every lab includes automated tests.

```bash
# Run tests
colcon test --packages-select heartbeat_package

# View results
colcon test-result --verbose
```

**Expected:** All tests pass ✅

### 4. Compare with Solution (After Attempting!)

Only look at `solutions/` **after** you've tried implementing it yourself. The struggle is where learning happens!

### 5. Self-Assessment

Use the lab success criteria provided in each lab guide to evaluate your work:

- ✅ All tests pass
- ✅ Code follows ROS 2 conventions (naming, structure)
- ✅ Documentation complete (README, inline comments)
- ✅ Performance meets benchmarks (frequency, latency)

---

## 🎓 Learning Outcomes by Module

### After Module 1 Labs

- ✅ Create ROS 2 packages from scratch
- ✅ Implement publishers and subscribers with proper QoS
- ✅ Build and validate URDF robot descriptions
- ✅ Use ROS 2 CLI tools for debugging
- ✅ Deploy nodes to different environments (simulation, hardware)

### After Module 2 Labs (Coming Soon)

- ✅ Configure Gazebo worlds with custom physics
- ✅ Simulate sensors (cameras, LiDAR, IMU) with realistic noise
- ✅ Fuse multiple sensor streams for robust state estimation
- ✅ Tune PID controllers for stable robot behavior

### After Module 3 Labs (Coming Soon)

- ✅ Generate synthetic training data with Isaac Sim
- ✅ Deploy perception models to Jetson edge devices
- ✅ Implement Visual SLAM for autonomous navigation
- ✅ Optimize models for real-time inference (TensorRT)

### After Module 4 Labs (Coming Soon)

- ✅ Integrate speech recognition (Whisper) with ROS 2
- ✅ Use GPT-4 for high-level task planning
- ✅ Translate natural language to robot actions
- ✅ Build a voice-controlled autonomous robot

---

## 🆘 Getting Help

### Debugging Tips

**Node not showing up in `ros2 node list`?**

- Check if node is actually running (look for process in `ps aux | grep python`)
- Verify you sourced the workspace: `source install/setup.bash`
- Ensure ROS_DOMAIN_ID matches (if using multiple machines)

**Topic not publishing?**

- Check topic name spelling (case-sensitive!)
- Verify message type matches: `ros2 topic info /topic_name`
- Check QoS compatibility: `ros2 topic info /topic_name --verbose`

**Tests failing?**

- Read the test output carefully - it tells you what's wrong
- Check your implementation against the lab instructions
- Verify you filled in ALL `TODO` sections
- Compare with expected output in `assets/` directory

### Community Support

- **GitHub Issues:** Report bugs or unclear instructions
- **Discord Channel:** Ask questions, share solutions (after attempting!)
- **Office Hours:** Weekly live sessions for 1-on-1 help

---

## 📊 Progress Tracking

Your lab progress is automatically tracked:

- ✅ **Completed Labs:** Marked in sidebar with checkmark
- 📊 **Module Progress:** X/Y labs completed
- 🏆 **Achievements:** Unlock badges for milestones (coming soon)

Progress is saved locally (localStorage) and will sync to your account when authentication is added.

---

## 🚀 Ready to Start?

### Recommended Learning Path

**Week 1-2:** Module 1 Theory + Labs 1-2  
Build foundation in ROS 2 and URDF.

**Week 3-4:** Module 2 Theory + Labs 3-4  
Master simulation and sensor fusion.

**Week 5-6:** Module 3 Theory + Labs 5-6  
Deploy AI perception to edge devices.

**Week 7-8:** Module 4 Theory + Lab 7  
Build voice-controlled autonomous robot.

### Start with Lab 1

👉 **[Lab 1: Your First ROS 2 Node](/docs/labs/lab01-ros2-basics)** - Perfect for beginners!

---

## 📝 Feedback

Your feedback helps improve these labs!

- Found a bug? [Report it on GitHub](https://github.com/shazilk-dev/physical-ai-and-robotics-book/issues)
- Have a suggestion? [Open a discussion](https://github.com/shazilk-dev/physical-ai-and-robotics-book/discussions)
- Want to contribute? [Submit a PR](https://github.com/shazilk-dev/physical-ai-and-robotics-book/pulls)

---

**Happy coding! 🤖**
