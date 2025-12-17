---
sidebar_position: 3
title: Lab 2 - URDF Humanoid Torso
---

# Lab 2: URDF Humanoid Torso with 6 DOF Arms

## Overview

**Difficulty:** 🟡 Intermediate
**Estimated Time:** 90 minutes
**Module:** 1.2 URDF & Robot Description

In this lab, you'll build a **URDF model of a humanoid torso** with two 6 degree-of-freedom (DOF) arms. This is the foundation for understanding robot kinematics and is similar to the upper body structure used in humanoids like **Unitree G1**, **Figure 02**, and **Tesla Optimus**.

---

## Learning Objectives

By completing this lab, you will:

1. ✅ Create a complete URDF model with multiple links and joints
2. ✅ Define kinematic chains for dual-arm manipulation
3. ✅ Add visual and collision geometries
4. ✅ Set proper mass/inertia properties for realistic physics
5. ✅ Visualize and validate your URDF in RViz2
6. ✅ Test joint limits and coordinate frame transformations

---

## Prerequisites

### Required Knowledge

- ✅ **Section 1.2.1:** URDF Basics - Links, Joints, and Coordinate Frames
- ✅ **Section 1.2.2:** Sensors in URDF - Adding Cameras and IMUs
- ✅ **Section 1.2.3:** Validating Robot Kinematics
- ✅ Basic XML syntax
- ✅ Understanding of 3D transformations (translation, rotation)

### Required Software

- ROS 2 Humble (Ubuntu 22.04) or Iron
- `joint_state_publisher_gui` package
- `robot_state_publisher` package
- RViz2
- `urdf_tutorial` package (for URDF validation)

### Hardware

**None required!** This lab runs entirely in simulation.

---

## What You'll Build

### Humanoid Torso Specifications

**Kinematic Structure:**
```
           [torso]
          /       \
    [l_shoulder]  [r_shoulder]
         |              |
    [l_upper_arm]  [r_upper_arm]
         |              |
    [l_forearm]    [r_forearm]
         |              |
      [l_hand]       [r_hand]
```

**Total DOF:** 12 (6 per arm)

**Per-Arm Joint Layout:**
1. **Shoulder Pitch** (Y-axis rotation, -π to π)
2. **Shoulder Roll** (X-axis rotation, -π/2 to π/2)
3. **Shoulder Yaw** (Z-axis rotation, -π/2 to π/2)
4. **Elbow Pitch** (Y-axis rotation, 0 to 3π/4)
5. **Wrist Pitch** (Y-axis rotation, -π/2 to π/2)
6. **Wrist Roll** (X-axis rotation, -π to π)

**Dimensions (similar to Unitree G1):**
- Torso: 0.3m width × 0.15m depth × 0.5m height
- Upper arm length: 0.28m
- Forearm length: 0.25m
- Hand length: 0.12m

---

## Lab Structure

The complete lab materials are located in the repository at:

```
/labs/lab02-urdf-humanoid/
├── README.md                    # Detailed instructions
├── starter/                     # Start here - contains TODOs
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── urdf/
│   │   ├── humanoid_torso.urdf  # Main URDF file (fill in TODOs)
│   │   └── materials.xacro      # Color definitions
│   ├── launch/
│   │   └── visualize.launch.py  # RViz launch file
│   └── config/
│       └── urdf.rviz            # RViz configuration
├── solutions/                   # Reference implementation
│   ├── urdf/
│   │   └── humanoid_torso.urdf
│   └── launch/
│       └── visualize.launch.py
├── tests/
│   └── test_urdf_validity.py   # Automated validation
└── assets/
    ├── expected_rviz.png        # What it should look like
    └── joint_limits.yaml        # Expected joint ranges
```

---

## Quick Start

### Step 1: Copy Starter Code

```bash
# Navigate to your ROS 2 workspace
cd ~/ros2_ws/src

# Copy starter code
cp -r /path/to/physical-ai-robotics-book/labs/lab02-urdf-humanoid/starter ./humanoid_description

# Navigate to workspace root
cd ~/ros2_ws
```

### Step 2: Install Dependencies

```bash
# Install required ROS 2 packages
sudo apt update
sudo apt install ros-humble-joint-state-publisher-gui \
                 ros-humble-robot-state-publisher \
                 ros-humble-xacro \
                 ros-humble-urdf-tutorial

# Source ROS 2
source /opt/ros/humble/setup.bash
```

### Step 3: Review Instructions

```bash
cat labs/lab02-urdf-humanoid/README.md
# Or open in your code editor
```

---

## Implementation Guide

### TODO 1: Define the Torso Link

**Location:** `urdf/humanoid_torso.urdf` (Line ~15)

**Task:** Create the base torso link with proper visual, collision, and inertial properties.

**Hints:**
- Use a `<box>` geometry with dimensions 0.3 × 0.15 × 0.5 meters
- Set mass to 5.0 kg (realistic for carbon fiber/aluminum torso)
- Calculate box inertia using: `I_xx = (m/12)(h² + d²)`, `I_yy = (m/12)(w² + d²)`, `I_zz = (m/12)(w² + h²)`

**Expected Structure:**
```xml
<link name="torso">
  <visual>
    <geometry>
      <box size="0.3 0.15 0.5"/>
    </geometry>
    <material name="gray"/>
  </visual>
  <collision>
    <!-- TODO: Add collision geometry (same as visual) -->
  </collision>
  <inertial>
    <!-- TODO: Calculate mass and inertia -->
  </inertial>
</link>
```

---

### TODO 2: Define Left Shoulder Joints

**Location:** `urdf/humanoid_torso.urdf` (Line ~45)

**Task:** Create the 3-DOF shoulder joint chain (pitch → roll → yaw).

**Joint Configuration:**
1. **l_shoulder_pitch_joint**
   - Type: `revolute`
   - Parent: `torso`
   - Child: `l_shoulder_pitch_link`
   - Axis: `0 1 0` (Y-axis)
   - Origin: `xyz="0 0.175 0.2"` (left side of torso, near top)
   - Limits: `lower="-3.14" upper="3.14"` effort="50" velocity="2.0"`

2. **l_shoulder_roll_joint**
   - Parent: `l_shoulder_pitch_link`
   - Child: `l_shoulder_roll_link`
   - Axis: `1 0 0` (X-axis)
   - Limits: `lower="-1.57" upper="1.57"`

3. **l_shoulder_yaw_joint**
   - Parent: `l_shoulder_roll_link`
   - Child: `l_upper_arm`
   - Axis: `0 0 1` (Z-axis)
   - Limits: `lower="-1.57" upper="1.57"`

**Pro Tip:** Use small dummy links between joints for proper kinematic representation.

---

### TODO 3: Define Left Arm Links

**Location:** `urdf/humanoid_torso.urdf` (Line ~80)

**Task:** Create upper arm, forearm, and hand links.

**Specifications:**
- **Upper arm:** Cylinder (radius=0.04m, length=0.28m), mass=1.5kg
- **Forearm:** Cylinder (radius=0.035m, length=0.25m), mass=1.0kg
- **Hand:** Box (0.08 × 0.05 × 0.12m), mass=0.3kg

**Cylinder Inertia Formula:**
```
I_xx = I_yy = (m/12)(3r² + h²)
I_zz = (m/2)r²
```

---

### TODO 4: Define Elbow and Wrist Joints

**Location:** `urdf/humanoid_torso.urdf` (Line ~120)

**Task:** Complete the arm with elbow pitch, wrist pitch, and wrist roll joints.

**Joint Specifications:**
- **Elbow pitch:** 0 to 135° (0 to 2.36 rad) - can't bend backward
- **Wrist pitch:** ±90° (±1.57 rad)
- **Wrist roll:** ±180° (±3.14 rad) - full rotation

---

### TODO 5: Mirror for Right Arm

**Location:** `urdf/humanoid_torso.urdf` (Line ~180)

**Task:** Duplicate left arm structure for the right arm.

**Key Differences:**
- Joint names: `l_` → `r_`
- Y-axis positions: Negate (e.g., `0.175` → `-0.175`)
- Keep all other parameters identical

**Pro Tip:** Use a text editor's find-replace feature carefully!

---

### TODO 6: Add Joint State Publisher Configuration

**Location:** `launch/visualize.launch.py` (Line ~25)

**Task:** Configure the launch file to publish joint states and spawn the robot in RViz.

**Expected Nodes:**
1. `joint_state_publisher_gui` - Interactive sliders for joints
2. `robot_state_publisher` - Publishes TF transforms
3. `rviz2` - 3D visualization

---

## Testing & Validation

### Test 1: URDF Syntax Validation

```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select humanoid_description

# Source the workspace
source install/setup.bash

# Validate URDF syntax
check_urdf src/humanoid_description/urdf/humanoid_torso.urdf
```

**Expected Output:**
```
robot name is: humanoid_torso
---------- Successfully Parsed XML ---------------
root Link: torso has 2 child(ren)
    child(1):  l_shoulder_pitch_link
        child(1):  l_shoulder_roll_link
            child(1):  l_upper_arm
                child(1):  l_forearm
                    child(1):  l_hand
    child(2):  r_shoulder_pitch_link
        [... similar structure for right arm ...]
```

---

### Test 2: Visualize in RViz

```bash
# Launch visualization
ros2 launch humanoid_description visualize.launch.py
```

**What You Should See:**
1. **RViz window** opens with the torso and arms displayed
2. **Joint State Publisher GUI** window with sliders for all 12 joints
3. **TF display** showing coordinate frames

**Interactive Testing:**
- Move the shoulder pitch sliders → Arms should raise/lower
- Move elbow pitch → Forearms should bend
- Move wrist roll → Hands should rotate

**Verify:**
- ✅ No red collision warnings in RViz
- ✅ Joints move within expected ranges (no flipping through pi boundaries)
- ✅ Arms are symmetric
- ✅ All 12 joints are controllable

---

### Test 3: Automated Validation

```bash
# Run automated tests
cd ~/ros2_ws
colcon test --packages-select humanoid_description
colcon test-result --verbose
```

**Tests Check:**
1. ✅ URDF parses without errors
2. ✅ All joint limits are within bounds
3. ✅ Inertia matrices are positive-definite
4. ✅ No collisions in default pose
5. ✅ TF tree is fully connected

---

## Acceptance Criteria

Your URDF passes if:

1. ✅ `check_urdf` reports no errors
2. ✅ RViz displays both arms without warnings
3. ✅ All 12 joints move smoothly in Joint State Publisher GUI
4. ✅ Automated tests pass 100%
5. ✅ Arms reach at least 0.5m from torso center (workspace requirement)

---

## Common Errors & Fixes

### Error: "Failed to parse URDF"

**Cause:** XML syntax error (unclosed tag, typo in attribute name)

**Fix:**
```bash
# XML validators highlight the exact line
xmllint --noout urdf/humanoid_torso.urdf
```

---

### Error: "Link 'X' is not connected to the tree"

**Cause:** Joint parent/child names don't match link names

**Fix:** Verify parent/child in each joint match actual link names exactly (case-sensitive!)

---

### Error: "Inertia matrix is not positive-definite"

**Cause:** Incorrect inertia calculation or negative values

**Fix:** Use the provided formulas carefully. For debugging:
```python
# Check if your inertia makes sense (in Python)
m = 1.5  # mass (kg)
r = 0.04  # radius (m)
h = 0.28  # height (m)
Ixx = (m/12) * (3*r**2 + h**2)
print(f"I_xx should be: {Ixx:.6f}")
```

---

### Error: Arms "flip" or behave unexpectedly

**Cause:** Joint axis direction is reversed

**Fix:** Check `<axis xyz="..."/>` - common mistake is using `"0 -1 0"` instead of `"0 1 0"`

---

## Extensions (Optional Challenges)

### Challenge 1: Add Collision Spheres

Replace box collision geometries with multiple spheres for more accurate physics simulation.

**Hint:** Use `<sphere radius="0.05"/>` geometries at joint locations.

---

### Challenge 2: Parameterize with Xacro

Convert the URDF to Xacro format with parameters for:
- Arm lengths (easy to resize)
- Joint limits (easy to adjust for different actuators)
- Mass properties (test different materials)

**Hint:** Use `<xacro:property>` tags for constants.

---

### Challenge 3: Add End-Effector Tool

Add a simple 2-finger gripper to each hand.

**Specifications:**
- Two prismatic joints (finger opening/closing)
- Finger length: 0.08m
- Grip range: 0 to 0.1m

---

### Challenge 4: Workspace Analysis

Write a Python script to:
1. Sample all joint angles within limits
2. Compute forward kinematics to find end-effector positions
3. Plot the reachable workspace in 3D

**Hint:** Use `kdl_parser` and `PyKDL` for forward kinematics.

---

## Real-World Context

### Why This Matters

This torso design is based on **real humanoid specifications**:

| Robot | Arms | DOF/Arm | Reach | Mass |
|-------|------|---------|-------|------|
| **Unitree G1** | 2 | 6 | 0.65m | 8kg (total upper body) |
| **Figure 02** | 2 | 7 | 0.73m | ~12kg |
| **Tesla Optimus Gen 2** | 2 | 6 | 0.68m | ~10kg |

**Your model:** 2 arms × 6 DOF = 12 DOF (industry-standard configuration)

---

### Next Steps

After mastering this lab:
1. **Lab 3:** Add RealSense cameras and IMU to the torso
2. **Module 2:** Learn about the motors/actuators that drive these joints
3. **Module 3:** Simulate this robot in NVIDIA Isaac Sim with physics

---

## Troubleshooting Tips

**Can't see the robot in RViz?**
```bash
# Check if robot_state_publisher is running
ros2 node list | grep robot_state

# Check TF tree
ros2 run tf2_tools view_frames
# Opens frames.pdf showing coordinate frame connections
```

**Joints not moving?**
```bash
# Verify joint_state_publisher_gui is publishing
ros2 topic echo /joint_states
# Should show changing values as you move sliders
```

**Getting "No transform" errors?**
```bash
# Ensure robot_state_publisher received the URDF
ros2 param get /robot_state_publisher robot_description
# Should output your URDF XML
```

---

## Further Reading

### URDF Specifications
- [URDF XML Specification](http://wiki.ros.org/urdf/XML)
- [URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)

### Humanoid Kinematics
- **Paper:** "Kinematic Design to Improve Ergonomics in Human-Robot Collaboration" (IEEE)
- **Unitree G1 Specs:** [Official Documentation](https://www.unitree.com/g1/)

### Advanced Topics
- **Xacro Macros:** Reduce URDF duplication
- **KDL (Kinematics and Dynamics Library):** Compute forward/inverse kinematics
- **MoveIt 2:** Motion planning for manipulation tasks

---

## Summary

🎯 **You've learned:**
- How to structure complex URDF models with multiple kinematic chains
- Proper mass/inertia properties for realistic simulation
- Joint limit specification for safe operation
- Visualization and validation workflows

🚀 **Next:** Lab 3 will add **sensors** (RealSense camera + IMU) to this torso, making it a complete perception platform!

---

**Need help?** Check the solution in `/labs/lab02-urdf-humanoid/solutions/` or ask in the book's GitHub discussions.

**Completed the lab?** Share your RViz screenshot with `#PhysicalAI` on social media! 📸
