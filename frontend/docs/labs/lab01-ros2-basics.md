---
sidebar_position: 2
title: Lab 1 - Your First ROS 2 Node
---

# Lab 1: Your First ROS 2 Node - Heartbeat Publisher

## Overview

**Difficulty:** 🟢 Beginner  
**Estimated Time:** 30 minutes  
**Module:** 1.1 ROS 2 Fundamentals

In this lab, you'll build your first functional ROS 2 node - a **heartbeat publisher** that announces your robot is operational. This pattern is used in every production robot for health monitoring and fault detection.

---

## Learning Objectives

By completing this lab, you will:

1. ✅ Create a ROS 2 Python package from scratch
2. ✅ Implement a publisher node with timer callbacks
3. ✅ Use ROS 2 CLI tools to verify node operation
4. ✅ Test message frequency and content
5. ✅ Understand the publisher pattern for real-time systems

---

## Prerequisites

### Required Knowledge

- ✅ **Section 1.1.1:** ROS 2 Architecture & Developer Workflow
- ✅ **Section 1.1.2:** rclpy Patterns & Example Nodes
- ✅ Basic Python (functions, classes, imports)

### Required Software

- ROS 2 Humble (Ubuntu 22.04) or Iron (Ubuntu 22.04)
- Python 3.10+
- `colcon` build tool
- `ros2` CLI tools

### Hardware

**None required!** This lab runs entirely in simulation on your workstation.

---

## Lab Structure

The complete lab materials are located in the repository at:

```
/labs/lab01-ros2-basics/
├── README.md                    # Detailed instructions (read this first!)
├── starter/                     # Start here - contains TODOs
│   ├── package.xml
│   ├── setup.py
│   └── heartbeat_package/
│       ├── __init__.py
│       └── heartbeat_node.py
├── solutions/                   # Reference implementation (check after attempting!)
│   ├── package.xml
│   ├── setup.py
│   └── heartbeat_package/
│       ├── __init__.py
│       └── heartbeat_node.py
├── tests/
│   └── test_heartbeat_node.py
└── assets/
    └── expected_output.txt
```

---

## Quick Start

### Step 1: Copy Starter Code

```bash
# Clone or navigate to the repository
cd /path/to/physical-ai-robotics-book

# Copy starter code to your ROS 2 workspace
cp -r labs/lab01-ros2-basics/starter ~/ros2_ws/src/heartbeat_package

# Navigate to workspace
cd ~/ros2_ws
```

### Step 2: Review Instructions

Open the detailed README:

```bash
cat labs/lab01-ros2-basics/README.md
# Or view in your browser/editor
```

### Step 3: Implement TODOs

Open `~/ros2_ws/src/heartbeat_package/heartbeat_package/heartbeat_node.py` and complete the marked `TODO` sections:

**Your tasks:**

1. ✏️ Create a publisher for `/robot/ready` topic (String messages, queue size 10)
2. ✏️ Create a timer that fires every 1.0 seconds
3. ✏️ Implement `publish_heartbeat()` callback to publish messages
4. ✏️ Log each published message

**Hints:**

- Review Section 1.1.2 for publisher pattern examples
- Message format: `f'READY: Heartbeat #{counter} at {timestamp}'`
- Use `self.get_logger().info()` for logging

### Step 4: Build and Run

```bash
# Build the package
colcon build --packages-select heartbeat_package

# Source the workspace
source install/setup.bash

# Run the node
ros2 run heartbeat_package heartbeat_node
```

**Expected output:**

```
[INFO] [heartbeat_publisher]: Heartbeat node started - publishing to /robot/ready
[INFO] [heartbeat_publisher]: Published: READY: Heartbeat #0 at 2025-12-08T10:30:00.123456
[INFO] [heartbeat_publisher]: Published: READY: Heartbeat #1 at 2025-12-08T10:30:01.123789
[INFO] [heartbeat_publisher]: Published: READY: Heartbeat #2 at 2025-12-08T10:30:02.124012
...
```

### Step 5: Verify with CLI Tools

Open a **second terminal** and run:

```bash
# Source the workspace
source ~/ros2_ws/install/setup.bash

# List all topics (should show /robot/ready)
ros2 topic list

# Echo messages (shows content in real-time)
ros2 topic echo /robot/ready

# Check publish rate (should be ~1 Hz)
ros2 topic hz /robot/ready

# Inspect topic info
ros2 topic info /robot/ready --verbose
```

### Step 6: Run Automated Tests

```bash
cd ~/ros2_ws

# Run tests
colcon test --packages-select heartbeat_package

# View results
colcon test-result --verbose
```

**Expected:** All tests pass ✅

---

## Success Criteria

Your implementation is complete when:

- ✅ Node starts without errors
- ✅ `/robot/ready` topic appears in `ros2 topic list`
- ✅ Messages publish at approximately 1 Hz
- ✅ Message format matches: `READY: Heartbeat #N at <timestamp>`
- ✅ All automated tests pass
- ✅ No warnings or errors in console output

---

## Common Issues & Debugging

### Issue: "Package not found"

**Symptom:** `ros2 run heartbeat_package heartbeat_node` returns "Package not found"

**Solution:**

```bash
# Did you source the workspace?
source ~/ros2_ws/install/setup.bash

# Did you build the package?
cd ~/ros2_ws
colcon build --packages-select heartbeat_package
```

### Issue: Topic not showing up

**Symptom:** `ros2 topic list` doesn't show `/robot/ready`

**Solution:**

- Check if node is actually running (look for log output)
- Verify publisher was created correctly (check your `create_publisher()` call)
- Ensure topic name matches exactly (case-sensitive!)

### Issue: Wrong publish rate

**Symptom:** `ros2 topic hz` shows rate != 1.0 Hz

**Solution:**

- Check timer period: `self.create_timer(1.0, ...)` (1.0 = 1 Hz)
- System under load? Close other applications
- Verify callback isn't blocking (should execute quickly)

### Issue: Tests failing

**Symptom:** `colcon test` reports failures

**Solution:**

- Read the test output carefully - it tells you what's wrong
- Check message format matches exactly
- Verify all required methods are implemented
- Compare your code with the instructions

---

## Extension Challenges (Optional)

Once you've completed the basic lab, try these enhancements:

### Challenge 1: Parameterize Publish Rate

Make the publish rate configurable via ROS 2 parameters:

```python
self.declare_parameter('rate_hz', 1.0)
rate = self.get_parameter('rate_hz').value
self.timer = self.create_timer(1.0 / rate, self.publish_heartbeat)
```

Run with custom rate:

```bash
ros2 run heartbeat_package heartbeat_node --ros-args -p rate_hz:=2.0
```

### Challenge 2: Add Sequence Number Validation

Modify the subscriber to detect missed heartbeats:

```python
expected_seq = self.last_seq + 1
if received_seq != expected_seq:
    self.get_logger().warn(f'Missed {received_seq - expected_seq} heartbeats!')
```

### Challenge 3: Multi-Node System

Create a second node (`heartbeat_monitor`) that:

- Subscribes to `/robot/ready`
- Tracks last heartbeat timestamp
- Publishes `/robot/status` (ALIVE/DEAD) based on 5-second timeout

---

## Reference Solution

After you've attempted the lab, you can check the reference implementation:

```bash
# View solution code
cat /path/to/physical-ai-robotics-book/labs/lab01-ros2-basics/solutions/heartbeat_package/heartbeat_node.py

# Or copy and compare
diff ~/ros2_ws/src/heartbeat_package/heartbeat_package/heartbeat_node.py \
     /path/to/labs/lab01-ros2-basics/solutions/heartbeat_package/heartbeat_node.py
```

:::caution
**Try solving it yourself first!** Copying the solution defeats the learning purpose. Struggle is where growth happens.
:::

---

## What You Learned

✅ **ROS 2 Package Structure:** How to organize Python packages  
✅ **Publisher Pattern:** Creating publishers and timer callbacks  
✅ **Message Types:** Using `std_msgs/String` and message fields  
✅ **CLI Tools:** `ros2 topic`, `ros2 node`, `ros2 run` commands  
✅ **Testing:** Running automated tests with `colcon test`  
✅ **Debugging:** Common issues and how to resolve them

---

## Next Lab

Ready for more? Continue to **Lab 2: Build a URDF Humanoid** (coming soon) to learn robot description formats and visualization.

👉 Lab 2: Build a URDF Humanoid (Coming Soon)

---

## Feedback

Found an issue or have suggestions? [Open an issue on GitHub](https://github.com/shazilk-dev/physical-ai-and-robotics-book/issues) or ask in Discord!
