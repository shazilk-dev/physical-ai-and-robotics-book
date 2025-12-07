# Lab 1: Your First ROS 2 Node - Heartbeat Publisher

## Overview

**Difficulty:** Beginner  
**Estimated Time:** 30 minutes  
**Module:** 1.1 ROS 2 Fundamentals

In this lab, you'll build your first functional ROS 2 node - a **heartbeat publisher** that announces your robot is operational. This pattern is used in every production robot for health monitoring and fault detection.

## Learning Objectives

By completing this lab, you will:

1. ✅ Create a ROS 2 Python package from scratch
2. ✅ Implement a publisher node with timer callbacks
3. ✅ Use ROS 2 CLI tools to verify node operation
4. ✅ Test message frequency and content
5. ✅ Understand the publisher pattern for real-time systems

## Prerequisites

**Required Knowledge:**

- Section 1.1.1: ROS 2 Architecture & Developer Workflow
- Section 1.1.2: rclpy Patterns & Example Nodes

**Required Software:**

- ROS 2 Humble (Ubuntu 22.04) or Iron (Ubuntu 22.04)
- Python 3.10+
- `colcon` build tool

## Lab Structure

```
lab01-ros2-basics/
├── README.md                    # This file
├── starter/                     # Start here - contains TODOs
│   ├── package.xml
│   ├── setup.py
│   └── src/
│       └── heartbeat_node.py
├── solutions/                   # Reference implementation
│   ├── package.xml
│   ├── setup.py
│   └── src/
│       └── heartbeat_node.py
├── tests/
│   └── test_heartbeat_node.py
└── assets/
    └── expected_output.txt
```

## Instructions

### Step 1: Set Up Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Copy starter code
cp -r /path/to/labs/lab01-ros2-basics/starter heartbeat_package

cd ~/ros2_ws
```

### Step 2: Implement the Node

Open `starter/src/heartbeat_node.py` and complete the TODOs:

**Your tasks:**

1. Create a publisher for `/robot/ready` topic (String messages, queue size 10)
2. Create a timer that fires every 1.0 seconds
3. Implement `publish_heartbeat()` callback to publish messages
4. Log each published message

**Hints:**

- Review Section 1.1.2 for publisher pattern examples
- Message format: `f'READY: Heartbeat #{counter} at {timestamp}'`
- Use `self.get_logger().info()` for logging

### Step 3: Build and Run

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
[INFO] [heartbeat_publisher]: Heartbeat node started
[INFO] [heartbeat_publisher]: Published: READY: Heartbeat #0 at 2025-12-07T10:30:00
[INFO] [heartbeat_publisher]: Published: READY: Heartbeat #1 at 2025-12-07T10:30:01
...
```

### Step 4: Verify with CLI Tools

```bash
# List topics
ros2 topic list

# Echo messages
ros2 topic echo /robot/ready

# Check frequency (should be ~1Hz)
ros2 topic hz /robot/ready

# Inspect node
ros2 node info /heartbeat_publisher
```

## Acceptance Criteria

- ✅ Publishes to `/robot/ready` with `std_msgs/String` type
- ✅ Publish rate: 1Hz (±0.1Hz)
- ✅ Message includes counter and timestamp
- ✅ Counter increments correctly (0, 1, 2, ...)
- ✅ Node logs each message
- ✅ Graceful shutdown on Ctrl+C

## Troubleshooting

**Package not found:**

```bash
source ~/ros2_ws/install/setup.bash
colcon build --packages-select heartbeat_package
```

**Timer not firing:**

```python
# Wrong: self.timer = self.create_timer(1.0, self.callback())
# Right: self.timer = self.create_timer(1.0, self.callback)
```

**Publisher not working:**

```python
# Must store reference:
self.publisher_ = self.create_publisher(...)
```

## Extension Challenges

1. **Parameterize rate:** Accept `publish_rate` parameter
2. **Add health status:** Publish "INITIALIZING" for first 3 heartbeats
3. **Create subscriber:** Build companion node that counts heartbeats

## Solution

Complete implementation available in `solutions/src/heartbeat_node.py`

## Next Steps

Continue to Section 1.1.3 to learn about launch files and parameters, or proceed to Lab 2 to build a subscriber node.

---

**Need help?** Review Section 1.1.2 or compare with the solution.
