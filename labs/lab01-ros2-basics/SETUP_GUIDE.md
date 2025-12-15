# Lab 1 Setup Guide: Complete Installation Instructions

## Quick Start (5 minutes)

```bash
# 1. Navigate to your ROS 2 workspace
cd ~/ros2_ws/src

# 2. Copy the starter code
cp -r /path/to/labs/lab01-ros2-basics/starter heartbeat_package

# 3. Build the package
cd ~/ros2_ws
colcon build --packages-select heartbeat_package

# 4. Source the workspace
source install/setup.bash

# 5. Run the node (after implementing TODOs!)
ros2 run heartbeat_package heartbeat_node
```

---

## Detailed Setup Instructions

### Prerequisites Check

Before starting, verify you have:

```bash
# Check ROS 2 installation
ros2 --version
# Should output: ros2 cli version: ... humble ...

# Check Python version
python3 --version
# Should output: Python 3.10.x or higher

# Check colcon
colcon version-check
# Should show colcon and its extensions
```

If any command fails, revisit Module 1 Overview for installation instructions.

---

## Step-by-Step Setup

### Step 1: Create Workspace (if not exists)

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build empty workspace
colcon build

# Source it
source install/setup.bash

# Add to bashrc (optional, for convenience)
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Step 2: Copy Starter Code

**Option A: Copy from labs directory**

```bash
cd ~/ros2_ws/src

# If you cloned the textbook repository:
cp -r /path/to/physical-ai-robotics-book/labs/lab01-ros2-basics/starter heartbeat_package

# Verify structure
ls heartbeat_package
# Should show: package.xml  setup.py  heartbeat_package/  resource/
```

**Option B: Create manually**

```bash
cd ~/ros2_ws/src
mkdir -p heartbeat_package/heartbeat_package
mkdir -p heartbeat_package/resource
cd heartbeat_package

# Download files from GitHub or copy-paste from textbook
```

### Step 3: Verify Package Structure

Your package should look like this:

```
heart beat_package/
├── package.xml                    # Package metadata
├── setup.py                       # Python package configuration
├── resource/
│   └── heartbeat_package          # Empty marker file
└── heartbeat_package/             # Python package directory
    ├── __init__.py                # Makes this a Python package
    └── heartbeat_node.py          # Your code goes here (TODOs)
```

Verify with:

```bash
cd ~/ros2_ws/src/heartbeat_package
tree  # Or use: find . -type f
```

### Step 4: Build the Package

```bash
cd ~/ros2_ws

# Build just this package
colcon build --packages-select heartbeat_package

# Should see output like:
# Starting >>> heartbeat_package
# Finished <<< heartbeat_package [X.XXs]
```

**Common build errors:**

```bash
# Error: "Package 'heartbeat_package' not found"
# Solution: Make sure you're in ~/ros2_ws and package.xml exists

# Error: "No module named 'heartbeat_package'"
# Solution: Check __init__.py exists in heartbeat_package/

# Error: Permission denied
# Solution: chmod -R +x ~/ros2_ws/src/heartbeat_package
```

### Step 5: Source the Workspace

```bash
# IMPORTANT: Must source after every build!
source ~/ros2_ws/install/setup.bash

# Verify package is visible
ros2 pkg list | grep heartbeat
# Should output: heartbeat_package
```

### Step 6: Run the Starter Node (Before Implementation)

```bash
ros2 run heartbeat_package heartbeat_node

# Expected output (before TODOs):
# [INFO] [heartbeat_publisher]: Heartbeat node started - waiting for implementation...
# (Nothing else happens because TODOs aren't implemented yet)
```

**If you see errors:**

```bash
# Error: "No executable found"
# Solution: Check setup.py entry_points section

# Error: "cannot import name 'heartbeat_node'"
# Solution: Make sure heartbeat_node.py is in heartbeat_package/ directory

# Error: Node crashes immediately
# Solution: Check Python syntax errors with: python3 heartbeat_package/heartbeat_node.py
```

---

## Implementing the Lab

### Step 7: Complete the TODOs

Open `heartbeat_package/heartbeat_node.py` in your favorite editor:

```bash
# Using VS Code
code ~/ros2_ws/src/heartbeat_package/heartbeat_package/heartbeat_node.py

# Using Vim/Nano
vim ~/ros2_ws/src/heartbeat_package/heartbeat_package/heartbeat_node.py
```

**Find and complete these TODOs:**

1. ✅ Create publisher (line ~11)
2. ✅ Create timer (line ~17)
3. ✅ Create String message (line ~25)
4. ✅ Set message data (line ~28)
5. ✅ Publish message (line ~32)
6. ✅ Log message (line ~35)
7. ✅ Increment counter (line ~38)

**Hints are in the code!** Look for `# Hint:` comments.

### Step 8: Rebuild After Changes

```bash
cd ~/ros2_ws
colcon build --packages-select heartbeat_package
source install/setup.bash
```

**Pro tip:** Create an alias to make this faster:

```bash
echo "alias build='cd ~/ros2_ws && colcon build && source install/setup.bash'" >> ~/.bashrc
source ~/.bashrc

# Now just type: build
```

### Step 9: Run Your Implementation

```bash
ros2 run heartbeat_package heartbeat_node

# Expected output (after correct implementation):
# [INFO] [heartbeat_publisher]: ✅ Heartbeat node started - publishing at 1Hz
# [INFO] [heartbeat_publisher]: Published: READY: Heartbeat #0 at 2025-12-15T12:30:00.123456
# [INFO] [heartbeat_publisher]: Published: READY: Heartbeat #1 at 2025-12-15T12:30:01.124012
# ...
```

**Stop the node:** Press `Ctrl+C`

---

## Verification & Testing

### Step 10: Verify with ROS 2 CLI Tools

**While the node is running** (in another terminal):

```bash
# Terminal 1: Run node
ros2 run heartbeat_package heartbeat_node

# Terminal 2: Verify
# List all topics
ros2 topic list
# Should show: /robot/ready

# Echo messages
ros2 topic echo /robot/ready

# Check frequency
ros2 topic hz /robot/ready
# Should show: average rate: 1.000

# Node info
ros2 node info /heartbeat_publisher
```

### Step 11: Run Automated Tests

```bash
# Make sure node is RUNNING in another terminal first!
# Terminal 1:
ros2 run heartbeat_package heartbeat_node

# Terminal 2: Run tests
cd ~/ros2_ws/src/heartbeat_package
python3 ../../tests/test_heartbeat_node.py

# Or copy tests to package:
cp /path/to/labs/lab01-ros2-basics/tests/test_heartbeat_node.py tests/
python3 tests/test_heartbeat_node.py
```

**Expected test output:**

```
test_counter_increments (__main__.TestHeartbeatNode) ... ok
test_message_format (__main__.TestHeartbeatNode) ... ok
test_node_name (__main__.TestHeartbeatNode) ... ok
test_publish_rate (__main__.TestHeartbeatNode) ... ok
test_topic_exists (__main__.TestHeartbeatNode) ... ok

----------------------------------------------------------------------
Ran 5 tests in 18.523s

OK

======================================================================
✅ ALL TESTS PASSED! Lab 1 complete!
======================================================================
```

---

## Troubleshooting Common Issues

### Issue 1: "Package not found" when running

**Symptoms:**
```
Package 'heartbeat_package' not found
```

**Solution:**
```bash
# 1. Rebuild
cd ~/ros2_ws
colcon build --packages-select heartbeat_package

# 2. Source workspace
source install/setup.bash

# 3. Verify package exists
ros2 pkg list | grep heartbeat

# If still fails:
# - Check package.xml has correct name
# - Check setup.py has correct package_name
```

### Issue 2: Node starts but no messages

**Symptoms:**
- Node starts
- No "Published:" logs appear
- `/robot/ready` topic doesn't exist

**Solution:**
```python
# Check these in heartbeat_node.py:

# 1. Publisher created?
self.publisher_ = self.create_publisher(String, '/robot/ready', 10)

# 2. Timer created?
self.timer = self.create_timer(1.0, self.publish_heartbeat)

# 3. Callback implemented?
def publish_heartbeat(self):
    msg = String()
    msg.data = f'READY: Heartbeat #{self.counter} at {datetime.now().isoformat()}'
    self.publisher_.publish(msg)
    self.counter += 1
```

### Issue 3: Messages publish too fast or slow

**Symptoms:**
```
ros2 topic hz /robot/ready
average rate: 0.5  (too slow)
# OR
average rate: 10.0  (too fast)
```

**Solution:**
```python
# Timer period should be 1.0 (seconds)
self.timer = self.create_timer(1.0, self.publish_heartbeat)
#                               ^^^
#                               This number controls frequency
#                               1.0 = 1 Hz (once per second)
```

### Issue 4: Import errors

**Symptoms:**
```
ModuleNotFoundError: No module named 'heartbeat_package'
```

**Solution:**
```bash
# 1. Check __init__.py exists
ls ~/ros2_ws/src/heartbeat_package/heartbeat_package/__init__.py

# 2. If missing, create it:
touch ~/ros2_ws/src/heartbeat_package/heartbeat_package/__init__.py

# 3. Rebuild
cd ~/ros2_ws
colcon build --packages-select heartbeat_package
source install/setup.bash
```

### Issue 5: Tests fail even though node works

**Symptoms:**
- Node runs fine
- Messages look correct
- Tests fail

**Solution:**

1. **Make sure node is RUNNING** when you run tests
   ```bash
   # Terminal 1: MUST be running
   ros2 run heartbeat_package heartbeat_node

   # Terminal 2: Then run tests
   python3 tests/test_heartbeat_node.py
   ```

2. **Check message format exactly**:
   ```
   Correct:   READY: Heartbeat #0 at 2025-12-15T12:30:00.123456
   Wrong:     Ready: Heartbeat #0 at 2025-12-15T12:30:00  (case, format)
   ```

---

## Next Steps After Completion

1. ✅ Compare your solution with `solutions/heartbeat_node.py`
2. ✅ Try the Extension Challenges (README.md)
3. ✅ Move on to Lab 2: URDF Humanoid
4. ✅ Review Section 1.1.3 (Parameters & Launch Files)

---

## Getting Help

- **Check the solution:** `labs/lab01-ros2-basics/solutions/`
- **Review expected output:** `labs/lab01-ros2-basics/assets/expected_output.txt`
- **Revisit Chapter 1.1.2:** Publisher patterns and examples
- **ROS 2 Documentation:** https://docs.ros.org/en/humble/

---

**Good luck! You're building the foundation for humanoid robot programming!** 🤖
