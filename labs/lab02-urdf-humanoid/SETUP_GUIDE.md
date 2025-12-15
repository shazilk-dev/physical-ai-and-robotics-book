# Lab 2 Setup Guide: URDF Humanoid

## Quick Start (5 minutes)

```bash
# 1. Install required packages
sudo apt install -y \
  ros-humble-rviz2 \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-robot-state-publisher

# 2. Copy starter code
cd ~/ros2_ws/src
cp -r /path/to/labs/lab02-urdf-humanoid/starter humanoid_description

# 3. Build
cd ~/ros2_ws
colcon build --packages-select humanoid_description
source install/setup.bash

# 4. Launch (after implementing TODOs)
ros2 launch humanoid_description display.launch.py
```

---

## Complete Setup Instructions

### Step 1: Install Dependencies

```bash
# Update package list
sudo apt update

# Install ROS 2 visualization tools
sudo apt install -y \
  ros-humble-rviz2 \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-urdf-tutorial

# Verify installation
rviz2 --version
# Should output: rviz version: ...
```

### Step 2: Copy Starter Package

```bash
cd ~/ros2_ws/src

# Option A: Copy from labs directory
cp -r /path/to/physical-ai-robotics-book/labs/lab02-urdf-humanoid/starter humanoid_description

# Option B: Create manually (advanced)
ros2 pkg create humanoid_description --build-type ament_cmake
# Then add urdf/, launch/, rviz/ directories manually
```

### Step 3: Verify Package Structure

```bash
cd ~/ros2_ws/src/humanoid_description

# Should see:
ls -la
# package.xml
# CMakeLists.txt
# urdf/humanoid.urdf
# launch/display.launch.py
# rviz/humanoid.rviz
```

### Step 4: Build Package

```bash
cd ~/ros2_ws

# Build
colcon build --packages-select humanoid_description

# Common errors:
# - "CMakeLists.txt not found" → Check you're in ~/ros2_ws
# - "Package not found" → Ensure package.xml exists and has correct name
```

### Step 5: Source Workspace

```bash
source ~/ros2_ws/install/setup.bash

# Verify package is visible
ros2 pkg list | grep humanoid_description
# Should output: humanoid_description
```

---

## Implementing the Lab

### Step 6: Open URDF File

```bash
# Use your favorite editor
code ~/ros2_ws/src/humanoid_description/urdf/humanoid.urdf

# Or
gedit ~/ros2_ws/src/humanoid_description/urdf/humanoid.urdf
```

### Step 7: Complete TODOs in Order

The URDF has **10 TODOs**. Complete them in order:

**TODOs 1-3: Torso**
- TODO 1: Visual geometry (blue box)
- TODO 2: Collision geometry
- TODO 3: Inertial properties (mass 5kg)

**TODOs 4-5: Head & Neck**
- TODO 4: Head geometry (red sphere)
- TODO 5: Neck joint (revolute, Z-axis)

**TODOs 6-9: Left Arm**
- TODO 6: Left upper arm link
- TODO 7: Left shoulder joints (2 joints!)
- TODO 8: Left forearm link
- TODO 9: Left elbow joint

**TODO 10: Right Arm**
- TODO 10: Mirror entire left arm structure

**Pro tip:** Save after each TODO and validate!

### Step 8: Validate After Each Section

```bash
# Check URDF syntax
check_urdf ~/ros2_ws/src/humanoid_description/urdf/humanoid.urdf

# Expected output (when complete):
# robot name is: simple_humanoid
# ---------- Successfully Parsed XML ---------------
# root Link: torso has 3 child(ren)
#     child(1):  head
#     child(2):  left_shoulder_virtual_link
#     child(3):  right_shoulder_virtual_link
```

### Step 9: Rebuild After Changes

```bash
cd ~/ros2_ws
colcon build --packages-select humanoid_description
source install/setup.bash
```

**Create alias for faster rebuilds:**
```bash
echo "alias rb='cd ~/ros2_ws && colcon build && source install/setup.bash'" >> ~/.bashrc
source ~/.bashrc

# Now just type: rb
```

### Step 10: Launch Visualization

```bash
ros2 launch humanoid_description display.launch.py
```

**What you should see:**
1. **RViz window** opens with robot visualization
2. **Joint State Publisher GUI** opens with sliders
3. **Robot model** appears in RViz (if URDF is valid)

**Test movements:**
- Move neck slider → head rotates
- Move shoulder sliders → arms raise/lower
- Move elbow sliders → forearms bend

---

## Troubleshooting

### Issue 1: Robot doesn't appear in RViz

**Symptoms:**
- RViz opens but shows only grid
- No robot model visible

**Solutions:**

```bash
# 1. Check if robot_state_publisher is running
ros2 node list
# Should show: /robot_state_publisher

# 2. Check robot_description topic
ros2 topic echo /robot_description --once
# Should show your URDF XML

# 3. Check TF frames
ros2 run tf2_tools view_frames
# Generates frames.pdf - should show all links

# 4. In RViz, check:
# - Fixed Frame is set to "torso"
# - RobotModel display is enabled (checkbox)
# - Description Topic is "/robot_description"
```

### Issue 2: Joint sliders don't move robot

**Symptoms:**
- Sliders appear in GUI
- Moving sliders doesn't change robot pose

**Solutions:**

```bash
# 1. Check if joint_states topic is publishing
ros2 topic echo /joint_states

# Should show joint positions updating as you move sliders

# 2. Verify joint names match
# - Joint names in URDF must match names in GUI
# - Check for typos (left_elbow vs left-elbow)

# 3. Restart launch file
# Press Ctrl+C and re-launch
```

### Issue 3: "Failed to build tree" error

**Symptoms:**
```
Error: Failed to build tree: parent link [xxx] of joint [yyy] not found
```

**Solutions:**

1. **Check parent/child link names:**
   ```xml
   <joint name="neck" type="revolute">
     <parent link="torso"/>  <!-- Must exactly match <link name="torso"> -->
     <child link="head"/>     <!-- Must exactly match <link name="head"> -->
   </joint>
   ```

2. **Ensure no circular dependencies:**
   - Each link can have only ONE parent
   - Torso should be the root (no parent)

3. **Verify all links are defined:**
   ```bash
   grep "<link name=" urdf/humanoid.urdf
   # Should show all 8 links
   ```

### Issue 4: check_urdf reports errors

**Common errors:**

**Error: "link [xxx] is not root link and has no parent"**
```xml
<!-- Missing joint connecting this link to tree -->
<!-- Add joint with parent=torso and child=xxx -->
```

**Error: "joint [xxx] missing <axis>"**
```xml
<joint name="neck" type="revolute">
  <axis xyz="0 0 1"/>  <!-- Must specify axis for revolute joints -->
</joint>
```

**Error: "joint [xxx] missing <limit>"**
```xml
<joint name="neck" type="revolute">
  <limit lower="-1.57" upper="1.57" effort="10.0" velocity="2.0"/>
</joint>
```

### Issue 5: Robot appears in weird pose

**Symptoms:**
- Robot appears twisted or upside-down
- Links intersect each other

**Solutions:**

1. **Check origin tags:**
   ```xml
   <!-- xyz: position offset -->
   <!-- rpy: rotation (roll, pitch, yaw) in radians -->
   <origin xyz="0 0 0.45" rpy="0 0 0"/>
   ```

2. **Check joint axes:**
   - X-axis: Red arrow (pitch - forward/backward)
   - Y-axis: Green arrow (roll - left/right tilt)
   - Z-axis: Blue arrow (yaw - rotation around vertical)

3. **Use RViz TF display:**
   - Enable "TF" in Displays panel
   - Shows coordinate frames for each link
   - Verify orientations match expectations

---

## Running Tests

```bash
cd ~/ros2_ws/src/humanoid_description

# Test 1: URDF validity
python3 ../../lab02-urdf-humanoid/tests/test_urdf_validity.py

# Test 2: Joint limits
python3 ../../lab02-urdf-humanoid/tests/test_joint_limits.py
```

**Expected output:**
```
test_01_urdf_file_exists ... ok
test_02_valid_xml ... ok
test_03_robot_has_name ... ok
...
Ran 10 tests in 0.543s

OK

✅ ALL URDF VALIDITY TESTS PASSED!
```

---

## Next Steps After Completion

1. ✅ Compare with solution: `diff starter/urdf/humanoid.urdf solutions/urdf/humanoid.urdf`
2. ✅ Try extension challenges (add hands, use Xacro, add lower body)
3. ✅ Move to Lab 3: Sensor Fusion
4. ✅ Review Section 1.2.3: Validating Kinematics

---

## Useful Commands Reference

```bash
# Validate URDF
check_urdf humanoid.urdf

# View URDF as graph
urdf_to_graphiz humanoid.urdf
# Generates output.gv and output.pdf

# View TF tree
ros2 run tf2_tools view_frames
# Creates frames.pdf

# Echo joint states
ros2 topic echo /joint_states

# List all topics
ros2 topic list

# Launch with custom RViz config
ros2 launch humanoid_description display.launch.py gui:=true
```

---

**Good luck building your humanoid!** Remember: complete TODOs in order, validate after each section, and use the tests to verify correctness. 🤖
