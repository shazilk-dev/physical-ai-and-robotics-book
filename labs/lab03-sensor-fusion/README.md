# Lab 3: Sensor Fusion (IMU + RealSense Camera)

## Overview

In this lab, you'll implement a **sensor fusion node** that combines data from multiple sensors to estimate a robot's pose and orientation. You'll integrate:

- **IMU (Inertial Measurement Unit)** - for orientation estimation
- **Intel RealSense D435i Camera** - for depth-based obstacle detection
- **Complementary Filter** - for sensor fusion

This lab teaches the core techniques used in humanoid robots, drones, and autonomous vehicles for robust state estimation.

---

## Learning Objectives

By completing this lab, you will:

1. ✅ Subscribe to IMU sensor data (`/imu/data`)
2. ✅ Subscribe to RealSense depth data (`/camera/depth/image_rect_raw`)
3. ✅ Implement a complementary filter for orientation estimation
4. ✅ Fuse accelerometer and gyroscope data
5. ✅ Publish fused state estimates to `/robot/state`
6. ✅ Detect obstacles using depth data
7. ✅ Handle sensor noise and calibration
8. ✅ Implement real-time state estimation at 50Hz

**Estimated Time:** 60-90 minutes

---

## Prerequisites

**Knowledge:**
- ROS 2 publishers and subscribers (Lab 1)
- Basic trigonometry (sin, cos, atan2)
- Understanding of IMU sensors (Chapter 1.3.1)

**Hardware (Optional):**
- Intel RealSense D435i camera (simulation works without hardware)
- IMU sensor (or simulated data)

**Software:**
- ROS 2 Humble
- Python 3.10+
- NumPy, SciPy
- OpenCV (cv_bridge)

---

## Robot Specifications

### Sensor Configuration

**IMU Sensor:**
- Topic: `/imu/data` (Type: `sensor_msgs/Imu`)
- Frequency: 100 Hz
- Data: Linear acceleration (m/s²), angular velocity (rad/s)
- Noise: Gaussian with σ = 0.01 m/s²

**RealSense D435i Camera:**
- Depth Topic: `/camera/depth/image_rect_raw` (Type: `sensor_msgs/Image`)
- RGB Topic: `/camera/color/image_raw` (Type: `sensor_msgs/Image`)
- Resolution: 640x480
- Frequency: 30 Hz
- Range: 0.3m - 10m

**Fused State Output:**
- Topic: `/robot/state` (Type: `geometry_msgs/PoseStamped`)
- Frequency: 50 Hz
- Data: Roll, pitch, yaw (orientation)

---

## Implementation Tasks

### Phase 1: IMU Subscriber (TODOs 1-3)

**TODO 1: Create IMU subscriber**
```python
# Subscribe to /imu/data topic
# Use callback: self.imu_callback
```

**TODO 2: Extract IMU data**
```python
# Extract linear acceleration (x, y, z)
# Extract angular velocity (x, y, z)
```

**TODO 3: Calculate roll and pitch from accelerometer**
```python
# roll = atan2(accel_y, accel_z)
# pitch = atan2(-accel_x, sqrt(accel_y² + accel_z²))
```

### Phase 2: Complementary Filter (TODOs 4-6)

**TODO 4: Integrate gyroscope data**
```python
# roll_gyro = previous_roll + gyro_x * dt
# pitch_gyro = previous_pitch + gyro_y * dt
```

**TODO 5: Implement complementary filter**
```python
# alpha = 0.98 (high-pass for gyro, low-pass for accel)
# roll = alpha * roll_gyro + (1 - alpha) * roll_accel
# pitch = alpha * pitch_gyro + (1 - alpha) * pitch_accel
```

**TODO 6: Store filtered values**
```python
# Update self.roll and self.pitch
# Store timestamp for next iteration
```

### Phase 3: RealSense Integration (TODOs 7-9)

**TODO 7: Create depth subscriber**
```python
# Subscribe to /camera/depth/image_rect_raw
# Use callback: self.depth_callback
```

**TODO 8: Process depth image**
```python
# Convert ROS Image to NumPy array (16-bit depth in mm)
# Extract center region (200x200 pixels)
# Calculate average depth
```

**TODO 9: Detect obstacles**
```python
# If avg_depth < 1.0m: obstacle detected
# Publish warning on /robot/obstacles
```

### Phase 4: State Publisher (TODOs 10-12)

**TODO 10: Create state publisher**
```python
# Create publisher on /robot/state (PoseStamped)
# Publish at 50Hz
```

**TODO 11: Populate PoseStamped message**
```python
# Set header.stamp and header.frame_id
# Convert roll/pitch/yaw to quaternion
# Set pose.position and pose.orientation
```

**TODO 12: Add quality metrics**
```python
# Calculate orientation confidence (0-1)
# Log warnings if confidence < 0.7
```

---

## File Structure

```
starter/sensor_fusion_package/
├── __init__.py
├── sensor_fusion_node.py      # YOUR IMPLEMENTATION (12 TODOs)
├── package.xml
└── setup.py

solutions/sensor_fusion_package/
├── sensor_fusion_node.py      # REFERENCE SOLUTION
└── (same structure)

tests/
├── test_sensor_fusion.py      # Automated tests
└── test_calibration.py

assets/
├── imu_calibration.yaml       # Calibration parameters
└── expected_output.txt        # Sample terminal output
```

---

## Running the Lab

### Step 1: Install Dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-cv-bridge \
  python3-numpy \
  python3-opencv
```

### Step 2: Copy Starter Code

```bash
cd ~/ros2_ws/src
cp -r /path/to/labs/lab03-sensor-fusion/starter sensor_fusion_package
```

### Step 3: Build Package

```bash
cd ~/ros2_ws
colcon build --packages-select sensor_fusion_package
source install/setup.bash
```

### Step 4: Run Sensor Fusion Node

```bash
ros2 run sensor_fusion_package sensor_fusion_node
```

### Step 5: Verify Output

```bash
# In another terminal:
ros2 topic echo /robot/state
```

**Expected Output:**
```
header:
  stamp: {sec: 1234567890, nanosec: 123456789}
  frame_id: 'base_link'
pose:
  position: {x: 0.0, y: 0.0, z: 0.0}
  orientation: {x: 0.01, y: -0.02, z: 0.0, w: 0.999}
```

---

## Testing Your Implementation

### Automated Tests

```bash
cd ~/ros2_ws/src/sensor_fusion_package
python3 ../../lab03-sensor-fusion/tests/test_sensor_fusion.py
```

**Test Suite:**
1. ✅ Node starts successfully
2. ✅ IMU subscriber exists on `/imu/data`
3. ✅ Depth subscriber exists on `/camera/depth/image_rect_raw`
4. ✅ State publisher publishes at 50Hz (±5Hz)
5. ✅ Orientation values are within valid range
6. ✅ Complementary filter produces smooth output
7. ✅ Obstacle detection triggers correctly
8. ✅ Quaternion normalization is correct

### Manual Verification

**Test 1: IMU Data Processing**
```bash
# Publish test IMU data
ros2 topic pub /imu/data sensor_msgs/Imu "{
  linear_acceleration: {x: 0.0, y: 0.0, z: 9.81},
  angular_velocity: {x: 0.1, y: 0.0, z: 0.0}
}"

# Should see roll changing gradually
```

**Test 2: Depth Processing**
```bash
# Check depth subscription
ros2 topic list | grep depth

# Echo processed state
ros2 topic echo /robot/state
```

**Test 3: Filter Stability**
```bash
# Record 30 seconds of data
ros2 bag record /imu/data /robot/state

# Analyze for stability (roll/pitch shouldn't drift)
```

---

## Acceptance Criteria

Your implementation must:

- [x] **Subscribe to IMU data** at 100Hz
- [x] **Subscribe to depth data** at 30Hz
- [x] **Publish fused state** at 50Hz
- [x] **Implement complementary filter** with α = 0.98
- [x] **Calculate roll and pitch** from accelerometer
- [x] **Integrate gyroscope data** with proper timestamping
- [x] **Detect obstacles** within 1 meter
- [x] **Output quaternions** (not Euler angles)
- [x] **Handle sensor dropouts** gracefully
- [x] **Pass all 8 automated tests**

**Bonus Challenges:**
- [ ] Add magnetometer for yaw estimation
- [ ] Implement Extended Kalman Filter (EKF)
- [ ] Add sensor health monitoring
- [ ] Visualize orientation in RViz
- [ ] Log fusion metrics to file

---

## Common Issues

### Issue 1: Drift in Roll/Pitch

**Symptom:** Orientation slowly drifts over time

**Solution:**
```python
# Ensure alpha is high (0.98) to trust gyro more
# Verify timestamp calculation: dt = (current_time - last_time)
# Check gyroscope bias removal
```

### Issue 2: Noisy Output

**Symptom:** Roll/pitch values oscillate rapidly

**Solution:**
```python
# Lower alpha (try 0.95) to smooth more
# Add moving average filter:
# filtered = 0.9 * previous + 0.1 * current
```

### Issue 3: Depth Image Not Processing

**Symptom:** No obstacle detection

**Solution:**
```bash
# Check RealSense drivers installed
ros2 pkg list | grep realsense

# Verify depth topic exists
ros2 topic list | grep depth

# Check image encoding (should be 16UC1)
ros2 topic echo /camera/depth/image_rect_raw --once
```

---

## Key Equations

### Complementary Filter
```
roll_fused = α × (roll_prev + ω_x × Δt) + (1 - α) × atan2(a_y, a_z)
```

### Euler from Accelerometer
```
roll = atan2(a_y, a_z)
pitch = atan2(-a_x, √(a_y² + a_z²))
```

### Euler to Quaternion
```python
cy = cos(yaw * 0.5)
sy = sin(yaw * 0.5)
cp = cos(pitch * 0.5)
sp = sin(pitch * 0.5)
cr = cos(roll * 0.5)
sr = sin(roll * 0.5)

qw = cr * cp * cy + sr * sp * sy
qx = sr * cp * cy - cr * sp * sy
qy = cr * sp * cy + sr * cp * sy
qz = cr * cp * sy - sr * sp * cy
```

---

## Next Steps

After completing Lab 3:

1. ✅ Compare with solution: `diff starter/ solutions/`
2. ✅ Try bonus challenges (EKF, magnetometer)
3. ✅ Review Chapter 1.3.3: Sensor Fusion Algorithms
4. ✅ Move to Module 2: Computer Vision for Robotics
5. ✅ Read Section 1.3.2: RealSense Integration

---

## Resources

**ROS 2 Sensor Messages:**
- [sensor_msgs/Imu](https://docs.ros2.org/latest/api/sensor_msgs/msg/Imu.html)
- [sensor_msgs/Image](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html)
- [geometry_msgs/PoseStamped](https://docs.ros2.org/latest/api/geometry_msgs/msg/PoseStamped.html)

**Tutorials:**
- [Complementary Filter Explanation](https://www.youtube.com/watch?v=whSw42XddsU)
- [RealSense ROS 2 Wrapper](https://github.com/IntelRealSense/realsense-ros)
- [robot_localization Package](https://docs.ros.org/en/humble/p/robot_localization/)

**Related Labs:**
- Lab 1: ROS 2 Basics (Heartbeat Publisher)
- Lab 2: URDF Humanoid (Robot Description)
- Lab 4: Visual Servoing (coming soon)

---

**Good luck building your sensor fusion pipeline!** This is the same technique used in drones, self-driving cars, and humanoid robots like Boston Dynamics Atlas.
