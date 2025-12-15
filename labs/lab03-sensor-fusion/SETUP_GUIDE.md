# Lab 3 Setup Guide: Sensor Fusion

## Quick Start (10 minutes)

```bash
# 1. Install required packages
sudo apt install -y \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-cv-bridge \
  python3-numpy \
  python3-opencv

# 2. Copy starter code
cd ~/ros2_ws/src
cp -r /path/to/labs/lab03-sensor-fusion/starter sensor_fusion_package

# 3. Build
cd ~/ros2_ws
colcon build --packages-select sensor_fusion_package
source install/setup.bash

# 4. Run (after implementing TODOs)
ros2 run sensor_fusion_package sensor_fusion_node
```

---

## Complete Setup Instructions

### Step 1: Install Dependencies

```bash
# Update package list
sudo apt update

# Install ROS 2 sensor packages
sudo apt install -y \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-std-msgs \
  ros-humble-cv-bridge

# Install Python libraries
sudo apt install -y \
  python3-numpy \
  python3-opencv \
  python3-scipy

# Optional: Install RealSense SDK (for hardware testing)
sudo apt install -y \
  ros-humble-realsense2-camera \
  ros-humble-realsense2-description

# Verify installations
python3 -c "import cv2; import numpy; print('✓ OpenCV and NumPy installed')"
ros2 pkg list | grep sensor_msgs
# Should show: sensor_msgs
```

### Step 2: Copy Starter Package

```bash
cd ~/ros2_ws/src

# Option A: Copy from labs directory
cp -r /path/to/physical-ai-robotics-book/labs/lab03-sensor-fusion/starter sensor_fusion_package

# Option B: Create manually (advanced)
ros2 pkg create sensor_fusion_package --build-type ament_python --dependencies rclpy sensor_msgs geometry_msgs std_msgs cv_bridge
# Then copy sensor_fusion_node.py manually
```

### Step 3: Verify Package Structure

```bash
cd ~/ros2_ws/src/sensor_fusion_package

# Should see:
ls -la
# sensor_fusion_package/
#   __init__.py
#   sensor_fusion_node.py
# package.xml
# setup.py
# resource/
#   sensor_fusion_package
```

### Step 4: Build Package

```bash
cd ~/ros2_ws

# Build
colcon build --packages-select sensor_fusion_package

# Common errors:
# - "setup.py not found" → Check you're in ~/ros2_ws
# - "No module named 'cv_bridge'" → Install ros-humble-cv-bridge
# - Python syntax errors → Fix TODOs before building
```

### Step 5: Source Workspace

```bash
source ~/ros2_ws/install/setup.bash

# Verify package is visible
ros2 pkg list | grep sensor_fusion_package
# Should output: sensor_fusion_package

# Check executable is registered
ros2 run sensor_fusion_package sensor_fusion_node --help
```

---

## Implementing the Lab

### Step 6: Open Starter Code

```bash
# Use your favorite editor
code ~/ros2_ws/src/sensor_fusion_package/sensor_fusion_package/sensor_fusion_node.py

# Or
gedit ~/ros2_ws/src/sensor_fusion_package/sensor_fusion_package/sensor_fusion_node.py
```

### Step 7: Complete TODOs in Order

The file has **12 TODOs**. Complete them in order:

**Phase 1: IMU Processing (TODOs 1-6)**
- TODO 1: Create IMU subscriber
- TODO 2: Extract linear acceleration and angular velocity
- TODO 3: Calculate roll/pitch from accelerometer
- TODO 4: Integrate gyroscope data
- TODO 5: Implement complementary filter
- TODO 6: Store filtered values

**Phase 2: Camera Processing (TODOs 7-9)**
- TODO 7: Create depth subscriber
- TODO 8: Process depth image (convert and extract ROI)
- TODO 9: Detect obstacles within 1 meter

**Phase 3: State Publishing (TODOs 10-12)**
- TODO 10: Create state publisher
- TODO 11: Populate PoseStamped message
- TODO 12: Add quality metrics

**Pro tip:** Save after each TODO and test incrementally!

### Step 8: Test After Each Phase

**After Phase 1 (IMU):**
```bash
cd ~/ros2_ws
colcon build --packages-select sensor_fusion_package
source install/setup.bash

# Run node
ros2 run sensor_fusion_package sensor_fusion_node

# In another terminal, publish test IMU data:
ros2 topic pub /imu/data sensor_msgs/Imu "{
  linear_acceleration: {x: 0.0, y: 0.0, z: 9.81},
  angular_velocity: {x: 0.1, y: 0.0, z: 0.0}
}"

# Should see orientation logging in node terminal
```

**After Phase 2 (Camera):**
```bash
# Check depth subscriber exists
ros2 node info /sensor_fusion_node

# Should show:
# Subscriptions:
#   /imu/data: sensor_msgs/msg/Imu
#   /camera/depth/image_rect_raw: sensor_msgs/msg/Image
```

**After Phase 3 (State Publishing):**
```bash
# Check published state
ros2 topic echo /robot/state

# Should show PoseStamped messages at 50Hz
```

### Step 9: Run Automated Tests

```bash
cd ~/ros2_ws/src/sensor_fusion_package
python3 ../../lab03-sensor-fusion/tests/test_sensor_fusion.py
```

**Expected output:**
```
test_01_node_starts ... ok
test_02_imu_subscriber_exists ... ok
test_03_depth_subscriber_exists ... ok
...
Ran 9 tests in 5.123s

OK

✅ ALL SENSOR FUSION TESTS PASSED!
```

---

## Troubleshooting

### Issue 1: Node crashes on startup

**Symptoms:**
```bash
$ ros2 run sensor_fusion_package sensor_fusion_node
Traceback (most recent call last):
  ...
AttributeError: 'SensorFusionNode' object has no attribute 'imu_subscriber'
```

**Solutions:**

1. **Check TODO 1 is completed:**
   ```python
   # TODO 1 should create:
   self.imu_subscriber = self.create_subscription(
       Imu,
       '/imu/data',
       self.imu_callback,
       10
   )
   ```

2. **Verify indentation is correct** (common Python error)

3. **Make sure you're running the right file:**
   ```bash
   ls ~/ros2_ws/src/sensor_fusion_package/sensor_fusion_package/sensor_fusion_node.py
   ```

### Issue 2: Orientation values are crazy (±180°)

**Symptoms:**
```
[INFO] Orientation: Roll=+157.3°, Pitch= +89.2°, Yaw=  +0.0°
[INFO] Orientation: Roll=-178.1°, Pitch= +91.5°, Yaw=  +0.0°
```

**Solutions:**

1. **Check TODO 3 (accelerometer calculation):**
   ```python
   # Should be:
   roll_accel = math.atan2(accel_y, accel_z)
   pitch_accel = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))

   # NOT:
   # roll_accel = accel_y / accel_z  # WRONG!
   ```

2. **Check TODO 5 (complementary filter):**
   ```python
   # Should be:
   roll_fused = self.alpha * roll_gyro + (1 - self.alpha) * roll_accel

   # NOT:
   # roll_fused = roll_gyro + roll_accel  # WRONG!
   ```

3. **Verify alpha is 0.98:**
   ```python
   self.alpha = 0.98  # Should be near 1.0
   ```

### Issue 3: Orientation drifts over time

**Symptoms:**
- Roll/pitch slowly increase even when robot is stationary
- After 1 minute, roll is at 45° but robot hasn't moved

**Solutions:**

1. **Reduce alpha to trust accelerometer more:**
   ```python
   self.alpha = 0.95  # Instead of 0.98
   ```

2. **Check that you're using complementary filter correctly:**
   ```python
   # High alpha = trust gyro MORE (smooth but drifts)
   # Low alpha = trust accel MORE (noisy but no drift)
   ```

3. **Verify gyroscope bias is removed** (advanced):
   ```python
   # After TODO 2:
   gyro_x = msg.angular_velocity.x - self.gyro_bias_x
   gyro_y = msg.angular_velocity.y - self.gyro_bias_y
   gyro_z = msg.angular_velocity.z - self.gyro_bias_z
   ```

### Issue 4: Depth image processing fails

**Symptoms:**
```
[ERROR] Depth processing failed: 'NoneType' object has no attribute 'shape'
```

**Solutions:**

1. **Check TODO 8 image conversion:**
   ```python
   # Should be:
   depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')

   # NOT:
   # depth_image = self.bridge.imgmsg_to_cv2(msg)  # Missing encoding!
   ```

2. **Verify cv_bridge is imported:**
   ```python
   from cv_bridge import CvBridge

   # In __init__:
   self.bridge = CvBridge()
   ```

3. **Check ROS message is valid:**
   ```bash
   ros2 topic echo /camera/depth/image_rect_raw --once
   # Should show image data
   ```

### Issue 5: No obstacle warnings

**Symptoms:**
- Place object within 1 meter
- No warnings appear in terminal
- `/robot/obstacles` topic is empty

**Solutions:**

1. **Check TODO 9 obstacle detection:**
   ```python
   # Should be:
   if avg_depth_m < 1.0:
       obstacle_msg = String()
       obstacle_msg.data = f'OBSTACLE at {avg_depth_m:.2f}m'
       self.obstacle_publisher.publish(obstacle_msg)
   ```

2. **Verify obstacle publisher exists (TODO 10):**
   ```python
   self.obstacle_publisher = self.create_publisher(
       String,
       '/robot/obstacles',
       10
   )
   ```

3. **Check depth data is being received:**
   ```bash
   ros2 topic echo /camera/depth/image_rect_raw --once
   ```

### Issue 6: State not publishing

**Symptoms:**
```bash
$ ros2 topic echo /robot/state
# No output (waiting forever)
```

**Solutions:**

1. **Check TODO 10 (state publisher):**
   ```python
   self.state_publisher = self.create_publisher(
       PoseStamped,
       '/robot/state',
       10
   )
   ```

2. **Check TODO 11 (publish_state method):**
   ```python
   # At end of publish_state():
   self.state_publisher.publish(msg)
   ```

3. **Verify timer is created:**
   ```python
   # In __init__:
   self.create_timer(0.02, self.publish_state)  # 50 Hz
   ```

4. **Check for errors in publish_state:**
   ```bash
   # Run with verbose logging:
   ros2 run sensor_fusion_package sensor_fusion_node --ros-args --log-level debug
   ```

### Issue 7: Tests fail with "No module named 'sensor_fusion_package'"

**Symptoms:**
```bash
$ python3 tests/test_sensor_fusion.py
ImportError: No module named 'sensor_fusion_package'
```

**Solutions:**

1. **Build and source workspace first:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select sensor_fusion_package
   source install/setup.bash
   ```

2. **Add package to PYTHONPATH:**
   ```bash
   export PYTHONPATH=$PYTHONPATH:~/ros2_ws/src
   python3 tests/test_sensor_fusion.py
   ```

3. **Run tests from correct directory:**
   ```bash
   cd ~/ros2_ws/src/sensor_fusion_package
   python3 ../../lab03-sensor-fusion/tests/test_sensor_fusion.py
   ```

### Issue 8: Quaternion not normalized

**Symptoms:**
```
test_08_quaternion_normalized ... FAIL
❌ Quaternion not normalized (magnitude=1.4142)
```

**Solutions:**

1. **Check euler_to_quaternion function:**
   ```python
   # The provided function should be correct
   # If you modified it, revert to original
   ```

2. **Verify you're using the function correctly:**
   ```python
   # In publish_state (TODO 11):
   q = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)
   msg.pose.orientation = q
   ```

3. **Don't manually set quaternion values:**
   ```python
   # WRONG:
   # msg.pose.orientation.w = self.roll
   # msg.pose.orientation.x = self.pitch

   # CORRECT:
   # q = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)
   # msg.pose.orientation = q
   ```

---

## Testing with Simulated Data

If you don't have real hardware, use these commands to test:

### Simulate IMU Data

```bash
# Terminal 1: Run your node
ros2 run sensor_fusion_package sensor_fusion_node

# Terminal 2: Publish IMU data at 100Hz
ros2 topic pub -r 100 /imu/data sensor_msgs/Imu "{
  header: {frame_id: 'imu_link'},
  linear_acceleration: {x: 0.0, y: 0.0, z: 9.81},
  angular_velocity: {x: 0.1, y: 0.0, z: 0.0}
}"

# Should see orientation changing:
# Roll increases due to gyro_x = 0.1 rad/s
```

### Simulate Depth Data

```bash
# Create test depth publisher (Python script):
cat > test_depth_publisher.py << 'EOF'
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class TestDepthPublisher(Node):
    def __init__(self):
        super().__init__('test_depth_publisher')
        self.publisher = self.create_publisher(Image, '/camera/depth/image_rect_raw', 10)
        self.bridge = CvBridge()
        self.create_timer(0.033, self.publish_depth)  # 30 Hz

    def publish_depth(self):
        # Create fake depth image (500mm = 0.5m obstacle)
        depth_array = np.full((480, 640), 500, dtype=np.uint16)
        msg = self.bridge.cv2_to_imgmsg(depth_array, encoding='16UC1')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_depth_optical_frame'
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = TestDepthPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
EOF

chmod +x test_depth_publisher.py
python3 test_depth_publisher.py

# Should trigger obstacle warnings in sensor_fusion_node
```

---

## Running Tests

### Test Suite 1: Main Functionality

```bash
cd ~/ros2_ws/src/sensor_fusion_package
python3 ../../lab03-sensor-fusion/tests/test_sensor_fusion.py
```

**Tests:**
1. Node starts without errors
2. IMU subscriber exists
3. Depth subscriber exists
4. State publisher publishes at 50Hz
5. Orientation values are valid
6. Complementary filter is smooth
7. Obstacle detection works
8. Quaternions are normalized

### Test Suite 2: Calibration

```bash
python3 ../../lab03-sensor-fusion/tests/test_calibration.py
```

**Tests:**
1. Alpha parameter is valid (0.9-0.99)
2. Initial orientation is zero
3. Gravity calculation is correct
4. Roll direction is correct
5. Pitch direction is correct
6. Gyroscope integration works
7. Euler to quaternion (identity)
8. Euler to quaternion (90° roll)

---

## Performance Tuning

### Adjust Complementary Filter

```python
# In __init__:
self.alpha = 0.98  # Default

# For fast response, slight drift:
self.alpha = 0.99

# For slow response, no drift:
self.alpha = 0.95

# Balanced:
self.alpha = 0.97
```

### Adjust Update Rates

```python
# State publishing rate (default: 50 Hz)
self.create_timer(0.02, self.publish_state)  # 50 Hz
self.create_timer(0.01, self.publish_state)  # 100 Hz (faster)
self.create_timer(0.04, self.publish_state)  # 25 Hz (slower)

# Orientation logging rate (default: ~5 Hz)
if self._imu_callback_count % 20 == 0:  # Log every 20 callbacks
if self._imu_callback_count % 50 == 0:  # Less frequent logging
```

---

## Useful Commands Reference

```bash
# Validate implementation
cd ~/ros2_ws/src/sensor_fusion_package
python3 ../../lab03-sensor-fusion/tests/test_sensor_fusion.py

# Check topics
ros2 topic list
ros2 topic info /robot/state
ros2 topic hz /robot/state

# Echo state (see orientation)
ros2 topic echo /robot/state

# Record data for analysis
ros2 bag record /imu/data /robot/state /robot/obstacles

# View node graph
rqt_graph

# Monitor CPU usage
top -p $(pgrep -f sensor_fusion_node)
```

---

## Hardware Setup (Optional)

If you have real hardware (RealSense D435i):

### Install RealSense ROS 2 Wrapper

```bash
sudo apt install -y ros-humble-realsense2-camera

# Launch RealSense node
ros2 launch realsense2_camera rs_launch.py

# Check topics
ros2 topic list | grep camera
# Should show:
# /camera/color/image_raw
# /camera/depth/image_rect_raw
```

### Install IMU (if separate sensor)

```bash
# For MPU6050/MPU9250 IMU:
sudo apt install -y ros-humble-imu-tools

# Launch IMU node (device-specific)
ros2 run imu_tools imu_node
```

---

## Next Steps After Completion

1. ✅ Compare with solution: `diff starter/ solutions/`
2. ✅ Try bonus challenges:
   - Add magnetometer for yaw estimation
   - Implement Extended Kalman Filter (EKF)
   - Add sensor health monitoring
   - Visualize orientation in RViz
3. ✅ Review Chapter 1.3.3: Sensor Fusion Algorithms
4. ✅ Test with real hardware (RealSense D435i)
5. ✅ Move to Module 2: Computer Vision for Robotics

---

**Good luck with your sensor fusion implementation!** This technique is used in every modern robot, drone, and autonomous vehicle. 🤖
