---
sidebar_position: 4
title: Lab 3 - RealSense D435i Integration
---

# Lab 3: RealSense D435i Integration & Point Cloud Processing

## Overview

**Difficulty:** 🟡 Intermediate
**Estimated Time:** 75 minutes
**Module:** 1.3 Sensors & Proprioception

In this lab, you'll integrate an **Intel RealSense D435i** RGB-D camera with your ROS 2 system and process 3D point cloud data. This is the foundation for object detection, obstacle avoidance, and navigation—critical capabilities for autonomous robots like **Boston Dynamics Spot** and humanoids like **Figure 02**.

---

## Learning Objectives

By completing this lab, you will:

1. ✅ Install and configure the RealSense ROS 2 wrapper
2. ✅ Launch the camera and visualize RGB, depth, and point cloud streams
3. ✅ Process point clouds to detect objects and surfaces
4. ✅ Filter noisy depth data for robust perception
5. ✅ Integrate the camera with your URDF model from Lab 2
6. ✅ Understand camera calibration and coordinate frame transformations

---

## Prerequisites

### Required Knowledge

- ✅ **Section 1.3.1:** IMU and Encoder Basics
- ✅ **Section 1.3.2:** RealSense Integration & Point Clouds
- ✅ **Section 1.3.3:** Sensor Fusion Fundamentals
- ✅ Lab 2: URDF Humanoid Torso (for sensor mounting)
- ✅ Basic understanding of 3D geometry and transformations

### Required Software

- ROS 2 Humble (Ubuntu 22.04)
- RealSense ROS 2 wrapper (`realsense2_camera`)
- Point Cloud Library (PCL) with ROS 2 bindings
- RViz2 with point cloud plugins

### Hardware

**Option 1: Physical Camera** (Recommended)
- Intel RealSense D435i camera (~$379)
- USB 3.0 cable (included)
- Tripod or camera mount

**Option 2: Simulation** (No camera required)
- Pre-recorded ROS 2 bag file with sample data
- Located at `/labs/lab03-realsense-integration/assets/sample_data.bag`

---

## What You'll Build

### System Architecture

```
┌──────────────────┐
│  RealSense D435i │
│  (RGB-D Camera)  │
└────────┬─────────┘
         │ USB 3.0
         ↓
┌──────────────────────────┐
│  realsense2_camera_node  │
│  ├─ /camera/color/image  │ ← RGB stream (1920×1080 @ 30fps)
│  ├─ /camera/depth/image  │ ← Depth map (1280×720 @ 30fps)
│  ├─ /camera/imu          │ ← IMU data (200Hz)
│  └─ /camera/points       │ ← Point cloud (PointCloud2)
└────────┬─────────────────┘
         ↓
┌──────────────────────────┐
│  point_cloud_processor   │ ← Your node (processes clouds)
│  ├─ Voxel downsampling   │
│  ├─ Plane segmentation   │
│  └─ Object clustering    │
└────────┬─────────────────┘
         ↓
┌──────────────────────────┐
│  RViz2 Visualization     │
└──────────────────────────┘
```

---

## Lab Structure

```
/labs/lab03-realsense-integration/
├── README.md                       # Detailed instructions
├── starter/                        # Start here
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── launch/
│   │   ├── camera.launch.py        # Launch RealSense camera
│   │   └── process_cloud.launch.py # Your processing node
│   ├── config/
│   │   └── camera_config.yaml      # Camera parameters
│   ├── realsense_processor/
│   │   ├── __init__.py
│   │   └── point_cloud_processor.py # TODO: Implement this
│   └── urdf/
│       └── camera_mount.urdf.xacro # Add camera to Lab 2 torso
├── solutions/                      # Reference implementation
│   └── realsense_processor/
│       └── point_cloud_processor.py
├── tests/
│   └── test_point_cloud_processing.py
└── assets/
    ├── sample_data.bag             # Pre-recorded data for testing
    └── expected_output.png         # What processed cloud should look like
```

---

## Quick Start

### Step 1: Install RealSense SDK & ROS 2 Wrapper

```bash
# Install librealsense2 (camera SDK)
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"
sudo apt update
sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev

# Install ROS 2 wrapper
sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-description

# Install PCL and ROS 2 PCL conversions
sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions

# Source ROS 2
source /opt/ros/humble/setup.bash
```

### Step 2: Verify Camera Connection (Physical Camera Only)

```bash
# Check if camera is detected
realsense-viewer
# GUI should open showing live camera feed

# Verify USB 3.0 connection (critical for performance!)
lsusb -t
# Look for "Intel Corp. RealSense D435i" under a "5000M" USB 3.0 hub
```

**Troubleshooting:**
- If camera not detected: Try a different USB port (must be USB 3.0/3.1)
- If showing USB 2.0 speeds: Check cable quality or try another port

---

### Step 3: Copy Starter Code

```bash
cd ~/ros2_ws/src
cp -r /path/to/physical-ai-robotics-book/labs/lab03-realsense-integration/starter ./realsense_processor

cd ~/ros2_ws
```

---

## Implementation Guide

### TODO 1: Launch RealSense Camera

**Location:** `launch/camera.launch.py` (Line ~15)

**Task:** Configure the RealSense launch file with optimal parameters.

**Required Parameters:**
```python
parameters=[{
    'rgb_camera.profile': '1920x1080x30',
    'depth_module.profile': '1280x720x30',
    'enable_color': True,
    'enable_depth': True,
    'enable_pointcloud': True,
    'enable_gyro': True,
    'enable_accel': True,
    'pointcloud.stream_filter': 2,  # Texture-based filter
    'decimation_filter.enabled': True,
    'spatial_filter.enabled': True,   # Reduce noise
    'temporal_filter.enabled': True,  # Smooth over time
}]
```

**Why These Settings?**
- **1920×1080 RGB:** High resolution for object recognition
- **1280×720 Depth:** Balanced resolution/framerate for real-time processing
- **Filters enabled:** Crucial for reducing RealSense depth noise

---

### TODO 2: Subscribe to Point Cloud Topic

**Location:** `realsense_processor/point_cloud_processor.py` (Line ~25)

**Task:** Create a ROS 2 node that subscribes to `/camera/depth/color/points`.

**Code Structure:**
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('point_cloud_processor')

        # TODO: Create subscriber to /camera/depth/color/points
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/color/points',
            self.point_cloud_callback,
            10)

        # TODO: Create publisher for processed cloud
        self.publisher = self.create_publisher(
            PointCloud2,
            '/processed_cloud',
            10)

    def point_cloud_callback(self, msg):
        # TODO: Implement processing (next steps)
        pass
```

---

### TODO 3: Convert Point Cloud to NumPy Array

**Location:** `point_cloud_processor.py` (Line ~45)

**Task:** Extract XYZ points from PointCloud2 message.

**Hint:**
```python
def point_cloud_callback(self, msg):
    # Convert ROS PointCloud2 to list of (x, y, z) points
    points = []
    for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        points.append([point[0], point[1], point[2]])

    points_np = np.array(points, dtype=np.float32)
    self.get_logger().info(f'Received cloud with {len(points_np)} points')

    # TODO: Process points (voxel downsample, filter, segment)
```

**Expected Output:** Array of shape `(N, 3)` where N ≈ 300,000-500,000 points

---

### TODO 4: Voxel Downsampling

**Location:** `point_cloud_processor.py` (Line ~60)

**Task:** Reduce point cloud size using voxel grid filtering.

**Why?** Raw point clouds are too dense for real-time processing. Downsampling reduces computational load while preserving shape.

**Implementation:**
```python
import open3d as o3d

def downsample_cloud(self, points_np):
    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_np)

    # TODO: Voxel downsample (leaf size = 0.01m = 1cm)
    downsampled = pcd.voxel_down_sample(voxel_size=0.01)

    return np.asarray(downsampled.points)
```

**Expected Result:** ~10,000-20,000 points (95%+ reduction)

---

### TODO 5: Statistical Outlier Removal

**Location:** `point_cloud_processor.py` (Line ~75)

**Task:** Remove noisy points using statistical filtering.

**Why?** RealSense depth can have "flying pixels" (outliers) that corrupt object detection.

**Implementation:**
```python
def remove_outliers(self, pcd):
    # TODO: Remove points that are >2 std deviations from neighbors
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    # Keep only inlier points
    return pcd.select_by_index(ind)
```

---

### TODO 6: Plane Segmentation (Ground/Table Detection)

**Location:** `point_cloud_processor.py` (Line ~90)

**Task:** Extract the largest planar surface (table/ground) using RANSAC.

**Why?** Robots need to distinguish support surfaces from objects to pick up.

**Implementation:**
```python
def segment_plane(self, pcd):
    # TODO: RANSAC plane fitting
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=0.01,  # 1cm tolerance
        ransac_n=3,               # Min points to fit plane
        num_iterations=1000
    )

    # Separate plane (inliers) and objects (outliers)
    plane_cloud = pcd.select_by_index(inliers)
    object_cloud = pcd.select_by_index(inliers, invert=True)

    # Log plane equation: ax + by + cz + d = 0
    [a, b, c, d] = plane_model
    self.get_logger().info(f'Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0')

    return plane_cloud, object_cloud
```

**Expected Plane Normal:** Approx `[0, 0, 1]` for horizontal table

---

### TODO 7: Euclidean Clustering (Object Segmentation)

**Location:** `point_cloud_processor.py` (Line ~110)

**Task:** Group remaining points into discrete objects.

**Implementation:**
```python
def cluster_objects(self, pcd):
    # TODO: DBSCAN clustering
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(pcd.cluster_dbscan(
            eps=0.02,       # 2cm neighbor threshold
            min_points=10,  # Min 10 points per cluster
            print_progress=False
        ))

    max_label = labels.max()
    self.get_logger().info(f'Found {max_label + 1} objects')

    # Assign random colors to each cluster
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

    return pcd
```

**Expected Output:** 3-8 object clusters (depending on scene complexity)

---

### TODO 8: Publish Processed Cloud

**Location:** `point_cloud_processor.py` (Line ~135)

**Task:** Convert processed cloud back to ROS 2 PointCloud2 message.

**Implementation:**
```python
def publish_cloud(self, points_np):
    # TODO: Create PointCloud2 message
    header = std_msgs.msg.Header()
    header.stamp = self.get_clock().now().to_msg()
    header.frame_id = 'camera_depth_optical_frame'

    cloud_msg = pc2.create_cloud_xyz32(header, points_np.tolist())

    self.publisher.publish(cloud_msg)
```

---

### TODO 9: Add Camera to URDF (Integration with Lab 2)

**Location:** `urdf/camera_mount.urdf.xacro` (Line ~10)

**Task:** Mount RealSense camera on torso from Lab 2.

**Mounting Position:**
```xml
<joint name="camera_joint" type="fixed">
  <parent link="torso"/>
  <child link="camera_link"/>
  <!-- TODO: Position camera at top-front of torso -->
  <origin xyz="0.1 0 0.3" rpy="0 0 0"/>
  <!-- x=0.1m forward, z=0.3m up from torso center -->
</joint>

<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.025 0.09 0.025"/>
    </geometry>
    <material name="camera_black"/>
  </visual>

  <!-- TODO: Add camera optical frame (required for point clouds) -->
</link>

<!-- Optical frame: rotated 90° to match ROS conventions -->
<joint name="camera_optical_joint" type="fixed">
  <parent link="camera_link"/>
  <child link="camera_depth_optical_frame"/>
  <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
  <!-- Rotate to make Z-axis point forward -->
</joint>
```

---

## Testing & Validation

### Test 1: Launch Camera and Verify Topics

```bash
# Terminal 1: Launch camera
ros2 launch realsense_processor camera.launch.py

# Terminal 2: Check topics
ros2 topic list | grep camera
# Should show:
# /camera/color/image_raw
# /camera/depth/image_rect_raw
# /camera/depth/color/points
# /camera/imu

# Check point cloud rate
ros2 topic hz /camera/depth/color/points
# Should show ~30 Hz
```

---

### Test 2: Visualize Raw Point Cloud in RViz

```bash
# Terminal 1: Keep camera running

# Terminal 2: Launch RViz
rviz2

# In RViz:
# 1. Set Fixed Frame to "camera_depth_optical_frame"
# 2. Add → PointCloud2
# 3. Topic: /camera/depth/color/points
# 4. Style: Points, Size: 0.01
```

**What You Should See:**
- 3D point cloud of your surroundings
- Colors match RGB camera feed
- Cloud updates in real-time (30 FPS)

---

### Test 3: Run Processing Node

```bash
# Terminal 1: Camera (keep running)

# Terminal 2: Launch processor
ros2 launch realsense_processor process_cloud.launch.py

# Terminal 3: Visualize processed cloud
rviz2
# Add PointCloud2, topic: /processed_cloud
```

**Expected Results:**
- ✅ Downsampled cloud (~10k points)
- ✅ Plane highlighted in one color (table/ground)
- ✅ Objects clustered in different colors
- ✅ Processing rate: ~10-15 Hz

---

### Test 4: Automated Validation

```bash
cd ~/ros2_ws
colcon test --packages-select realsense_processor
colcon test-result --verbose
```

**Tests Verify:**
1. ✅ Point cloud subscriber receives data
2. ✅ Downsampling reduces points by ≥90%
3. ✅ Plane segmentation finds dominant surface
4. ✅ Clustering detects 1+ objects
5. ✅ Processed cloud is published to `/processed_cloud`

---

## Acceptance Criteria

Your implementation passes if:

1. ✅ Camera launches without errors
2. ✅ `/camera/depth/color/points` publishes at ~30 Hz
3. ✅ Processor node downsamples clouds correctly
4. ✅ Plane segmentation detects table/ground
5. ✅ Object clustering identifies 3+ distinct objects
6. ✅ Automated tests pass 100%
7. ✅ Camera integrated into Lab 2 URDF with correct TF frames

---

## Common Errors & Fixes

### Error: "No devices detected"

**Cause:** Camera not plugged in or USB 3.0 issue

**Fix:**
```bash
# Reconnect camera
# If still fails, check dmesg:
dmesg | tail -50 | grep -i realsense
```

---

### Error: "PointCloud2 timestamp is too old"

**Cause:** Temporal filter buffer lag

**Fix:** Disable temporal filter or reduce `holes_fill` parameter:
```yaml
temporal_filter.enabled: False
```

---

### Error: "Plane segmentation fails"

**Cause:** Scene has no dominant plane, or distance threshold too small

**Fix:**
```python
# Increase distance threshold
plane_model, inliers = pcd.segment_plane(distance_threshold=0.02)  # was 0.01
```

---

### Error: "TF lookup failed: camera_depth_optical_frame"

**Cause:** Camera optical frame not defined in URDF

**Fix:** Ensure you added the optical frame joint in TODO 9 with the correct rotation:
```xml
<origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
```

---

## Extensions (Optional Challenges)

### Challenge 1: Real-Time Object Pose Estimation

Use ICP (Iterative Closest Point) to match detected clusters to CAD models.

**Hint:** Use Open3D's `registration_icp()` function.

---

### Challenge 2: Point Cloud Odometry

Track camera motion by matching consecutive point clouds.

**Hint:** Research ICP-based odometry or use `libpointmatcher`.

---

### Challenge 3: Multi-Camera Fusion

Add a second RealSense camera and merge their point clouds.

**Requirements:**
- Calibrate extrinsic transforms between cameras
- Handle overlapping fields of view
- Voxel downsample merged cloud

---

### Challenge 4: Semantic Segmentation

Train a neural network to classify points (e.g., chair, table, floor).

**Hint:** Use PointNet++ or DGCNN architectures with PyTorch.

---

## Real-World Context

### Why Point Clouds Matter

**Boston Dynamics Spot** uses 5 depth cameras (stereo pairs) for:
- Terrain mapping (detect stairs, obstacles)
- Foothold planning (ensure stable footing)
- Dynamic obstacle avoidance (people, moving objects)

**Figure 02** uses RGB-D cameras for:
- Object recognition (identify car parts)
- Grasp pose estimation (where to grip)
- Collision avoidance (don't hit nearby tools/humans)

**Your pipeline** is a simplified version of these systems!

---

## Troubleshooting Tips

**Point cloud looks noisy?**
```bash
# Increase spatial filter strength
ros2 param set /camera/camera spatial_filter.filter_magnitude 5
# Range: 1-5 (higher = smoother but slower)
```

**Processing too slow?**
```python
# Increase voxel size (faster but coarser)
downsampled = pcd.voxel_down_sample(voxel_size=0.02)  # was 0.01
```

**Clustering finds too many tiny objects?**
```python
# Increase min_points threshold
labels = pcd.cluster_dbscan(eps=0.02, min_points=50)  # was 10
```

---

## Further Reading

### RealSense Documentation
- [RealSense ROS 2 Wrapper](https://github.com/IntelRealSense/realsense-ros)
- [Depth Camera Best Practices](https://dev.intelrealsense.com/docs/tuning-depth-cameras-for-best-performance)

### Point Cloud Processing
- **Open3D Tutorial:** [Point Cloud Processing](http://www.open3d.org/docs/latest/tutorial/geometry/pointcloud.html)
- **PCL Documentation:** [Point Cloud Library](https://pointclouds.org/)

### Academic Papers
- **PointNet++:** "Deep Hierarchical Feature Learning on Point Sets" (NeurIPS 2017)
- **ICP:** "A Method for Registration of 3-D Shapes" (IEEE PAMI 1992)

---

## Summary

🎯 **You've learned:**
- RealSense camera setup and configuration
- Point cloud data structures and processing pipelines
- Downsampling, filtering, and segmentation techniques
- Sensor integration with URDF models

🚀 **Next:** Module 2 will cover the **actuators and compute systems** that drive robots, including motor control and edge AI hardware!

---

**Completed the lab?** Try Challenge 4 (semantic segmentation) and share results with `#PhysicalAI`! 🤖

**Need the dataset?** Download sample ROS 2 bags from the [GitHub repository](https://github.com/shazilk-dev/physical-ai-and-robotics-book/tree/main/labs/lab03-realsense-integration/assets).
