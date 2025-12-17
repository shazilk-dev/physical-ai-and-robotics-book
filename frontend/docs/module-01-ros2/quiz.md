---
sidebar_position: 99
title: Module 1 Quiz
---

# Module 1: Checkpoint Quiz

## Instructions

This quiz tests your understanding of **Module 1: The Robotic Nervous System** covering ROS 2 fundamentals, URDF robot description, and sensor integration.

**Format:**
- 15 multiple-choice questions
- Each question has 4 options (A, B, C, D)
- Only ONE correct answer per question
- Passing score: 12/15 (80%)

**Topics Covered:**
- ROS 2 Architecture & DDS Middleware
- rclpy Patterns (Publishers, Subscribers, Services, Actions)
- URDF/Xacro Robot Description
- Sensor Integration (RealSense, IMU)
- Coordinate Frames & Transformations

**Time Limit:** None (self-paced)

**Answer Key:** Located at the bottom of this page (try to answer first!)

---

## Questions

### Question 1: ROS 2 Architecture

**What is the primary advantage of ROS 2's DDS middleware over ROS 1's TCPROS?**

A) DDS is faster because it uses UDP instead of TCP
B) DDS supports peer-to-peer communication without a central master node
C) DDS requires less memory and CPU resources
D) DDS automatically compresses all message data

<details>
<summary>💡 Hint</summary>
Think about the architecture differences: ROS 1 uses a central roscore master, while ROS 2 uses DDS for discovery.
</details>

---

### Question 2: QoS Profiles

**A robot camera publishes images at 30 FPS. Occasional dropped frames are acceptable, but you want minimal latency. Which QoS profile should you use?**

A) `qos_profile_sensor_data` (Best Effort, Volatile)
B) `qos_profile_system_default` (Reliable, Volatile)
C) `qos_profile_services_default` (Reliable, Keep Last 10)
D) `qos_profile_parameters` (Reliable, Keep All)

<details>
<summary>💡 Hint</summary>
Sensor data like images prioritizes low latency over reliability. Review Section 1.1.4 on QoS profiles.
</details>

---

### Question 3: rclpy Publishers

**What happens if you publish a message BEFORE calling `spin()` or `spin_once()` in rclpy?**

A) The message is lost because subscribers haven't connected yet
B) The message is queued and sent when `spin()` is called
C) The message is sent immediately regardless of subscriber status
D) An exception is raised

<details>
<summary>💡 Hint</summary>
ROS 2 discovery takes time. Review Section 1.1.2 on publisher patterns and the "warm-up period."
</details>

---

### Question 4: Services vs Actions

**You're implementing a "move to waypoint" behavior that takes 5-10 seconds. Which ROS 2 communication pattern should you use?**

A) Service - because it has a request-response structure
B) Action - because it supports feedback and preemption
C) Publisher/Subscriber - because it's asynchronous
D) Parameter - because it stores state

<details>
<summary>💡 Hint</summary>
Long-running tasks need progress feedback and the ability to cancel. Review Section 1.1.2 on actions.
</details>

---

### Question 5: URDF Basics

**In a URDF file, what does a `<joint type="fixed">` represent?**

A) A joint that cannot move (rigid attachment)
B) A joint that has been calibrated and won't drift
C) A joint with a single degree of freedom
D) A joint that requires manual positioning

<details>
<summary>💡 Hint</summary>
Think about sensor mounts or camera attachments that don't move relative to their parent link.
</details>

---

### Question 6: URDF Coordinate Frames

**A robot's camera is mounted 0.2m forward and 0.3m up from the base_link. What origin should you use in the joint?**

A) `<origin xyz="0.2 0.3 0" rpy="0 0 0"/>`
B) `<origin xyz="0.2 0 0.3" rpy="0 0 0"/>`
C) `<origin xyz="0.3 0 0.2" rpy="0 0 0"/>`
D) `<origin xyz="0 0.2 0.3" rpy="0 0 0"/>`

<details>
<summary>💡 Hint</summary>
ROS uses the convention: X=forward, Y=left, Z=up. Review Section 1.2.1 on coordinate frames.
</details>

---

### Question 7: Joint Limits

**Why must you specify `<limit effort="X" velocity="Y"/>` even for fixed joints in URDF?**

A) Fixed joints don't require limits (this is a trick question)
B) The URDF parser requires it for all joint types
C) Limits are needed for Gazebo physics simulation
D) Limits define the joint's maximum load capacity

<details>
<summary>💡 Hint</summary>
Check the URDF XML specification - some parsers are strict about required fields.
</details>

---

### Question 8: Inertia Properties

**A solid cylinder (mass m, radius r, height h) rotates around its central axis (Z). Which inertia formula is correct?**

A) `I_zz = (m/2) * r²`
B) `I_zz = (m/12) * (3r² + h²)`
C) `I_zz = (m) * r²`
D) `I_zz = (m/3) * r²`

<details>
<summary>💡 Hint</summary>
Review Section 1.2.3 on inertia matrices. For a cylinder around its axis, it's simpler than the general formula.
</details>

---

### Question 9: RealSense Depth Range

**What is the effective depth range of the Intel RealSense D435i for accurate measurements?**

A) 0.1m to 3m
B) 0.3m to 10m
C) 0.5m to 5m
D) 1m to 15m

<details>
<summary>💡 Hint</summary>
RealSense cameras have a minimum depth (too close = no data) and maximum range (too far = noise). Review Section 1.3.2.
</details>

---

### Question 10: Point Cloud Downsampling

**You have a point cloud with 500,000 points. After voxel downsampling with leaf_size=0.01m, you get 15,000 points. If you change leaf_size to 0.02m, approximately how many points will remain?**

A) 30,000 points (2× more)
B) 7,500 points (2× fewer)
C) 3,750 points (4× fewer)
D) 1,875 points (8× fewer)

<details>
<summary>💡 Hint</summary>
Doubling the voxel size reduces points cubically: (2³ = 8× fewer points). Review Lab 3 on voxel downsampling.
</details>

---

### Question 11: IMU Coordinate Frames

**An IMU reports angular velocity `[0, 0, 0.5]` rad/s. What motion is the robot experiencing?**

A) Rotating around the X-axis (roll)
B) Rotating around the Y-axis (pitch)
C) Rotating around the Z-axis (yaw)
D) Linear acceleration, not rotation

<details>
<summary>💡 Hint</summary>
Angular velocity is a 3D vector: [roll_rate, pitch_rate, yaw_rate]. Review Section 1.3.1 on IMU data.
</details>

---

### Question 12: Sensor Fusion

**Why do we fuse IMU data with camera odometry instead of using camera alone?**

A) Cameras are too expensive for most robots
B) IMUs provide absolute position while cameras only give relative motion
C) IMUs maintain pose estimation during fast motion or poor lighting when cameras fail
D) IMUs have better accuracy than cameras for all motions

<details>
<summary>💡 Hint</summary>
Think about the complementary strengths: cameras drift over time, IMUs struggle with vibrations. Review Section 1.3.3.
</details>

---

### Question 13: RANSAC Plane Fitting

**In Lab 3, you used RANSAC to detect a table surface. What does the `distance_threshold` parameter control?**

A) The maximum number of iterations RANSAC will run
B) The minimum number of points required to form a plane
C) How far a point can be from the plane and still be considered an inlier
D) The minimum area a plane must have to be detected

<details>
<summary>💡 Hint</summary>
RANSAC classifies points as "inliers" (close to the fitted plane) or "outliers" based on distance.
</details>

---

### Question 14: Temporal vs Spatial Filters (RealSense)

**Your robot needs to detect moving objects. Should you enable the temporal filter on the RealSense depth stream?**

A) Yes - temporal filter reduces noise for cleaner detection
B) No - temporal filter blurs motion and will smear moving objects
C) Yes - temporal filter improves frame rate
D) No - temporal filter only works for static scenes

<details>
<summary>💡 Hint</summary>
Temporal filtering averages frames over time, which smooths noise but can blur fast motion. Review Lab 3 camera config.
</details>

---

### Question 15: End-Effector Workspace

**A 2-link arm has segments of length L1=0.3m and L2=0.25m. If both joints can rotate ±180°, what is the maximum reach from the shoulder?**

A) 0.3m (length of first link only)
B) 0.55m (L1 + L2)
C) 0.6m (2 × L1)
D) 0.05m (L1 - L2)

<details>
<summary>💡 Hint</summary>
Maximum reach occurs when both links are fully extended in the same direction. Review Section 1.2.3 on kinematics.
</details>

---

## Scoring

**Your Score:** _____ / 15

**Grading Scale:**
- **14-15:** Excellent! You've mastered Module 1 🏆
- **12-13:** Good! Minor review needed 🎯
- **10-11:** Pass, but review weak areas before Module 2 ⚠️
- **< 10:** Review Module 1 content before proceeding 🔄

---

## Answer Key

<details>
<summary>🔓 Click to Reveal Answers</summary>

### Answers

1. **B** - DDS supports peer-to-peer communication without a central master node
   - *Explanation:* ROS 1's roscore master is a single point of failure. DDS uses distributed discovery.

2. **A** - `qos_profile_sensor_data` (Best Effort, Volatile)
   - *Explanation:* Sensor data prioritizes low latency over reliability. Dropped frames are acceptable.

3. **C** - The message is sent immediately regardless of subscriber status
   - *Explanation:* Publishers send data as soon as `publish()` is called. Discovery happens asynchronously.

4. **B** - Action - because it supports feedback and preemption
   - *Explanation:* Actions are designed for long-running tasks that need progress updates and cancellation.

5. **A** - A joint that cannot move (rigid attachment)
   - *Explanation:* Fixed joints represent rigid connections, like a camera mounted to a robot frame.

6. **B** - `<origin xyz="0.2 0 0.3" rpy="0 0 0"/>`
   - *Explanation:* ROS convention: X=forward (0.2m), Y=left (0m), Z=up (0.3m).

7. **A** - Fixed joints don't require limits (this is a trick question)
   - *Explanation:* Fixed joints are static and don't need `<limit>` tags. Only revolute/prismatic joints do.

8. **A** - `I_zz = (m/2) * r²`
   - *Explanation:* For a cylinder rotating around its central axis, inertia simplifies to this formula.

9. **B** - 0.3m to 10m
   - *Explanation:* D435i min depth ≈ 0.3m (IR pattern can't focus closer), max useful range ≈ 10m indoors.

10. **D** - 1,875 points (8× fewer)
    - *Explanation:* Doubling voxel size (0.01→0.02m) reduces points by 2³=8×. 15,000/8 ≈ 1,875.

11. **C** - Rotating around the Z-axis (yaw)
    - *Explanation:* Angular velocity vector: [roll_rate, pitch_rate, yaw_rate]. Z-component is yaw.

12. **C** - IMUs maintain pose estimation during fast motion or poor lighting when cameras fail
    - *Explanation:* IMUs provide high-rate inertial measurements that complement vision-based odometry.

13. **C** - How far a point can be from the plane and still be considered an inlier
    - *Explanation:* `distance_threshold` defines the inlier/outlier boundary (e.g., 0.01m = 1cm tolerance).

14. **B** - No - temporal filter blurs motion and will smear moving objects
    - *Explanation:* Temporal filtering averages over time, which smooths static noise but blurs dynamics.

15. **B** - 0.55m (L1 + L2)
    - *Explanation:* Maximum reach = sum of link lengths when fully extended: 0.3m + 0.25m = 0.55m.

</details>

---

## Review Recommendations

### If you scored < 12/15, review these sections:

**Questions 1-4 (ROS 2 Fundamentals):**
- Section 1.1.1: ROS 2 Architecture & DDS
- Section 1.1.2: rclpy Patterns
- Section 1.1.4: QoS Profiles

**Questions 5-8 (URDF & Kinematics):**
- Section 1.2.1: URDF Basics
- Section 1.2.3: Validating Robot Kinematics
- Lab 2: URDF Humanoid Torso

**Questions 9-14 (Sensors & Perception):**
- Section 1.3.2: RealSense Integration
- Section 1.3.3: Sensor Fusion
- Lab 3: Point Cloud Processing

**Question 15 (Kinematics):**
- Section 1.2.3: Forward Kinematics
- Lab 2: Arm Workspace Analysis

---

## Next Steps

**Passed the quiz?** 🎉
- Continue to **[Module 2: Core Architecture](../module-02-architecture/overview.md)**
- Start working on advanced ROS 2 patterns
- Explore Isaac Sim simulation in Module 3

**Need more practice?**
- Redo Labs 1-3 with the extension challenges
- Review sections where you missed questions
- Join the [GitHub Discussions](https://github.com/shazilk-dev/physical-ai-and-robotics-book/discussions) for help

---

## Additional Resources

### Interactive Practice
- [ROS 2 Humble Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [URDF Visualizer](http://wiki.ros.org/urdf_tutorial)
- [RealSense GitHub Examples](https://github.com/IntelRealSense/realsense-ros/tree/ros2-development/realsense2_camera/launch)

### Video Supplements
- **ROS 2 Basics:** The Construct ROS 2 Course (YouTube)
- **URDF Deep Dive:** Articulated Robotics Channel
- **Point Cloud Processing:** Open3D Tutorial Series

---

**Questions about quiz answers?** Open an issue on [GitHub](https://github.com/shazilk-dev/physical-ai-and-robotics-book/issues) with the tag `quiz-module-1`.

**Ready for Module 2?** You'll learn about motors, actuators, and edge compute architectures that power real robots! 🤖⚡
