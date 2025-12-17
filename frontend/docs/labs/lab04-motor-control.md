---
sidebar_position: 5
title: Lab 4 - Motor Control with PID
---

# Lab 4: Motor Control with PID Tuning - Position Control System

## Overview

**Difficulty:** 🟡 Intermediate
**Estimated Time:** 2-3 hours
**Module:** 2.2 Actuation Systems

In this lab, you'll build a complete **PID (Proportional-Integral-Derivative) position controller** for a motor - the same control strategy used in humanoid robot joints. You'll implement the controller in ROS 2, tune it using the Ziegler-Nichols method, and visualize performance with PlotJuggler.

**Real-World Context:** This is exactly how **Tesla Optimus**, **Unitree G1**, and **Boston Dynamics Atlas** control their joint positions. You're building production-grade motor control!

---

## Learning Objectives

By completing this lab, you will:

1. ✅ Implement a PID controller in ROS 2 (Python)
2. ✅ Read encoder feedback at 1 kHz for real-time control
3. ✅ Tune PID gains using Ziegler-Nichols method
4. ✅ Analyze step response (rise time, overshoot, steady-state error)
5. ✅ Visualize control performance with PlotJuggler
6. ✅ Understand the relationship between gains and system behavior

---

## Prerequisites

### Required Knowledge

- ✅ **Section 2.2.1:** Servo Motors & Brushless DC Motors
- ✅ **Module 1:** ROS 2 fundamentals (nodes, topics, parameters)
- ✅ **Math:** Basic calculus (derivatives, integrals)
- ✅ **Physics:** Torque, angular velocity, inertia

### Required Software

- ROS 2 Humble (Ubuntu 22.04) or Iron
- Python 3.10+
- PlotJuggler: `sudo apt install ros-humble-plotjuggler-ros`
- Gazebo (for simulation): `sudo apt install ros-humble-gazebo-ros-pkgs`

### Hardware Options

**Option A: Simulation (Recommended for Learning)**
- No hardware needed!
- Gazebo simulated motor with realistic dynamics
- Virtual encoder feedback

**Option B: Physical Hardware**
- Arduino Uno/Mega ($25)
- Brushless DC motor with encoder ($40)
- Motor driver board (L298N or ODrive, $8-$120)
- 12V power supply ($15)
- **Total: ~$100**

**This lab supports both options!** Start with simulation, then deploy to hardware if available.

---

## Lab Structure

Complete lab materials located at:

```
/labs/lab04-motor-control/
├── README.md                           # Detailed instructions
├── starter/                            # Start here - contains TODOs
│   ├── package.xml
│   ├── setup.py
│   ├── config/
│   │   └── pid_params.yaml            # PID tuning parameters
│   └── motor_control_package/
│       ├── __init__.py
│       ├── pid_controller.py          # Main controller (7 TODOs)
│       ├── motor_simulator.py         # Gazebo interface
│       └── encoder_reader.py          # Encoder feedback node
├── solutions/                          # Reference implementation
│   ├── package.xml
│   ├── setup.py
│   ├── config/
│   │   └── pid_params_tuned.yaml     # Pre-tuned gains
│   └── motor_control_package/
│       ├── __init__.py
│       ├── pid_controller.py
│       ├── motor_simulator.py
│       └── encoder_reader.py
├── tests/
│   ├── test_pid_controller.py         # Unit tests
│   └── test_step_response.py          # Performance validation
├── assets/
│   ├── step_response_plot.png         # Expected performance
│   ├── tuning_guide.pdf               # Ziegler-Nichols walkthrough
│   └── gazebo_motor.urdf              # Motor simulation model
└── hardware/
    ├── arduino_encoder_firmware.ino   # Arduino code (optional)
    └── wiring_diagram.png             # Hardware setup
```

---

## Quick Start

### Step 1: Setup ROS 2 Workspace

```bash
# Navigate to your workspace
cd ~/ros2_ws/src

# Copy starter code
cp -r /path/to/physical-ai-robotics-book/labs/lab04-motor-control/starter motor_control_package

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select motor_control_package
source install/setup.bash
```

### Step 2: Launch Gazebo Simulation

```bash
# Terminal 1: Launch simulated motor
ros2 launch motor_control_package motor_sim.launch.py

# You should see Gazebo open with a single rotating joint
```

**What's happening:**
- Gazebo simulates a motor with realistic inertia (J = 0.01 kg·m²)
- Virtual encoder publishes position at 1 kHz to `/motor/encoder`
- Motor accepts velocity commands on `/motor/cmd_vel`

### Step 3: Implement PID Controller (TODOs)

Open `motor_control_package/pid_controller.py` and complete **7 TODOs**:

#### TODO 1: Initialize PID Gains

```python
def __init__(self):
    super().__init__('pid_controller')

    # TODO 1: Declare PID parameters
    self.declare_parameter('kp', 0.0)  # Proportional gain
    self.declare_parameter('ki', 0.0)  # Integral gain
    self.declare_parameter('kd', 0.0)  # Derivative gain

    # Load parameters
    self.kp = self.get_parameter('kp').value
    self.ki = self.get_parameter('ki').value
    self.kd = self.get_parameter('kd').value
```

#### TODO 2: Create Subscribers and Publishers

```python
# TODO 2: Subscribe to encoder feedback
self.encoder_sub = self.create_subscription(
    Float64,
    '/motor/encoder',
    self.encoder_callback,
    10
)

# TODO 2: Subscribe to setpoint (desired position)
self.setpoint_sub = self.create_subscription(
    Float64,
    '/motor/setpoint',
    self.setpoint_callback,
    10
)

# TODO 2: Publish motor velocity commands
self.cmd_pub = self.create_publisher(
    Float64,
    '/motor/cmd_vel',
    10
)
```

#### TODO 3: Implement PID Calculation

```python
def compute_control(self, current_position, setpoint, dt):
    """
    TODO 3: Implement PID control law

    u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt

    Args:
        current_position: Current motor angle (radians)
        setpoint: Desired motor angle (radians)
        dt: Time step (seconds)

    Returns:
        control_output: Motor velocity command (rad/s)
    """
    # Calculate error
    error = setpoint - current_position

    # Proportional term
    p_term = self.kp * error

    # Integral term (accumulate error over time)
    self.integral += error * dt
    i_term = self.ki * self.integral

    # Derivative term (rate of change of error)
    derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
    d_term = self.kd * derivative

    # PID output
    control_output = p_term + i_term + d_term

    # Store for next iteration
    self.previous_error = error

    return control_output
```

#### TODO 4-7: See starter code for remaining implementations

**Remaining tasks:**
- TODO 4: Implement encoder callback to update current position
- TODO 5: Implement setpoint callback to update desired position
- TODO 6: Create 1 kHz control loop timer
- TODO 7: Add anti-windup for integral term (prevent integral saturation)

---

## Step 4: Initial Testing (Before Tuning)

Start with zero gains to verify your implementation:

```bash
# Edit config/pid_params.yaml
kp: 0.0
ki: 0.0
kd: 0.0

# Run controller
ros2 run motor_control_package pid_controller --ros-args --params-file config/pid_params.yaml
```

**Verify:**
```bash
# Terminal 2: Check topics are active
ros2 topic list
# Should show: /motor/encoder, /motor/cmd_vel, /motor/setpoint

# Terminal 3: Publish step input (move to 1.57 radians = 90°)
ros2 topic pub /motor/setpoint std_msgs/Float64 "data: 1.57" --once

# Motor should NOT move (gains are zero)
```

---

## Step 5: PID Tuning with Ziegler-Nichols Method

### Phase 1: Find Ultimate Gain (K_u)

**Goal:** Find the proportional gain where the system oscillates continuously.

```bash
# Edit config/pid_params.yaml
kp: 1.0   # Start here
ki: 0.0   # Keep at zero
kd: 0.0   # Keep at zero

# Restart controller and apply step input
ros2 topic pub /motor/setpoint std_msgs/Float64 "data: 1.57" --once
```

**Increase `kp` until you see sustained oscillations:**

| kp | Behavior |
|----|----------|
| 1.0 | Slow response, no overshoot |
| 5.0 | Faster, slight overshoot |
| 10.0 | Large overshoot, damps out |
| **15.0** | **Continuous oscillation (K_u found!)** |
| 20.0 | Unstable, diverges |

**Record:**
- Ultimate Gain: `K_u = 15.0`
- Oscillation Period: `T_u = 0.5 seconds` (measure from PlotJuggler)

### Phase 2: Calculate Ziegler-Nichols Gains

Use the **classic Ziegler-Nichols tuning rules:**

```
Kp = 0.6 * K_u  = 0.6 * 15.0 = 9.0
Ki = 2 * Kp / T_u = 2 * 9.0 / 0.5 = 36.0
Kd = Kp * T_u / 8 = 9.0 * 0.5 / 8 = 0.56
```

### Phase 3: Apply Tuned Gains

```yaml
# config/pid_params.yaml
kp: 9.0
ki: 36.0
kd: 0.56
```

Restart controller and test:

```bash
ros2 run motor_control_package pid_controller --ros-args --params-file config/pid_params.yaml

# Apply step input
ros2 topic pub /motor/setpoint std_msgs/Float64 "data: 1.57" --once
```

**Expected Performance:**
- Rise time: &lt;0.2 seconds
- Overshoot: &lt;10%
- Settling time: &lt;0.5 seconds
- Steady-state error: &lt;0.01 radians

---

## Step 6: Visualize with PlotJuggler

```bash
# Terminal 4: Launch PlotJuggler
ros2 run plotjuggler plotjuggler

# In PlotJuggler:
# 1. Click "Streaming" -> "Start: ROS2 Topic Subscriber"
# 2. Select topics:
#    - /motor/encoder (actual position)
#    - /motor/setpoint (desired position)
#    - /motor/error (if you published it)
# 3. Drag topics to plot area
# 4. Apply step input and observe response
```

**What to look for:**

```
Setpoint (red line):    ________  1.57 rad
                       |
Actual (blue line):   /‾‾\__    (should closely follow)
                     /      ‾
                    /
```

**Quality Metrics:**
- **Rise Time:** Time to reach 90% of setpoint (should be &lt;0.2s)
- **Overshoot:** Peak above setpoint (should be &lt;10%)
- **Settling Time:** Time to stay within ±2% of setpoint (should be &lt;0.5s)
- **Steady-State Error:** Final error at steady state (should be &lt;0.01 rad)

---

## Step 7: Run Automated Tests

```bash
# Unit tests
cd ~/ros2_ws
colcon test --packages-select motor_control_package

# Check results
colcon test-result --verbose
```

**Tests verify:**
- ✅ PID calculation is correct (unit test)
- ✅ Step response meets performance criteria
- ✅ Controller runs at 1 kHz (timing test)
- ✅ Anti-windup prevents integral saturation

---

## Advanced Challenges (Optional)

### Challenge 1: Trajectory Tracking

Modify controller to track a sinusoidal trajectory:

```python
# In setpoint callback, generate sine wave
import math
t = self.get_clock().now().seconds_nanoseconds()[0]
setpoint = 1.0 * math.sin(2 * math.pi * 0.5 * t)  # 0.5 Hz sine wave
```

**Goal:** Minimize tracking error (RMS error &lt;0.05 rad)

### Challenge 2: Hardware Deployment

If you have Arduino + motor:

1. Flash `hardware/arduino_encoder_firmware.ino` to Arduino
2. Wire encoder to pins 2, 3 (interrupts)
3. Wire motor driver to PWM pin 9
4. Update `encoder_reader.py` to read from `/dev/ttyACM0`
5. Run same controller on real hardware!

**Warning:** Start with low gains on hardware to avoid oscillations damaging gears.

### Challenge 3: Adaptive PID

Implement gain scheduling - adjust PID gains based on operating region:

```python
# High gains for fast response when far from target
if abs(error) > 0.5:
    self.kp *= 1.5
    self.kd *= 1.2
# Lower gains near target for stability
else:
    self.kp *= 0.8
```

---

## Common Issues & Troubleshooting

### Issue 1: Motor Oscillates Wildly

**Cause:** `kp` or `kd` too high
**Fix:** Reduce both by 50%, re-tune incrementally

### Issue 2: Slow Response, Large Steady-State Error

**Cause:** `kp` too low, `ki` too low
**Fix:** Increase `kp` until slight overshoot, then increase `ki`

### Issue 3: Integral Windup (Motor Saturates)

**Symptom:** Motor "stuck" at max velocity, takes long to recover
**Fix:** Implement anti-windup (TODO 7):

```python
# Clamp integral term
MAX_INTEGRAL = 100.0
self.integral = max(-MAX_INTEGRAL, min(MAX_INTEGRAL, self.integral))
```

### Issue 4: Gazebo Motor Not Moving

**Check:**
```bash
# Verify Gazebo is publishing encoder data
ros2 topic echo /motor/encoder

# Verify controller is publishing commands
ros2 topic echo /motor/cmd_vel

# Check for ROS 2 parameter issues
ros2 param list /pid_controller
```

---

## Deliverables

Submit the following (if this is a course):

1. **Tuned PID Parameters** (`pid_params.yaml`)
2. **Step Response Plot** (screenshot from PlotJuggler showing all metrics)
3. **Performance Analysis** (1 paragraph):
   - Your K_u and T_u values
   - Calculated gains
   - Measured rise time, overshoot, settling time
   - Comparison to expected performance

**Example report:**
```
K_u = 15.2, T_u = 0.48s
Calculated: Kp=9.12, Ki=38.0, Kd=0.55
Measured Performance:
- Rise time: 0.18s ✅
- Overshoot: 8.2% ✅
- Settling time: 0.42s ✅
- Steady-state error: 0.008 rad ✅

All metrics meet specifications. Controller is production-ready!
```

---

## Real-World Application

**This lab teaches you the exact same PID control used in:**

**Tesla Optimus:**
- 28 joints, each with PID position control
- Control frequency: 1 kHz
- Custom actuators with integrated encoders

**Unitree G1:**
- 23 DOF, each with tuned PID loops
- Uses adaptive gains (switch between position/force control)
- Gains stored in motor EEPROM for fast boot

**Boston Dynamics Atlas:**
- Hydraulic actuators with model-based PID
- Adds feedforward torque compensation
- Control loop: 3 kHz (300× faster than human reaction time)

**You're now qualified to work on humanoid robot control systems!** 🦾

---

## Next Steps

**After completing this lab:**

1. ✅ Continue to **Chapter 2.3: Edge Compute** (deploy AI on Jetson)
2. ✅ Read **Section 2.2.3: Series Elastic Actuators** (force control)
3. ✅ Explore advanced control: Model Predictive Control (MPC)

**Related Resources:**
- Paper: "PID Control for Humanoid Robots" (IEEE)
- Video: "Boston Dynamics: How Atlas Maintains Balance" (YouTube)
- Tool: MATLAB PID Tuner (visual gain tuning)

---

**Lab Status:** ✅ Complete
**Estimated Completion Time:** 2-3 hours
**Difficulty Rating:** 🟡 Intermediate (requires calculus and control theory basics)

**Questions?** Check the [GitHub Discussions](https://github.com/shazilk-dev/physical-ai-and-robotics-book/discussions) or post on ROS Answers.

Happy tuning! 🎛️🤖
