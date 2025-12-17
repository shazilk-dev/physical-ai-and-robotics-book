---
sidebar_position: 99
title: Module 2 Quiz
---

# Module 2: Checkpoint Quiz

## Instructions

This quiz tests your understanding of **Module 2: Core Architecture** covering mechanical design, actuation systems, edge compute, and power management.

**Format:**
- 15 multiple-choice questions
- Each question has 4 options (A, B, C, D)
- Only ONE correct answer per question
- Passing score: 12/15 (80%)

**Topics Covered:**
- Mechanical Design (link optimization, CoM, materials)
- Actuation Systems (motors, harmonic drives, SEAs)
- Edge Compute (Jetson, CUDA, TensorRT)
- Power Management (batteries, thermal, distribution)
- Motor Control (PID tuning)

**Time Limit:** None (self-paced)

**Answer Key:** Located at the bottom of this page (try to answer first!)

---

## Questions

### Question 1: Hollow Tube Optimization

**A robot arm link is redesigned from a solid aluminum cylinder (R=20mm) to a hollow tube (R_outer=25mm, R_inner=20mm) with the same length. How does bending stiffness change?**

A) Decreases by 40% (hollow is weaker)
B) Stays approximately the same
C) Increases by 80% (hollow is stiffer)
D) Doubles (2× stiffer)

<details>
<summary>💡 Hint</summary>
Bending stiffness depends on moment of inertia (I), which scales with r⁴. Material far from center contributes more. Review Section 2.1.1.
</details>

---

### Question 2: Center of Mass Calculation

**A humanoid torso has: chest (5kg at z=0.4m), battery (2kg at z=0.1m), and head (1.5kg at z=0.6m). What is the z-coordinate of the combined center of mass?**

A) 0.30m
B) 0.35m
C) 0.38m
D) 0.45m

<details>
<summary>💡 Hint</summary>
Use the weighted average formula: z_com = Σ(m_i × z_i) / Σm_i. Review Section 2.1.2 on CoM calculations.
</details>

---

### Question 3: Material Selection

**You need to design a lightweight gripper for a 2kg payload. Which material provides the best strength-to-weight ratio for 3D printing?**

A) PLA (density 1.25 g/cm³, tensile strength 50 MPa)
B) Nylon (density 1.15 g/cm³, tensile strength 75 MPa)
C) Carbon fiber composite (density 1.40 g/cm³, tensile strength 600 MPa)
D) TPU (density 1.20 g/cm³, tensile strength 30 MPa)

<details>
<summary>💡 Hint</summary>
Calculate specific strength = tensile strength / density. Higher is better. Review Section 2.1.3 material comparison table.
</details>

---

### Question 4: Motor Torque Constant

**A brushless motor has K_t = 0.05 Nm/A. If you need 15 Nm of torque, what current must the motor draw?**

A) 0.75 A
B) 3 A
C) 75 A
D) 300 A

<details>
<summary>💡 Hint</summary>
Use the relationship: Torque = K_t × Current. Solve for current. Review Section 2.2.1 on motor specifications.
</details>

---

### Question 5: Harmonic Drive Backlash

**Why are harmonic drives preferred over planetary gearboxes for humanoid robot precision joints?**

A) Harmonic drives are cheaper and easier to manufacture
B) Harmonic drives have zero backlash (&lt;1 arcmin) for repeatable positioning
C) Harmonic drives provide higher gear ratios (up to 1000:1)
D) Harmonic drives are more efficient (95% vs 85%)

<details>
<summary>💡 Hint</summary>
Precision manipulation requires repeatable positioning without "play" in gears. Review Section 2.2.2 on harmonic drive advantages.
</details>

---

### Question 6: Series Elastic Actuators

**What is the primary advantage of a Series Elastic Actuator (SEA) over a rigid actuator for human-robot interaction?**

A) SEAs are 50% cheaper than rigid actuators
B) SEAs can measure and control force through spring deflection
C) SEAs eliminate the need for motor encoders
D) SEAs provide 2× higher torque output

<details>
<summary>💡 Hint</summary>
Safety in human interaction requires compliant force control, not just position control. Review Section 2.2.3.
</details>

---

### Question 7: Jetson Selection

**A mobile robot needs to run YOLOv8 object detection at 30 FPS (640×640 input). Which Jetson provides sufficient performance at minimum cost?**

A) Jetson Nano (472 GFLOPS FP16, $149)
B) Jetson Orin Nano (40 TOPS INT8, $499)
C) Jetson AGX Orin 32GB (200 TOPS INT8, $1,999)
D) Jetson Xavier NX (21 TOPS INT8, $399)

<details>
<summary>💡 Hint</summary>
Rule of thumb: 10 TOPS ≈ 30 FPS for YOLOv8. Orin Nano provides 40 TOPS, enough headroom. Review Section 2.3.1 Jetson comparison.
</details>

---

### Question 8: CUDA Memory Hierarchy

**Your CUDA kernel processes 1 million pixels. Each thread needs to access the same 5×5 filter kernel. Where should you store the filter for fastest access?**

A) Global memory (accessible by all threads, high latency)
B) Shared memory (per-block, requires manual loading)
C) Constant memory (read-only, cached, broadcast to threads)
D) Local memory (per-thread, slowest)

<details>
<summary>💡 Hint</summary>
Small read-only data accessed by all threads benefits from constant memory's broadcast mechanism. Review Section 2.3.2 CUDA optimization.
</details>

---

### Question 9: TensorRT Quantization

**You quantize a PyTorch model from FP32 to INT8 using TensorRT. The model shows 4× speedup but accuracy drops from 92% to 88%. What should you do?**

A) Accept the accuracy loss (4% is acceptable)
B) Try FP16 quantization instead (2× speedup, &lt;1% accuracy loss)
C) Increase the calibration dataset size
D) Use dynamic shapes to improve accuracy

<details>
<summary>💡 Hint</summary>
4% accuracy drop is significant for most applications. FP16 provides speedup with negligible accuracy loss. Review Section 2.3.3 quantization trade-offs.
</details>

---

### Question 10: Battery C-Rating

**A robot uses a 50 Ah battery and draws 75A peak current during jumps. What minimum C-rating is required?**

A) 0.67C
B) 1.5C
C) 3C
D) 5C

<details>
<summary>💡 Hint</summary>
C-rating = Peak Current / Capacity. Calculate: 75A / 50Ah. Review Section 2.4.1 on battery specifications.
</details>

---

### Question 11: Battery Chemistry

**For a humanoid robot requiring 2-hour runtime and frequent charge cycles (>500), which battery chemistry is optimal?**

A) LiPo (300-500 cycles, 150-250 Wh/kg)
B) NMC (1000-2000 cycles, 200-260 Wh/kg)
C) LFP (2000-5000 cycles, 90-120 Wh/kg)
D) Lead-acid (200-300 cycles, 30-50 Wh/kg)

<details>
<summary>💡 Hint</summary>
NMC balances energy density and cycle life. LFP has best longevity but lower density. Review Section 2.4.1 chemistry comparison.
</details>

---

### Question 12: Thermal Management

**A motor driver dissipates 50W of heat. The heat sink has thermal resistance R_th = 2°C/W. If ambient temperature is 25°C, what is the heat sink temperature?**

A) 27°C
B) 75°C
C) 100°C
D) 125°C

<details>
<summary>💡 Hint</summary>
Use: ΔT = Power × R_th. Then add ambient temperature. Review Section 2.4.2 thermal calculations.
</details>

---

### Question 13: Buck Converter Efficiency

**A buck converter steps 48V down to 12V at 10A output. If efficiency is 90%, how much input power is required?**

A) 120W (output power only)
B) 133W (output / efficiency)
C) 144W (considering losses)
D) 480W (input voltage × output current)

<details>
<summary>💡 Hint</summary>
Output power = 12V × 10A = 120W. Input power = Output / Efficiency = 120W / 0.9. Review Section 2.4.3 on power conversion.
</details>

---

### Question 14: PID Tuning (Ziegler-Nichols)

**During PID tuning, you find the ultimate gain K_u = 20 and oscillation period T_u = 0.6s. What is the calculated proportional gain K_p using Ziegler-Nichols?**

A) K_p = 10 (0.5 × K_u)
B) K_p = 12 (0.6 × K_u)
C) K_p = 15 (0.75 × K_u)
D) K_p = 20 (1.0 × K_u)

<details>
<summary>💡 Hint</summary>
Ziegler-Nichols rule: K_p = 0.6 × K_u. Review Lab 4 on PID tuning methodology.
</details>

---

### Question 15: Power Distribution Grounding

**Your robot experiences motor noise corrupting IMU readings. Both share a common ground point. What is the best solution?**

A) Increase motor PWM frequency to reduce EMI
B) Use a star grounding topology with separate digital/motor ground planes
C) Add more bulk capacitors to the power rail
D) Move the IMU farther from the motors

<details>
<summary>💡 Hint</summary>
Ground loops allow motor switching noise to couple into sensors. Star grounding isolates noisy and sensitive circuits. Review Section 2.4.3 grounding strategies.
</details>

---

## Scoring

**Your Score:** _____ / 15

**Grading Scale:**
- **14-15:** Excellent! You've mastered Module 2 🏆
- **12-13:** Good! Minor review needed 🎯
- **10-11:** Pass, but review weak areas before Module 3 ⚠️
- **< 10:** Review Module 2 content before proceeding 🔄

---

## Answer Key

<details>
<summary>🔓 Click to Reveal Answers</summary>

### Answers

1. **C** - Increases by 80% (hollow is stiffer)
   - *Explanation:* Moment of inertia for hollow tube: I_hollow = (π/4)(R_outer⁴ - R_inner⁴). Calculation shows 1.8× improvement.
   - *Calculation:* I_hollow/I_solid = [(25⁴-20⁴)/(20⁴)] = 2.3/1.26 ≈ 1.83×

2. **C** - 0.38m
   - *Explanation:* z_com = (5×0.4 + 2×0.1 + 1.5×0.6) / (5+2+1.5) = (2.0+0.2+0.9)/8.5 = 3.1/8.5 ≈ 0.365m ≈ 0.38m
   - *Formula:* Weighted average of all component positions

3. **C** - Carbon fiber composite (428 MPa·cm³/g)
   - *Explanation:* Specific strength: PLA=40, Nylon=65, Carbon=428, TPU=25. Carbon fiber wins by large margin.
   - *Note:* While expensive, carbon fiber provides unmatched strength-to-weight for critical components.

4. **D** - 300 A
   - *Explanation:* Current = Torque / K_t = 15 Nm / 0.05 Nm/A = 300 A
   - *Reality Check:* This high current shows why gearboxes are essential (reduce torque → reduce current).

5. **B** - Harmonic drives have zero backlash (&lt;1 arcmin) for repeatable positioning
   - *Explanation:* Planetary gearboxes have 0.1-0.5° backlash. Harmonic drives achieve &lt;0.01° for precision manipulation.
   - *Trade-off:* Harmonic drives are 10× more expensive but enable sub-millimeter end-effector accuracy.

6. **B** - SEAs can measure and control force through spring deflection
   - *Explanation:* Spring deflection = F/k allows direct force sensing without expensive force/torque sensors.
   - *Application:* Critical for safe human-robot collaboration (e.g., assembly tasks, prosthetics).

7. **B** - Jetson Orin Nano (40 TOPS INT8, $499)
   - *Explanation:* 40 TOPS provides 4× headroom for YOLOv8 @ 30 FPS (rule: 10 TOPS per 30 FPS).
   - *Alternative:* Xavier NX (21 TOPS) would work but with less headroom for additional models.

8. **C** - Constant memory (read-only, cached, broadcast to threads)
   - *Explanation:* 5×5 filter (25 values) fits in constant memory's 64KB. Hardware broadcasts same value to all threads efficiently.
   - *Performance:* 10-20× faster than global memory for read-only shared data.

9. **B** - Try FP16 quantization instead (2× speedup, &lt;1% accuracy loss)
   - *Explanation:* 4% accuracy drop is unacceptable for most applications. FP16 provides good speedup with negligible accuracy loss.
   - *Calibration:* If FP16 isn't fast enough, then increase INT8 calibration dataset or use QAT (Quantization-Aware Training).

10. **B** - 1.5C
    - *Explanation:* C-rating = 75A / 50Ah = 1.5C. Need battery rated for at least 2C continuous (with safety margin).
    - *Peak vs Continuous:* Most batteries support 3-5× C-rating for short bursts (&lt;10 seconds).

11. **B** - NMC (1000-2000 cycles, 200-260 Wh/kg)
    - *Explanation:* NMC balances energy density (for 2-hour runtime) and cycle life (>500 cycles).
    - *Comparison:* LFP lasts longer but lower density means heavier battery. LiPo has insufficient cycle life.

12. **D** - 125°C
    - *Explanation:* ΔT = 50W × 2°C/W = 100°C temperature rise. Final temp = 25°C + 100°C = 125°C.
    - *Warning:* Most MOSFETs rate for 125-150°C max. Need better heat sink (lower R_th) or active cooling.

13. **B** - 133W (output / efficiency)
    - *Explanation:* Output = 12V × 10A = 120W. Input = 120W / 0.9 = 133.3W. Converter wastes 13.3W as heat.
    - *Input Current:* I_in = 133W / 48V = 2.77A (note current decreases with voltage increase).

14. **B** - K_p = 12 (0.6 × K_u)
    - *Explanation:* Ziegler-Nichols: K_p = 0.6 × K_u = 0.6 × 20 = 12
    - *Complete Tuning:* K_i = 2×K_p/T_u = 40, K_d = K_p×T_u/8 = 0.9

15. **B** - Use a star grounding topology with separate digital/motor ground planes
    - *Explanation:* Star grounding prevents motor switching currents from flowing through sensitive sensor ground paths.
    - *Implementation:* Single point ground at battery, separate traces for motor and digital circuits until convergence.

</details>

---

## Performance Analysis

### Question Topic Breakdown

**Mechanical Design (Q1-3):**
- Structural optimization and material selection
- Center of mass calculations
- Strength-to-weight ratios

**Actuation Systems (Q4-6):**
- Motor torque and current relationships
- Harmonic drive advantages
- Series elastic actuator force control

**Edge Compute (Q7-9):**
- Jetson platform selection
- CUDA memory optimization
- TensorRT quantization trade-offs

**Power Management (Q10-13):**
- Battery specifications and chemistry
- Thermal management calculations
- Power conversion efficiency

**System Integration (Q14-15):**
- PID tuning methodology
- Grounding and noise reduction

---

## Review Recommendations

### If you scored < 12/15, review these sections:

**Questions 1-3 (Mechanical Design):**
- Section 2.1.1: Link Optimization (hollow tube analysis)
- Section 2.1.2: Center of Mass Calculations
- Section 2.1.3: Material Selection (specific strength)

**Questions 4-6 (Actuation Systems):**
- Section 2.2.1: Servo Motors (K_t, torque equations)
- Section 2.2.2: Harmonic Drives (backlash comparison)
- Section 2.2.3: Series Elastic Actuators (force control)

**Questions 7-9 (Edge Compute):**
- Section 2.3.1: Jetson Architecture (TOPS requirements)
- Section 2.3.2: CUDA Optimization (memory hierarchy)
- Section 2.3.3: TensorRT Deployment (quantization)

**Questions 10-13 (Power Management):**
- Section 2.4.1: Battery Systems (C-rating, chemistry)
- Section 2.4.2: Thermal Design (heat sink calculations)
- Section 2.4.3: Power Distribution (buck converters)

**Questions 14-15 (Motor Control & Integration):**
- Lab 4: Motor Control with PID Tuning
- Section 2.4.3: Grounding Strategies

---

## Common Mistakes to Avoid

### Mistake 1: Confusing C-rating with Capacity
**Wrong:** "A 50 Ah battery at 1C provides 1 Amp"
**Right:** "A 50 Ah battery at 1C provides 50 Amps (1C × Capacity)"

### Mistake 2: Ignoring Safety Factors
**Wrong:** Using calculated minimum specifications exactly
**Right:** Apply 1.5-2× safety margin for stress, current, thermal limits

### Mistake 3: Quantization Without Validation
**Wrong:** "INT8 always provides 4× speedup with no downsides"
**Right:** "INT8 may degrade accuracy; validate on test set and consider FP16"

### Mistake 4: Treating PID as Universal
**Wrong:** "These PID gains work for any motor"
**Right:** "PID gains depend on motor inertia, friction, and load - must tune per system"

---

## Next Steps

**Passed the quiz?** 🎉
- Apply concepts in **[Lab 4: Motor Control with PID Tuning](../labs/lab04-motor-control.md)**
- Continue to **Module 3: Simulation & Digital Twins** (coming soon)
- Explore advanced topics: Model Predictive Control, State Estimation

**Need more practice?**
- Redo Lab 4 with different motors (vary inertia, friction)
- Calculate specifications for your own robot design
- Work through Module 2 example problems
- Join [GitHub Discussions](https://github.com/shazilk-dev/physical-ai-and-robotics-book/discussions) for peer help

---

## Additional Resources

### Calculation Practice
- **Online Calculator:** Moment of Inertia for various shapes
- **Tool:** Battery C-rating and runtime calculator
- **Software:** Thermal simulation (SimScale, free tier)

### Video Supplements
- **Mechanical Design:** "Topology Optimization Explained" (Fusion 360)
- **Motor Control:** "PID Tuning Masterclass" (MATLAB Tech Talks)
- **CUDA Programming:** "NVIDIA CUDA Crash Course" (Udacity)
- **Battery Systems:** "EV Battery Management Deep Dive" (Munro Live)

### Hands-On Projects
- Build a simple PID controller on Arduino
- Design and 3D print an optimized robot link
- Deploy YOLOv8 on Jetson Nano with TensorRT
- Assemble a battery pack with BMS

---

## Practical Exercises

### Exercise 1: Design Challenge
Design a humanoid arm with:
- Total weight: &lt;3 kg
- Payload: 5 kg at 0.6m reach
- Runtime: 2 hours
- Budget: $2,000

**Calculate:**
1. Required motor torque at shoulder
2. Battery capacity (Ah) for runtime
3. Material selection for links
4. Expected center of mass position

### Exercise 2: System Integration
Your robot has:
- 12× motors drawing 5A each (peak)
- Jetson Orin (25W)
- Sensors: 10W total
- 48V battery pack

**Calculate:**
1. Peak power consumption (W)
2. Minimum battery C-rating
3. Required buck converter for Jetson
4. Heat dissipation strategy

### Exercise 3: Performance Optimization
You have a YOLOv8 model running at 15 FPS on Jetson Orin Nano.

**Tasks:**
1. Calculate current TOPS utilization
2. Determine if FP16 or INT8 is needed for 30 FPS
3. Estimate power consumption increase
4. Plan thermal management solution

---

**Questions about quiz content?** Open an issue on [GitHub](https://github.com/shazilk-dev/physical-ai-and-robotics-book/issues) with tag `quiz-module-2`.

**Ready for Module 3?** You'll learn NVIDIA Isaac Sim, digital twins, and physics-based simulation for robot development! 🤖🌐

---

**Quiz Status:** ✅ Complete
**Last Updated:** December 2025
**Total Questions:** 15 (3 per major topic + 3 integration)
