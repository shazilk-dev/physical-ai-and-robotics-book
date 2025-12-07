# **Physical AI & Humanoid Robotics**

## **Course Book: Bridging the Digital-Physical Divide (2025 Edition)**

Focus and Theme: AI Systems in the Physical World. Embodied Intelligence.  
Goal: Bridging the gap between the digital brain and the physical body. Students apply their AI knowledge to control Humanoid Robots in simulated and real-world environments.

### ---

**Quarter Overview: The Convergence of 2025**

The future of AI extends beyond digital spaces into the physical world. As of December 2025, the robotics industry has traversed a definitive threshold, evolving from experimental prototypes into a consolidated sector defined by **Physical AI**. This capstone quarter introduces the convergence of generative artificial intelligence with industrial-grade electromechanical actuation. Students will learn to design, simulate, and deploy humanoid robots capable of natural human interactions using the industry-standard stack: ROS 2, NVIDIA Isaac Lab, and VLA (Vision-Language-Action) models.

### ---

**Module 1: The Robotic Nervous System (ROS 2 & Middleware)**

**Focus:** Middleware for robot control and the "System 1" reactive layer.

In 2025, the "nervous system" of a humanoid robot relies on high-frequency, low-latency communication. While AI provides the high-level reasoning, the immediate execution of movement requires robust middleware.

- **ROS 2 in 2025:** The Robot Operating System 2 (ROS 2\) remains the industry standard for hardware abstraction. Leading platforms like the **Unitree G1** and **Agibot G2** rely on ROS 2 for secondary development, allowing developers to interface with high-torque actuators and sensors.1
- **The "System 1" Layer:** This module introduces the concept of "System 1" control—fast, reflex-like policies running at 200Hz+. Students will learn how ROS 2 nodes handle critical, high-speed tasks like balance recovery and slip detection, which must occur faster than a large AI model can "think."3
- **Unified Robot Description Format (URDF):** Students will dissect the URDFs of modern humanoids. For example, the **Unitree G1** features a configurable 23-43 degrees of freedom (DoF), requiring complex kinematic chains defined in URDF to manage its 120 N·m knee torque actuators.1

**Key Technical Skill:** Bridging Python Agents to ROS controllers using rclpy to control high-DOF limbs.

### ---

**Module 2: The Digital Twin (Sim-to-Real Pipelines)**

**Focus:** Physics simulation, synthetic data generation, and the "Sim-to-Real" gap.

Physical testing is prohibitively slow and expensive. The 2025 workflow relies on "Digital Twins"—high-fidelity simulations where robots experience millions of training hours in parallel.

- **Advanced Physics Engines:** We move beyond basic rigid body dynamics. This module utilizes the **NVIDIA Newton Physics Engine** (released late 2025), which provides the accurate contact dynamics required for humanoid hands to manipulate complex objects.5
- **The "Robot School" Methodology:** Students will replicate the data generation strategies used by companies like **Agibot**, where human teleoperation data is captured and then amplified in simulation. This "Sim-to-Real" pipeline allows robots to master corner cases (e.g., slipping on oil) without risking hardware.6
- **Simulating Sensors:** Implementation of high-fidelity sensor simulation, specifically modeling the **Livox Mid-360 LiDAR** and **Intel RealSense D435i** depth cameras found on the Unitree G1 and other standard platforms.1

**Key Technical Skill:** Training a reinforcement learning (RL) policy in a Digital Twin that can successfully transfer to a physical robot (Sim-to-Real).

### ---

**Module 3: The AI-Robot Brain (NVIDIA Isaac™ & Edge Compute)**

**Focus:** Advanced perception, embodied compute, and the "System 2" deliberative layer.

This module focuses on the "Brain" of the robot, often powered by NVIDIA's Jetson Orin or Thor platforms.

- **NVIDIA Isaac Lab & GR00T:** Students will work with **NVIDIA Isaac Lab**, integrating the **GR00T N1.6** foundation model. This "generalist robot 00 technology" allows robots to understand multimodal instructions and perform complex tasks like "opening a heavy door" while maintaining balance.5
- **Edge Intelligence:** We explore the constraints of onboard compute. The **Unitree G1** carries an **NVIDIA Jetson Orin NX** (100-157 TOPS), while the **Agibot G2** utilizes the more powerful **Jetson Thor T5000** (2070 TFLOPS).1 Students will learn to optimize perception models to run locally on these edge devices, ensuring robots can operate even without Wi-Fi.
- **VSLAM and Navigation:** Using **Isaac ROS**, students will implement Visual SLAM (Simultaneous Localization and Mapping) to allow the robot to navigate unstructured environments (cluttered rooms) rather than just static maps.1

**Key Technical Skill:** Deploying a containerized perception model to an NVIDIA Jetson edge device to control a robot's head/vision system.

### ---

**Module 4: Vision-Language-Action (VLA) & Agentic AI**

**Focus:** The convergence of LLMs and Robotics. "Large Behavior Models."

The defining breakthrough of 2025 is the **Vision-Language-Action (VLA)** model, which grounds language directly into physical motion.

- **VLA Architectures:** We will study **OpenVLA** (7B parameters) and **Figure AI's Helix**. Unlike traditional stacks, these models take visual data and text ("pick up the red apple") and output discretized motor actions directly, bypassing traditional code-generation steps.9
- **System 2 Reasoning:** Students will implement "System 2" thinking—a slower (7-9Hz) deliberative process that plans long-horizon tasks (e.g., "clean the kitchen"). This high-level plan is then broken down into primitives executed by the System 1 layer.10
- **Capstone Project: The Autonomous Humanoid.**
  - **Goal:** A simulated humanoid receives a vague voice command (e.g., "I spilled my drink"), plans a path, identifies the mess using VLM (Vision Language Model), grasps a tool, and cleans it.
  - **Tech Stack:** OpenAI Whisper (Voice) \-\> VLA Model (Reasoning/Action) \-\> ROS 2 (Execution).10

### ---

**Hardware Requirements: Building the Lab**

To teach "Physical AI" successfully in late 2025, the lab infrastructure must mirror the industry standard.

#### **1\. The "Digital Twin" Workstation (Required per Student)**

Simulation in 2025 is heavy. NVIDIA Isaac Sim and VLA inference require substantial VRAM.

- **GPU:** NVIDIA RTX 4080 Super or RTX 4090 (24GB VRAM recommended).
  - _Why:_ To load the USD (Universal Scene Description) assets and run 7B+ parameter VLA models (like OpenVLA) simultaneously.9
- **OS:** Ubuntu 22.04 LTS (Industry standard for ROS 2 Humble/Iron).

#### **2\. The "Physical AI" Edge Kit**

For students without a full robot, this "brain on a desk" setup replicates the compute of a humanoid head.

- **The Brain:** **NVIDIA Jetson Orin Nano Super Dev Kit** (8GB) or **Orin NX** (16GB).
  - _Note:_ The Orin NX is the exact module found inside the Unitree G1.1
- **The Eyes:** **Intel RealSense D435i** or **D455**.
  - _Why:_ Standard depth sensor for VSLAM and manipulation tasks.1

#### **3\. The Robot Lab (Options for 2025\)**

- **Option A: The "Proxy" Approach (Budget)**
  - **Robot:** **Unitree Go2 Edu** (\~$1,800 \- $3,000).
  - _Status:_ A quadruped, but it shares the same SDK and ROS 2 architecture as humanoids. Excellent for learning locomotion policies.11
- **Option B: The "Miniature Humanoid" Approach**
  - **Robot:** **Unitree G1 Humanoid** (\~$16,000).
  - _Status:_ The breakthrough product of 2025\. It is a full-sized (1.3m), highly dynamic humanoid priced for education. It features high-torque motors (120 N·m) and runs on the Orin NX, making it the perfect target for the Capstone Project.1
- **Option C: The "Cloud" Lab (Sim-Only)**
  - **Robot:** Digital Twin of **Figure 02** or **Tesla Optimus**.
  - _Status:_ Students use NVIDIA Omniverse Cloud to stream high-fidelity simulations of top-tier industrial robots they cannot afford physically.3

### ---

**Industry Context: Why This Course Matters Now (2025 Outlook)**

- **Market Explosion:** The humanoid market is valued at $2.92 billion in 2025, with companies like **Figure AI** valued at $39 billion. Students taking this course are entering a sector with aggressive hiring needs.5
- **Commercial Reality:** Humanoids are no longer just research projects. **Figure 02** robots are deployed at BMW factories, and **Agility Robotics' Digit** has moved over 100,000 totes at GXO logistics centers.10
- **The Skill Gap:** The industry is desperate for engineers who understand the intersection of **Generative AI** (VLAs) and **Control Theory** (ROS 2/Actuation). This course targets exactly that gap.

#### ---

**Works cited**

1. Unitree G1 Comp: 45-DOF Humanoid Robot with NVIDIA Jetson & High-Torque Actuators, accessed December 7, 2025, [https://futurology.tech/products/unitree-g1-comp-humanoid-robot](https://futurology.tech/products/unitree-g1-comp-humanoid-robot)
2. AgibotTech/agibot_x1_infer: The inference module for AgiBot X1. \- GitHub, accessed December 7, 2025, [https://github.com/AgibotTech/agibot_x1_infer](https://github.com/AgibotTech/agibot_x1_infer)
3. NVIDIA Announces Isaac GR00T N1 — the World's First Open Humanoid Robot Foundation Model — and Simulation Frameworks to Speed Robot Development, accessed December 7, 2025, [https://nvidianews.nvidia.com/news/nvidia-isaac-gr00t-n1-open-humanoid-robot-foundation-model-simulation-frameworks](https://nvidianews.nvidia.com/news/nvidia-isaac-gr00t-n1-open-humanoid-robot-foundation-model-simulation-frameworks)
4. Overview | Unitree G1 \- QRE DOCS, accessed December 7, 2025, [https://www.docs.quadruped.de/projects/g1/html/g1_overview.html](https://www.docs.quadruped.de/projects/g1/html/g1_overview.html)
5. NVIDIA Accelerates Robotics Research and Development With New Open Models and Simulation Libraries \- NVIDIA Investor Relations, accessed December 7, 2025, [https://investor.nvidia.com/news/press-release-details/2025/NVIDIA-Accelerates-Robotics-Research-and-Development-With-New-Open-Models-and-Simulation-Libraries/default.aspx](https://investor.nvidia.com/news/press-release-details/2025/NVIDIA-Accelerates-Robotics-Research-and-Development-With-New-Open-Models-and-Simulation-Libraries/default.aspx)
6. Top 12 Humanoid Robots of 2025, accessed December 7, 2025, [https://humanoidroboticstechnology.com/articles/top-12-humanoid-robots-of-2025/](https://humanoidroboticstechnology.com/articles/top-12-humanoid-robots-of-2025/)
7. R²D²: Three Neural Breakthroughs Transforming Robot Learning from NVIDIA Research, accessed December 7, 2025, [https://developer.nvidia.com/blog/r2d2-three-neural-breakthroughs-transforming-robot-learning-from-nvidia-research/](https://developer.nvidia.com/blog/r2d2-three-neural-breakthroughs-transforming-robot-learning-from-nvidia-research/)
8. Agibot Unveils Next-Gen Industrial-Grade Interactive Embodied Robot Agibot G2 \- PR Newswire, accessed December 7, 2025, [https://www.prnewswire.com/news-releases/agibot-unveils-next-gen-industrial-grade-interactive-embodied-robot-agibot-g2-302586152.html](https://www.prnewswire.com/news-releases/agibot-unveils-next-gen-industrial-grade-interactive-embodied-robot-agibot-g2-302586152.html)
9. Vision Language Action (VLA) Models Powering Robotics| Exxact Blog, accessed December 7, 2025, [https://www.exxactcorp.com/blog/deep-learning/vision-language-action-vla-models-powers-robotics](https://www.exxactcorp.com/blog/deep-learning/vision-language-action-vla-models-powers-robotics)
10. Helix: A Vision-Language-Action Model for Generalist Humanoid Control \- Figure AI, accessed December 7, 2025, [https://www.figure.ai/news/helix](https://www.figure.ai/news/helix)
11. Unitree Showcases Advanced High-Mobility Robots at CES 2025 | RoboticsTomorrow, accessed December 7, 2025, [https://www.roboticstomorrow.com/news/2025/01/08/unitree-showcases-advanced-high-mobility-robots-at-ces-2025/23818](https://www.roboticstomorrow.com/news/2025/01/08/unitree-showcases-advanced-high-mobility-robots-at-ces-2025/23818)
12. Unitree Launches Production Version of G1 Humanoid Robot \- Maginative, accessed December 7, 2025, [https://www.maginative.com/article/unitree-launches-production-version-of-g1-humanoid-robot/](https://www.maginative.com/article/unitree-launches-production-version-of-g1-humanoid-robot/)
13. Agility Robotics' Digit humanoid passes 100000-tote milestone in live GXO implementation, accessed December 7, 2025, [https://roboticsandautomationnews.com/2025/11/24/agility-robotics-digit-humanoid-passes-100000-tote-milestone-in-live-gxo-implementation/96877/](https://roboticsandautomationnews.com/2025/11/24/agility-robotics-digit-humanoid-passes-100000-tote-milestone-in-live-gxo-implementation/96877/)
