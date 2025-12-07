# Project Structure Documentation

# Physical AI & Humanoid Robotics Textbook

**Last Updated**: 2025-12-07  
**Version**: 3.0 (Module-Based Code-First Architecture)

## Overview

This document defines the comprehensive directory structure for the Physical AI educational textbook project, including frontend (Docusaurus with module-based content), backend (FastAPI), database schemas, **embedded labs with starter code**, **populated ROS 2 packages**, **Isaac Sim assets**, and deployment infrastructure.

**Key Architectural Principle**: Code-first, hands-on learning with 4 progressive modules (ROS 2 → Architecture → Isaac → VLA/Whisper). Students build real systems from Day 1.

## Root Structure

```
physical-ai-humanoids-textbook/
├── README.md                      # Project documentation
├── LICENSE                        # MIT License
├── .gitignore                     # Git exclusions
├── package.json                   # Root workspace configuration
├── docker-compose.yml             # Multi-container orchestration
├── Dockerfile.frontend            # Frontend container build
├── Dockerfile.backend             # Backend container build
│
├── frontend/                      # Docusaurus Frontend (Module-Based Content)
├── backend/                       # FastAPI RAG Chatbot Backend
├── database/                      # PostgreSQL Schemas & Migrations
├── auth/                          # Authentication System
├── labs/                          # Hands-on Lab Exercises (Complete with Starter/Solutions)
├── ros2_packages/                 # ROS 2 Workspace (Populated Examples)
├── isaac_assets/                  # NVIDIA Isaac Sim Assets (USD Scenes)
├── hardware/                      # Hardware Setup Guides (Jetson Orin Nano/NX)
├── cloud/                         # Cloud Deployment Configs
├── scripts/                       # Automation Scripts
├── grading/                       # Evaluation Infrastructure
├── assets/                        # Global Assets
├── .github/                       # CI/CD Workflows
├── .specify/                      # SpecKit Plus Configuration
├── history/                       # Prompt History Records
├── research/                      # Research Documents
└── specs/                         # Feature Specifications
```

## Frontend Structure (`frontend/`) - Module-Based Content

Docusaurus application with 4-module progressive curriculum and embedded lab references.

```
frontend/
├── docs/                          # Module content (Markdown files)
│   ├── intro.md                   # Homepage/landing
│   │
│   ├── module-01-ros2/            # MODULE 1: ROS 2 Nervous System (12 pages)
│   │   ├── 00-overview.md         # Module overview + learning outcomes
│   │   │
│   │   ├── 01-ros2-fundamentals/  # Chapter 1.1 (5 pages)
│   │   │   ├── index.md
│   │   │   ├── 1.1.1-architecture.md         # Theory → Diagram → Code → 🧪 Lab 1.1.1 → Checkpoint
│   │   │   ├── 1.1.2-rclpy-patterns.md
│   │   │   ├── 1.1.3-parameters-launch.md
│   │   │   └── 1.1.4-realtime-qos.md
│   │   │
│   │   ├── 02-urdf-robot-description/  # Chapter 1.2 (4 pages)
│   │   │   ├── index.md
│   │   │   ├── 1.2.1-urdf-xacro-basics.md
│   │   │   ├── 1.2.2-sensors-plugins.md
│   │   │   ├── 1.2.3-kinematics-validation.md
│   │   │   └── 1.2.4-package-testing.md
│   │   │
│   │   └── 03-sensors-proprioception/  # Chapter 1.3 (3 pages)
│   │       ├── index.md
│   │       ├── 1.3.1-imu-encoders-calibration.md
│   │       ├── 1.3.2-sensor-fusion.md
│   │       └── 1.3.3-jetson-orin-deployment.md  # Deploy to Orin Nano/NX
│   │
│   ├── module-02-architecture/    # MODULE 2: Core Robot Architecture (14 pages)
│   │   ├── 00-overview.md
│   │   │
│   │   ├── 01-mechanical-design-kinematics/  # Chapter 2.1 (5 pages)
│   │   │   ├── index.md
│   │   │   ├── 2.1.1-anatomy-dof-mapping.md
│   │   │   ├── 2.1.2-forward-inverse-kinematics.md
│   │   │   ├── 2.1.3-dynamics-com-basics.md
│   │   │   └── 2.1.4-design-checklist-prototyping.md
│   │   │
│   │   ├── 02-actuation-power-systems/  # Chapter 2.2 (5 pages)
│   │   │   ├── index.md
│   │   │   ├── 2.2.1-electric-qdd-hydraulic.md
│   │   │   ├── 2.2.2-series-elastic-tendon-driven.md
│   │   │   ├── 2.2.3-power-thermal-constraints.md
│   │   │   └── 2.2.4-actuator-selection-worksheet.md
│   │   │
│   │   └── 03-edge-compute-perception/  # Chapter 2.3 (4 pages)
│   │       ├── index.md
│   │       ├── 2.3.1-jetson-orin-nano-nx-guide.md  # ONLY Orin Nano/NX specs
│   │       ├── 2.3.2-perception-sensors-selection.md
│   │       ├── 2.3.3-realtime-inference-orin.md
│   │       └── 2.3.4-case-study-orin-perception-stack.md
│   │
│   ├── module-03-isaac-simulation/  # MODULE 3: NVIDIA Isaac & Simulation (13 pages)
│   │   ├── 00-overview.md
│   │   │
│   │   ├── 01-isaac-sim-ros-deep-dive/  # Chapter 3.1 (5 pages)
│   │   │   ├── index.md
│   │   │   ├── 3.1.1-isaac-sim-usd-scenes.md
│   │   │   ├── 3.1.2-isaac-ros-vslam-perception.md
│   │   │   ├── 3.1.3-nav2-humanoid-pathing.md
│   │   │   └── 3.1.4-synthetic-data-domain-randomization.md
│   │   │
│   │   ├── 02-gazebo-mujoco-unity-comparison/  # Chapter 3.2 (4 pages)
│   │   │   ├── index.md
│   │   │   ├── 3.2.1-gazebo-ros-workflows.md
│   │   │   ├── 3.2.2-mujoco-dynamics-rl.md
│   │   │   ├── 3.2.3-unity-hri-ui-scenarios.md
│   │   │   └── 3.2.4-simulator-selection-checklist.md
│   │   │
│   │   └── 03-sim-to-real-deployment-nav2/  # Chapter 3.3 (4 pages)
│   │       ├── index.md
│   │       ├── 3.3.1-domain-dynamics-randomization.md
│   │       ├── 3.3.2-policy-export-orin-inference.md
│   │       ├── 3.3.3-nav2-isaac-ros-integration.md
│   │       └── 3.3.4-case-study-isaac-orin-unitree.md
│   │
│   ├── module-04-vla-whisper/     # MODULE 4: VLA & Voice-to-Action (11 pages)
│   │   ├── 00-overview.md
│   │   │
│   │   ├── 01-vla-foundations-llm-to-action/  # Chapter 4.1 (4 pages)
│   │   │   ├── index.md
│   │   │   ├── 4.1.1-vla-architectures-grounding.md
│   │   │   ├── 4.1.2-cognitive-planning-safe-synthesis.md
│   │   │   ├── 4.1.3-llm-prompts-robotics-best-practices.md
│   │   │   └── 4.1.4-safety-validation-llm-outputs.md
│   │   │
│   │   ├── 02-voice-to-action-whisper-integration/  # Chapter 4.2 (3 pages)
│   │   │   ├── index.md
│   │   │   ├── 4.2.1-whisper-fundamentals-local-vs-cloud.md
│   │   │   ├── 4.2.2-building-voice-pipeline.md
│   │   │   └── 4.2.3-intent-extraction-error-recovery.md
│   │   │
│   │   └── 03-capstone-autonomous-humanoid/  # Chapter 4.3 (4 pages)
│   │       ├── index.md
│   │       ├── 4.3.1-capstone-spec-decomposition.md
│   │       ├── 4.3.2-evaluation-metrics-test-scenarios.md
│   │       ├── 4.3.3-deployment-options-fallback.md
│   │       └── 4.3.4-rubric-submission-portfolio.md
│   │
│   ├── glossary.md                # Technical terms glossary
│   └── resources.md               # Further reading, citations
│
├── src/                           # Docusaurus customization
│   ├── components/
│   │   ├── HomepageFeatures/
│   │   │   ├── index.tsx
│   │   │   └── styles.module.css
│   │   │
│   │   ├── ChatWidget/            # RAG chatbot UI component
│   │   │   ├── ChatWidget.tsx
│   │   │   ├── ChatWidget.module.css
│   │   │   └── index.ts
│   │   │
│   │   └── LabCallout/            # NEW: Inline lab reference component
│   │       ├── LabCallout.tsx     # Renders 🧪 Lab boxes with styling
│   │       ├── LabCallout.module.css
│   │       └── index.ts
│   │
│   ├── css/
│   │   ├── custom.css             # Global styles (purple/cyan theme)
│   │   └── navbar.css             # Professional navbar styling
│   │
│   └── pages/
│       ├── index.tsx              # Custom homepage (hero section)
│       └── index.module.css
│
├── static/
│   ├── img/
│   │   ├── logo.svg               # Light theme logo (purple gradient)
│   │   ├── logo-dark.svg          # Dark theme logo (cyan/pink)
│   │   └── favicon.ico
│   │
│   └── data/                      # Datasets (if any)
│
├── docusaurus.config.ts           # Docusaurus configuration
├── sidebars.ts                    # Sidebar navigation (module-based structure)
├── package.json                   # Node.js dependencies
├── tsconfig.json                  # TypeScript configuration
├── README.md                      # Frontend documentation
└── .gitignore
```

**Key Changes**:

- Content organized by modules (module-01-ros2/, module-02-architecture/, etc.)
- Each section includes inline lab references (🧪 Lab callouts)
- LabCallout component added for consistent styling
- Sidebar structure reflects 4-module progression

---

FastAPI application providing RAG chatbot, authentication, and personalization services.

```
backend/
├── app/
│   ├── __init__.py
│   ├── main.py                    # FastAPI app entry point
│   │
│   ├── routes/
│   │   ├── __init__.py
│   │   ├── chat.py                # POST /api/chat
│   │   ├── selection.py           # POST /api/selection-query
│   │   ├── auth.py                # /api/auth/* endpoints
│   │   ├── personalization.py     # GET/POST /api/personalize
│   │   └── translation.py         # POST /api/translate
│   │
│   ├── services/
│   │   ├── __init__.py
│   │   ├── rag_engine.py          # Core RAG logic
│   │   ├── embeddings.py          # OpenAI text-embedding-3-small
│   │   ├── vector_store.py        # Qdrant Cloud operations
│   │   ├── openai_client.py       # OpenAI ChatKit SDK
│   │   ├── neon_db.py             # Neon Postgres client
│   │   ├── content_adapter.py     # Personalization engine
│   │   └── translator.py          # Translation service
│   │
│   ├── models/
│   │   ├── __init__.py
│   │   ├── schemas.py             # Pydantic request/response models
│   │   ├── user.py                # User ORM model
│   │   └── conversation.py        # Chat history ORM model
│   │
│   ├── config/
│   │   ├── __init__.py
│   │   └── settings.py            # Environment variables (Pydantic)
│   │
│   └── middleware/
│       ├── __init__.py
│       ├── auth.py                # JWT verification
│       └── cors.py                # CORS configuration
│
├── tests/
│   ├── __init__.py
│   ├── test_rag_engine.py
│   ├── test_embeddings.py
│   ├── test_chat_api.py
│   └── test_auth.py
│
├── Dockerfile
├── requirements.txt               # Production dependencies
├── requirements-dev.txt           # Development dependencies
└── .env.example                   # Environment template
```

## Database Structure (`database/`)

PostgreSQL schemas, migrations, and seed data for Neon serverless database.

```
database/
├── schema/
│   ├── 001_users.sql              # User accounts with background
│   ├── 002_conversations.sql      # Chat history with RAG metadata
│   ├── 003_preferences.sql        # User personalization settings
│   └── 004_embeddings.sql         # Document embeddings metadata
│
├── migrations/
│   ├── 001_initial_setup.sql
│   └── 002_add_personalization.sql
│
└── seed/
    └── sample_data.sql
```

## Authentication Structure (`auth/`)

Better-auth configuration for email/OAuth authentication with user background surveys.

```
auth/
├── better-auth.config.py          # Better-auth initialization
├── providers.ts                   # Email, Google, GitHub OAuth
├── middleware.ts                  # Route protection
└── signup-questions.json          # User background questionnaire
```

## Frontend Structure (`frontend/`)

Docusaurus application with modular chapter organization, React components, and static assets.

```
frontend/
├── docusaurus.config.ts           # Docusaurus configuration
├── sidebars.ts                    # Sidebar navigation
├── package.json                   # Frontend dependencies
├── tsconfig.json                  # TypeScript configuration
│
├── docs/                          # Markdown content
│   ├── intro.md
│   │
│   ├── 01-module-1-ros2/
│   │   ├── 00-overview.md
│   │   ├── 01-ros2-fundamentals.md
│   │   │   ├── 1.1.1-architecture.md
│   │   │   ├── 1.1.2-rclpy-patterns.md
│   │   │   ├── 1.1.3-parameters-launch.md
│   │   │   └── 1.1.4-realtime-qos.md
│   │   ├── 02-urdf-robot-description.md
│   │   └── 03-sensors-proprioception.md
│   │
│   ├── 02-module-2-architecture/
│   │   ├── 00-overview.md
│   │   ├── 01-mechanics-kinematics.md
│   │   ├── 02-actuation-power.md
│   │   └── 03-edge-compute-perception.md
│   │
│   ├── 03-module-3-isaac/
│   │   ├── 00-overview.md
│   │   ├── 01-isaac-sim-ros.md
│   │   ├── 02-gazebo-mujoco-unity.md
│   │   └── 03-sim-to-real-deploy.md
│   │
│   ├── 04-module-4-vla-whisper/
│   │   ├── 00-overview.md
│   │   ├── 01-vla-foundations.md
│   │   ├── 02-voice-to-action-whisper.md
│   │   └── 03-capstone-project.md
│   │
│   ├── 05-capstone/
│   │   ├── requirements.md
│   │   ├── rubric.md
│   │   ├── demo-checklist.md
│   │   └── submission-template.md
│   │
│   ├── 06-glossary/
│   │   └── robotics-terms.md
│   │
│   ├── 07-resources/
│   │   ├── links.md
│   │   ├── recommended-books.md
│   │   ├── sensor-datasets.md
│   │   └── hardware-vendors.md
│   │
│   └── 08-assessments/
│       ├── quizzes/
│       └── practice-exams/
│
├── src/                           # React source code
│   ├── components/
│   │   ├── RAGChatbot/
│   │   │   ├── ChatWidget.js
│   │   │   ├── ChatWindow.js
│   │   │   ├── MessageList.js
│   │   │   ├── ChatInput.js
│   │   │   ├── TextSelectionQuery.js
│   │   │   ├── TypingIndicator.js
│   │   │   ├── hooks/
│   │   │   │   ├── useChat.js
│   │   │   │   └── useSelection.js
│   │   │   └── styles/
│   │   │       └── chatbot.module.css
│   │   │
│   │   ├── Personalization/
│   │   │   ├── PersonalizationContext.js
│   │   │   ├── DifficultyToggle.js
│   │   │   ├── ContentAdapter.js
│   │   │   └── BackgroundBadge.js
│   │   │
│   │   ├── Translation/
│   │   │   ├── LanguageSelector.js
│   │   │   ├── TranslateButton.js
│   │   │   └── UrduTranslator.js
│   │   │
│   │   ├── Interactive/
│   │   │   ├── Quiz.js
│   │   │   ├── LabChecklist.js
│   │   │   ├── CodeRunner.js
│   │   │   └── MermaidDiagram.js
│   │   │
│   │   ├── Auth/
│   │   │   ├── LoginForm.js
│   │   │   ├── SignupForm.js
│   │   │   └── BackgroundSurvey.js
│   │   │
│   │   └── HomepageFeatures/
│   │       └── index.tsx
│   │
│   ├── css/
│   │   ├── custom.css
│   │   ├── navbar.css
│   │   └── chatbot.css
│   │
│   ├── pages/
│   │   ├── index.tsx              # Custom homepage
│   │   ├── signup.js
│   │   ├── login.js
│   │   └── profile.js
│   │
│   └── theme/
│       └── Root.js                # Global providers
│
└── static/
    ├── img/
    │   ├── logo.svg
    │   ├── logo-dark.svg
    │   ├── diagrams/
    │   ├── hardware/
    │   └── screenshots/
    │
    ├── labs/
    │   ├── lab01/
    │   └── lab02/
    │
    ├── datasets/                  # RAG vector DB chunks
    │   └── processed/
    │       ├── module1-chunks.json
    │       ├── module2-chunks.json
    │       ├── module3-chunks.json
    │       └── module4-chunks.json
    │
    └── downloads/
        ├── setup-scripts/
        └── ros2-packages/
```

## Labs Structure (`labs/`)

Hands-on exercises with **complete starter code** (with TODO comments), solutions, tests, and assets. Each lab maps directly to module sections.

**Lab Philosophy**: Students fork starter code, complete TODOs, run tests, compare with solutions. NOT orphaned—each lab is referenced inline in module content via 🧪 Lab callouts.

```
labs/
├── lab01-ros2-basics/            # Module 1 Lab (COMPLETE IMPLEMENTATION)
│   ├── README.md                 # Learning objectives, prerequisites, estimated time, grading
│   ├── instructions.md           # Step-by-step guide with screenshots
│   │
│   ├── starter/                  # Actual ROS 2 package with TODOs
│   │   ├── package.xml           # Package manifest
│   │   ├── setup.py              # Python package setup
│   │   ├── setup.cfg             # Entry points
│   │   │
│   │   ├── launch/
│   │   │   └── basic_launch.py   # TODO: Add parameter loading
│   │   │
│   │   ├── config/
│   │   │   └── params.yaml       # TODO: Define publish rate parameter
│   │   │
│   │   └── src/
│   │       └── heartbeat_node.py # TODO: Implement timer callback, publish logic
│   │
│   ├── solutions/                # Complete working code (no TODOs)
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── launch/
│   │   │   └── basic_launch.py
│   │   ├── config/
│   │   │   └── params.yaml
│   │   ├── src/
│   │   │   └── heartbeat_node.py
│   │   └── README_SOLUTION.md    # Explanation of solution approach
│   │
│   ├── tests/
│   │   └── test_heartbeat_node.py # pytest assertions (node creates publisher, publishes at rate)
│   │
│   └── assets/
│       ├── expected_output.txt    # Sample terminal output
│       ├── demo.gif               # Screen recording of working lab
│       └── ros2_graph.png         # Expected ROS 2 graph visualization
│
├── lab02-urdf-humanoid/          # Module 1 Lab (URDF creation)
│   ├── README.md
│   ├── starter/
│   │   ├── package.xml
│   │   └── urdf/
│   │       └── simple_humanoid.urdf.xacro  # TODO: Complete joint definitions, add sensors
│   ├── solutions/
│   │   └── urdf/
│   │       └── simple_humanoid.urdf.xacro
│   └── assets/
│       └── rviz_view.png          # Expected RViz visualization
│
├── lab03-isaac-sim-scene/        # Module 3 Lab (Isaac Sim basics)
│   ├── README.md
│   ├── starter/
│   │   └── scenes/
│   │       └── warehouse_simple.usd  # Basic scene with TODOs in comments
│   ├── solutions/
│   │   └── scenes/
│   │       └── warehouse_complete.usd
│   └── assets/
│       ├── isaac_sim_screenshot.png
│       └── spawn_coords.json      # Robot spawn coordinates
│
├── lab04-isaac-ros-vslam/        # Module 3 Lab (VSLAM + Nav2)
│   ├── README.md
│   ├── starter/
│   │   ├── launch/
│   │   │   └── vslam_nav2.launch.py  # TODO: Configure VSLAM params
│   │   └── config/
│   │       └── nav2_params.yaml      # TODO: Set robot footprint
│   ├── solutions/
│   └── assets/
│       └── nav2_path_visualization.png
│
├── lab05-whisper-voice-command/  # Module 4 Lab (Whisper integration)
│   ├── README.md
│   ├── starter/
│   │   ├── package.xml
│   │   ├── scripts/
│   │   │   ├── whisper_node.py       # TODO: Integrate Whisper API, publish transcription
│   │   │   └── action_mapper.py      # TODO: Map voice intents to ROS actions
│   │   └── config/
│   │       └── intents.json          # Intent mapping dictionary
│   ├── solutions/
│   ├── tests/
│   │   └── test_whisper_integration.py
│   └── assets/
│       ├── voice_commands.wav        # Sample audio file
│       └── expected_topics.txt       # Expected ROS topic outputs
│
├── lab06-capstone-autonomous/    # Module 4 Capstone Lab
│   ├── README.md
│   ├── requirements.md               # Full pipeline spec (Whisper → LLM → Nav2 → perception → manipulation)
│   ├── rubric.md                     # Grading criteria (voice accuracy, nav success, object detection precision)
│   ├── demo-checklist.md             # Pre-demo testing, video recording requirements
│   │
│   ├── starter/
│   │   ├── launch/
│   │   │   └── capstone_system.launch.py  # TODO: Integrate all modules
│   │   ├── scripts/
│   │   │   ├── llm_planner.py        # TODO: LLM → action plan conversion
│   │   │   └── system_orchestrator.py # TODO: Coordinate voice → plan → execute
│   │   └── config/
│   │       └── system_params.yaml
│   │
│   ├── solutions/                    # Full working capstone (reference implementation)
│   │
│   └── assets/
│       ├── demo_video_template.mp4
│       └── deployment_options.md     # Orin + Unitree Go2 vs cloud options
│
└── README.md                         # Lab overview, prerequisites, setup instructions
```

**Key Additions**:

- Lab 1 has **complete ROS 2 package structure** (package.xml, setup.py, src/ with TODOs)
- Every lab includes **solutions/** directory for self-checking
- **tests/** directory with pytest assertions for automated validation
- **assets/** for visual validation (expected output, screenshots, demo GIFs)
- Capstone lab includes **rubric.md** and **demo-checklist.md** for evaluation

---

## ROS 2 Packages Structure (`ros2_packages/`)

**POPULATED** ROS 2 workspace with example packages students can use as references.

```
ros2_packages/
├── humanoid_description/         # Example URDF package (COMPLETE)
│   ├── urdf/
│   │   ├── torso.xacro           # Torso link + joints
│   │   ├── arm.xacro             # 7-DOF arm macro (shoulder, elbow, wrist)
│   │   ├── leg.xacro             # 6-DOF leg macro (hip, knee, ankle)
│   │   ├── sensors.xacro         # Camera + IMU + LiDAR macros
│   │   └── simple_humanoid.urdf.xacro  # Main assembly file
│   │
│   ├── meshes/                   # Collision and visual meshes (simplified)
│   │   ├── torso.stl
│   │   ├── upper_arm.stl
│   │   └── ... (other parts)
│   │
│   ├── launch/
│   │   └── display.launch.py     # Launch RViz with robot_state_publisher
│   │
│   ├── config/
│   │   └── joint_limits.yaml     # Joint position/velocity/effort limits
│   │
│   ├── package.xml
│   ├── setup.py
│   └── README.md                 # How to load URDF, view in RViz, check kinematics
│
├── humanoid_control/             # Basic joint control examples
│   ├── src/
│   │   ├── joint_trajectory_controller.py  # Publish JointTrajectory commands
│   │   └── teleoperation_node.py          # Keyboard teleop for testing
│   ├── config/
│   │   └── controller_config.yaml
│   ├── launch/
│   │   └── teleop.launch.py
│   ├── package.xml
│   └── README.md
│
├── perception_pipeline/          # Sensor fusion example
│   ├── nodes/
│   │   ├── realsense_processor.py  # Subscribe to /camera/depth/image_raw
│   │   └── imu_calibration.py      # IMU bias calibration node
│   ├── launch/
│   │   └── perception.launch.py
│   ├── package.xml
│   └── README.md
│
├── nav2_config/                  # Nav2 configuration for humanoid
│   ├── maps/
│   │   └── lab_map.yaml          # Example map for testing
│   ├── params/
│   │   └── nav2_params.yaml      # Nav2 tuned for bipedal footprint
│   ├── launch/
│   │   └── nav2_humanoid.launch.py
│   ├── package.xml
│   └── README.md
│
└── voice_to_action/              # Whisper integration package (COMPLETE)
    ├── scripts/
    │   ├── whisper_node.py       # Capture audio → Whisper API → publish transcription
    │   └── action_mapper.py      # Map voice intents to ROS action calls
    │
    ├── config/
    │   ├── intents.json          # Intent mapping: {"go forward": {"action": "cmd_vel", "params": {"linear_x": 0.5}}}
    │   └── whisper_config.yaml   # Model selection, API key, audio device
    │
    ├── launch/
    │   └── voice_control.launch.py
    │
    ├── package.xml
    ├── setup.py
    └── README.md                 # Setup Whisper, configure microphone, test voice commands
```

**Key Additions**:

- `humanoid_description/` is **fully populated** with URDF/xacro files (not empty)
- `voice_to_action/` includes **complete Whisper integration** code
- Each package has **README.md** with usage instructions
- `nav2_config/` includes **humanoid-specific parameters** (bipedal footprint)

```

## Isaac Assets Structure (`isaac_assets/`)

NVIDIA Isaac Sim USD files, scenes, datasets, and scripts.

```

isaac_assets/
├── usd/
│ ├── humanoid_robot.usd
│ ├── lab_environment.usd
│ ├── objects/
│ └── materials/
│
├── scenes/
│ ├── pick-and-place/
│ ├── navigation-arena/
│ └── manipulation-workspace/
│
├── datasets/
│ ├── synthetic_rgb/
│ ├── synthetic_depth/
│ ├── annotations/
│ └── config/
│ └── dataset_config.yaml
│
└── scripts/
├── generate_dataset.py
├── randomize_domain.py
├── isaac_vslam_example.py
└── export_to_ros.py

```

## Hardware Structure (`hardware/`)

Setup guides for Jetson, sensors, and robots.

```

hardware/
├── jetson/
│ ├── orin-nano-setup.md
│ ├── orin-nx-setup.md
│ ├── thor-setup.md
│ ├── calibration/
│ │ ├── imu_calibration.py
│ │ └── camera_calibration.py
│ ├── udev_rules/
│ │ └── 99-realsense.rules
│ └── system_images/
│
├── sensors/
│ ├── realsense-setup.md
│ ├── lidar-setup.md
│ └── imu-setup.md
│
└── robots/
├── unitree-go2-setup.md
├── unitree-g1-setup.md
└── integration-guide.md

```

## Cloud Deployment Structure (`cloud/`)

Infrastructure as code for AWS, Azure, and GCP.

```

cloud/
├── aws/
│ ├── README.md
│ ├── ami-spec.json
│ ├── setup-script.sh
│ └── terraform/
│ └── main.tf
│
├── azure/
│ ├── README.md
│ └── arm-template.json
│
└── gcp/
├── README.md
└── deployment.yaml

```

## Scripts Structure (`scripts/`)

Automation for building, testing, RAG seeding, and deployment.

```

scripts/
├── build_docs.sh
├── setup_dev_env.sh
├── run_ros2_tests.sh
├── generate_diagrams.sh
│
├── rag/
│ ├── seed-vector-db.py # Populate Qdrant
│ ├── process-markdown.py # Extract text chunks
│ ├── update-embeddings.sh # Refresh embeddings
│ └── test-rag-pipeline.py # Test queries
│
└── deployment/
├── deploy-frontend.sh
├── deploy-backend.sh
└── health-check.sh

```

## Grading Structure (`grading/`)

Automated evaluation and rubrics.

```

grading/
├── automated/
│ ├── test_lab01.py
│ ├── test_lab02.py
│ ├── test_lab03.py
│ └── grade_capstone.py
│
└── rubrics/
├── lab_rubric.md
├── capstone_rubric.md
└── grading_guide.md

```

## GitHub Workflows (`.github/`)

CI/CD pipelines for testing and deployment.

```

.github/
└── workflows/
├── deploy-frontend.yml # GitHub Pages deployment
├── backend-tests.yml # pytest on push
├── prose-lint.yml # Markdown quality
└── vector-db-update.yml # Re-seed Qdrant

```

## Configuration Files

### Root Level

- **package.json**: Workspace configuration with dev/build/docker scripts
- **docker-compose.yml**: Frontend + Backend orchestration
- **Dockerfile.frontend**: Nginx-based static site build
- **Dockerfile.backend**: Python FastAPI container
- **.gitignore**: Excludes node_modules, **pycache**, .env, build artifacts
- **README.md**: Project documentation with quick start

### Backend

- **requirements.txt**: FastAPI, qdrant-client, openai, pydantic, sqlalchemy
- **.env.example**: Template for DATABASE_URL, QDRANT_URL, OPENAI_API_KEY, JWT_SECRET

### Frontend (frontend/)

- **docusaurus.config.ts**: Site configuration, Mermaid support, navbar, footer
- **sidebars.ts**: Module-based sidebar with collapsible sections
- **package.json**: Docusaurus 3.9, React 18, Mermaid plugin
- **tsconfig.json**: TypeScript compilation settings

## Key Principles

1. **Modular Architecture**: Docs organized by learning modules, not flat chapters
2. **Separation of Concerns**: Frontend (Docusaurus), Backend (FastAPI), Database (Neon), Vector DB (Qdrant)
3. **Scalability**: Cloud deployment configs for multiple providers
4. **Developer Experience**: Clear directory naming, comprehensive READMEs, starter code
5. **Production Ready**: Docker support, CI/CD workflows, automated testing
6. **Educational Focus**: Labs with solutions, quizzes, capstone project, grading rubrics

## Migration Notes

### From Old Structure

- **Old**: `docs/docs/chapters/chapter-01-foundations.md`
- **New**: `frontend/docs/01-module-1-ros2/01-ros2-fundamentals.md` (with nested sections)

### Content Organization

- Module-based grouping for progressive learning
- Each module has overview + 3-4 main topics
- Topics have 1.1.1, 1.1.2 style subsections
- Supporting materials in separate directories (capstone, glossary, resources, assessments)

## Version History

- **v1.0** (2025-12-07): Initial flat chapter structure
- **v2.0** (2025-12-07): Modular architecture with backend, labs, ROS 2 packages, comprehensive infrastructure

---

**Last Updated**: 2025-12-07
**Maintained By**: shazilk-dev
```
