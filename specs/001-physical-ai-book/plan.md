# Implementation Plan: Physical AI & Humanoid Robotics Educational Book

**Branch**: `001-physical-ai-book` | **Date**: 2025-12-07 | **Updated**: 2025-12-07 (Module-based restructure) | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-physical-ai-book/spec.md`

## Summary

Build a code-first, hands-on educational book covering Physical AI and Humanoid Robotics using a 4-module progressive learning structure (50 pages total: Module 1: ROS 2 - 12p, Module 2: Architecture - 14p, Module 3: Isaac - 13p, Module 4: VLA/Whisper - 11p) with integrated RAG chatbot for interactive learning. Each module follows the pattern: Theory (300-400 words) → Visual (diagram/table) → Code Example (10-20 lines) → 🧪 Lab (starter code with TODOs) → ✅ Checkpoint (learning outcome). The book will be deployed as a static Docusaurus site to GitHub Pages with a FastAPI backend on Render providing RAG capabilities using Qdrant vector database and OpenAI models. Core objectives: deliver 2025-accurate technical content with embedded labs, enable natural language queries over module content, maintain <2s page loads and <3s chatbot responses, operate within free-tier infrastructure costs (<$10/month).

**Key Architectural Change**: Content structure shifted from traditional 10-chapter textbook to 4-module skill-building curriculum with embedded labs. This aligns with GIAIC course philosophy (code-first, hands-on) and ensures students build real systems (ROS 2 packages, Isaac Sim scenes, Whisper integration) from Day 1.

## Technical Context

**Frontend Language/Version**:

- TypeScript 5.3+ / JavaScript ES2022
- Node.js 18.x LTS (required by Docusaurus 3.9)
- React 18.3+

**Backend Language/Version**:

- Python 3.11+ (FastAPI requirement, async/await support)

**Primary Frontend Dependencies**:

- Docusaurus 3.9.3 (static site generator)
- @docusaurus/preset-classic (blog, docs, pages)
- @docusaurus/theme-mermaid (diagram rendering)
- docusaurus-theme-search-algolia (search functionality)
- prism-react-renderer (syntax highlighting)
- openai (for ChatKit SDK integration)

**Primary Backend Dependencies**:

- FastAPI 0.115+ (async web framework)
- uvicorn[standard] (ASGI server)
- qdrant-client 1.12+ (vector database SDK)
- openai 1.54+ (embeddings and chat completions)
- pydantic 2.10+ (data validation)
- python-dotenv (environment variable management)
- fastapi-cors-middleware (cross-origin requests)

**Storage**:

- Qdrant Cloud (vector database, 1GB free tier)
- GitHub repository (source code and content)
- GitHub Pages (static site hosting)

**Testing**:

- Frontend: None initially (time constraint); manual testing via browser
- Backend: pytest with pytest-asyncio for FastAPI endpoints
- Content: Manual review against constitution quality gates
- Integration: Manual end-to-end testing (chatbot queries, page loads)

**Target Platform**:

- Frontend: Web browsers (Chrome, Firefox, Safari, Edge last 2 years)
- Frontend Build: Node.js 18.x on GitHub Actions (Ubuntu latest)
- Backend: Linux server (Render free tier, Ubuntu-based)
- Deployment: GitHub Pages (frontend), Render (backend API)

**Project Type**: Web application (static frontend + API backend)

**Performance Goals**:

- Frontend: Lighthouse Performance score ≥90
- Frontend: First Contentful Paint (FCP) <1.5s
- Frontend: Time to Interactive (TTI) <2s on 4G
- Frontend: Bundle size <300KB gzipped per page
- Backend: RAG query response <3s (95th percentile)
- Backend: API throughput 20 queries/minute (Render free tier limit)

**Constraints**:

- Cost: <$10/month total (OpenAI API only chargeable component)
- Qdrant: 1GB storage limit (must optimize embedding strategy for modules + labs)
- Render: 512MB RAM, shared CPU (free tier)
- GitHub Pages: 1GB site size, 100GB/month bandwidth
- OpenAI: Rate limits 3 req/min (tier 1), 200K tokens/day
- Time: Hackathon deadline (prioritize core features: Module 1 + Lab 1 first)
- Accessibility: WCAG AA compliance mandatory
- Content: Strict blueprint adherence (50 pages, 4 modules)
- Hardware: References must use ONLY Jetson Orin Nano/NX (NO Thor)

**Scale/Scope**:

- Content: 4 modules, 50 pages, ~30,000 words (more concise than theory-heavy approach)
- Visuals: Minimum 8 Mermaid diagrams, 8 comparison tables, ~20 code examples (runnable snippets)
- Labs: Minimum 4 complete labs (1 per module) with starter code, solutions, tests
- Users: 1000-5000 page views/day expected
- Concurrent: 100+ simultaneous readers supported
- RAG: ~200-300 document chunks indexed (modules + sections + labs with metadata)
- Deployment: 1 production environment (GitHub Pages + Render)

## Constitution Check

_GATE: Must pass before Phase 0 research. Re-check after Phase 1 design._

### ✅ I. 2025-Accurate Technical Depth

- **Status**: PASS
- **Evidence**: Spec requires all facts cited with verifiable sources (FR-005), technical specs must be 2025-current
- **Plan Impact**: Research phase will validate all technical specifications against manufacturer docs, academic papers, market reports
- **Validation**: Citations included in all chapters, code examples tested with current API versions

### ✅ II. Progressive Difficulty Architecture

- **Status**: PASS
- **Evidence**: Content explicitly structured beginner → advanced (Chapters 1-10), each chapter builds on previous
- **Plan Impact**: Chapter development order enforced (1, 3, 6 first; then 2, 4, 5; then 8, 9; finally 7, 10)
- **Validation**: Learning objectives stated at chapter start, prerequisites checked in content reviews

### ✅ III. Blueprint Conformance (NON-NEGOTIABLE)

- **Status**: PASS
- **Evidence**: Spec FR-001 mandates MASTER-BLUEPRINT.md as authoritative source for structure, depth, visuals
- **Plan Impact**: Each chapter follows exact blueprint specifications (page count, subchapters, diagrams, tables)
- **Validation**: Quality gate checklist validates 5-8 pages, 3-4 subchapters, minimum 1 diagram per chapter

### ✅ IV. Visual-First Technical Communication

- **Status**: PASS
- **Evidence**: Spec FR-003 requires minimum 1 Mermaid diagram/chapter, FR-004 requires 8 comparison tables with real data
- **Plan Impact**: Visual content created alongside text content, diagrams rendered and tested in light/dark modes
- **Validation**: Every technical concept has supporting visual (diagram, table, or code example)

### ✅ V. Implementation-Oriented Content

- **Status**: PASS
- **Evidence**: Spec FR-007 mandates code examples, FR-008 requires hands-on exercises where applicable
- **Plan Impact**: Code examples tested and validated, real-world case studies (BMW, Figure AI deployments) included
- **Validation**: Code blocks executable, trade-offs explained with concrete scenarios

### ✅ VI. Accessibility and Performance Standards

- **Status**: PASS
- **Evidence**: FR-023 to FR-033 define performance (<2s load) and accessibility (WCAG AA) requirements
- **Plan Impact**: Lighthouse CI pipeline validates ≥90 scores, WCAG AA compliance checked
- **Validation**: Alt text for all images, keyboard navigation tested, contrast ratios verified

### ✅ VII. Source Traceability for RAG Chatbot

- **Status**: PASS
- **Evidence**: FR-017 requires chapter/section citations, FR-016 mandates book-only responses
- **Plan Impact**: RAG chunking strategy preserves chapter/section metadata, embedding includes source identifiers
- **Validation**: Test queries verify correct citations, out-of-scope queries declined gracefully

**Overall Gate Status**: ✅ **PASS** - All constitutional principles satisfied by specification and implementation plan

**Notes**: No complexity justifications needed. Plan aligns with all principles without compromises.

## Project Structure

### Documentation (this feature)

```text
specs/001-physical-ai-book/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature specification (completed)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   └── rag-api.openapi.yaml
├── checklists/
│   └── requirements.md  # Validation checklist (completed)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
physical-ai-robotics-book/
├── frontend/                       # Docusaurus Frontend Application
│   ├── docs/                       # Module content (Markdown files)
│   │   ├── intro.md                # Homepage/landing content
│   │   │
│   │   ├── module-01-ros2/         # Module 1: ROS 2 Nervous System (12 pages)
│   │   │   ├── 00-overview.md      # Module overview + learning outcomes
│   │   │   ├── 01-ros2-fundamentals/
│   │   │   │   ├── index.md        # Chapter intro (5 pages total)
│   │   │   │   ├── 1.1.1-architecture.md  # Section with Theory → Diagram → Lab link
│   │   │   │   ├── 1.1.2-rclpy-patterns.md
│   │   │   │   ├── 1.1.3-parameters-launch.md
│   │   │   │   └── 1.1.4-realtime-qos.md
│   │   │   ├── 02-urdf-robot-description/
│   │   │   │   ├── index.md
│   │   │   │   ├── 1.2.1-urdf-xacro-basics.md
│   │   │   │   ├── 1.2.2-sensors-plugins.md
│   │   │   │   ├── 1.2.3-kinematics-validation.md
│   │   │   │   └── 1.2.4-package-testing.md
│   │   │   └── 03-sensors-proprioception/
│   │   │       ├── index.md
│   │   │       ├── 1.3.1-imu-encoders-calibration.md
│   │   │       ├── 1.3.2-sensor-fusion.md
│   │   │       └── 1.3.3-jetson-orin-deployment.md
│   │   │
│   │   ├── module-02-architecture/  # Module 2: Core Robot Architecture (14 pages)
│   │   │   ├── 00-overview.md
│   │   │   ├── 01-mechanical-design-kinematics/
│   │   │   │   ├── index.md
│   │   │   │   ├── 2.1.1-anatomy-dof-mapping.md
│   │   │   │   ├── 2.1.2-forward-inverse-kinematics.md
│   │   │   │   ├── 2.1.3-dynamics-com-basics.md
│   │   │   │   └── 2.1.4-design-checklist-prototyping.md
│   │   │   ├── 02-actuation-power-systems/
│   │   │   │   ├── index.md
│   │   │   │   ├── 2.2.1-electric-qdd-hydraulic.md
│   │   │   │   ├── 2.2.2-series-elastic-tendon-driven.md
│   │   │   │   ├── 2.2.3-power-thermal-constraints.md
│   │   │   │   └── 2.2.4-actuator-selection-worksheet.md
│   │   │   └── 03-edge-compute-perception/
│   │   │       ├── index.md
│   │   │       ├── 2.3.1-jetson-orin-nano-nx-guide.md  # ONLY Orin Nano/NX, NO Thor
│   │   │       ├── 2.3.2-perception-sensors-selection.md
│   │   │       ├── 2.3.3-realtime-inference-orin.md
│   │   │       └── 2.3.4-case-study-orin-perception-stack.md
│   │   │
│   │   ├── module-03-isaac-simulation/ # Module 3: NVIDIA Isaac & Simulation (13 pages)
│   │   │   ├── 00-overview.md
│   │   │   ├── 01-isaac-sim-ros-deep-dive/
│   │   │   │   ├── index.md
│   │   │   │   ├── 3.1.1-isaac-sim-usd-scenes.md
│   │   │   │   ├── 3.1.2-isaac-ros-vslam-perception.md
│   │   │   │   ├── 3.1.3-nav2-humanoid-pathing.md
│   │   │   │   └── 3.1.4-synthetic-data-domain-randomization.md
│   │   │   ├── 02-gazebo-mujoco-unity-comparison/
│   │   │   │   ├── index.md
│   │   │   │   ├── 3.2.1-gazebo-ros-workflows.md
│   │   │   │   ├── 3.2.2-mujoco-dynamics-rl.md
│   │   │   │   ├── 3.2.3-unity-hri-ui-scenarios.md
│   │   │   │   └── 3.2.4-simulator-selection-checklist.md
│   │   │   └── 03-sim-to-real-deployment-nav2/
│   │   │       ├── index.md
│   │   │       ├── 3.3.1-domain-dynamics-randomization.md
│   │   │       ├── 3.3.2-policy-export-orin-inference.md
│   │   │       ├── 3.3.3-nav2-isaac-ros-integration.md
│   │   │       └── 3.3.4-case-study-isaac-orin-unitree.md
│   │   │
│   │   ├── module-04-vla-whisper/     # Module 4: VLA & Voice-to-Action (11 pages)
│   │   │   ├── 00-overview.md
│   │   │   ├── 01-vla-foundations-llm-to-action/
│   │   │   │   ├── index.md
│   │   │   │   ├── 4.1.1-vla-architectures-grounding.md
│   │   │   │   ├── 4.1.2-cognitive-planning-safe-synthesis.md
│   │   │   │   ├── 4.1.3-llm-prompts-robotics-best-practices.md
│   │   │   │   └── 4.1.4-safety-validation-llm-outputs.md
│   │   │   ├── 02-voice-to-action-whisper-integration/
│   │   │   │   ├── index.md
│   │   │   │   ├── 4.2.1-whisper-fundamentals-local-vs-cloud.md
│   │   │   │   ├── 4.2.2-building-voice-pipeline.md
│   │   │   │   └── 4.2.3-intent-extraction-error-recovery.md
│   │   │   └── 03-capstone-autonomous-humanoid/
│   │   │       ├── index.md
│   │   │       ├── 4.3.1-capstone-spec-decomposition.md
│   │   │       ├── 4.3.2-evaluation-metrics-test-scenarios.md
│   │   │       ├── 4.3.3-deployment-options-fallback.md
│   │   │       └── 4.3.4-rubric-submission-portfolio.md
│   │   │
│   │   ├── glossary.md              # Technical terms glossary
│   │   └── resources.md             # Further reading, citations
│   │
│   ├── src/                         # Docusaurus customization
│   │   ├── components/
│   │   │   ├── ChatWidget/          # RAG chatbot UI component
│   │   │   │   ├── ChatWidget.tsx
│   │   │   │   ├── ChatWidget.module.css
│   │   │   │   └── index.ts
│   │   │   └── LabCallout/          # Inline lab reference component
│   │   │       ├── LabCallout.tsx   # Renders 🧪 Lab boxes with styling
│   │   │       └── LabCallout.module.css
│   │   ├── css/
│   │   │   ├── custom.css           # Global styles, theme overrides
│   │   │   └── navbar.css           # Professional navbar styling
│   │   └── pages/
│   │       └── index.tsx            # Custom homepage (hero section)
│   │
│   ├── static/
│   │   ├── img/                     # Images, logos
│   │   │   ├── logo.svg             # Light theme logo
│   │   │   └── logo-dark.svg        # Dark theme logo
│   │   └── data/                    # Datasets (if any)
│   │
│   ├── docusaurus.config.ts         # Docusaurus configuration
│   ├── sidebars.ts                  # Sidebar navigation (module-based)
│   ├── package.json                 # Node.js dependencies
│   ├── tsconfig.json                # TypeScript configuration
│   └── README.md                    # Frontend documentation
│
├── backend/                         # FastAPI RAG Backend (unchanged structure)
│   ├── app/
│   │   ├── main.py
│   │   ├── routes/                  # API endpoints
│   │   ├── services/                # RAG engine, embeddings, vector store
│   │   ├── models/                  # Pydantic schemas
│   │   ├── config/                  # Settings
│   │   └── middleware/              # Auth, CORS
│   ├── tests/
│   ├── requirements.txt
│   └── .env.example
│
├── labs/                            # Hands-on Lab Exercises
│   ├── lab01-ros2-basics/           # Module 1 Lab (COMPLETE)
│   │   ├── README.md                # Learning objectives, instructions, grading
│   │   ├── starter/                 # Starter code with TODOs
│   │   │   ├── package.xml
│   │   │   ├── setup.py
│   │   │   ├── launch/
│   │   │   │   └── basic_launch.py
│   │   │   └── src/
│   │   │       └── heartbeat_node.py  # TODO: Implement publisher logic
│   │   ├── solutions/               # Complete working code
│   │   │   └── ... (same structure, no TODOs)
│   │   ├── tests/
│   │   │   └── test_heartbeat_node.py  # pytest assertions
│   │   └── assets/
│   │       └── expected_output.txt     # Sample output for validation
│   │
│   ├── lab02-urdf-humanoid/         # Module 1 Lab (starter template)
│   │   ├── README.md
│   │   ├── starter/
│   │   │   └── urdf/
│   │   │       └── simple_humanoid.urdf.xacro  # TODO: Complete joint definitions
│   │   └── solutions/
│   │
│   ├── lab03-isaac-sim-scene/       # Module 3 Lab (starter template)
│   │   ├── README.md
│   │   ├── starter/
│   │   │   └── scenes/
│   │   │       └── warehouse_simple.usd  # Basic scene with TODOs
│   │   └── solutions/
│   │
│   └── lab04-whisper-voice-command/ # Module 4 Lab (starter template)
│       ├── README.md
│       ├── starter/
│       │   └── src/
│       │       └── whisper_node.py  # TODO: Integrate Whisper API
│       └── solutions/
│
├── ros2_packages/                   # ROS 2 Workspace (populated examples)
│   ├── humanoid_description/        # Example URDF package
│   │   ├── urdf/
│   │   │   └── simple_humanoid.urdf.xacro
│   │   ├── launch/
│   │   │   └── display.launch.py
│   │   ├── package.xml
│   │   └── README.md
│   │
│   └── voice_to_action/             # Whisper integration package
│       ├── scripts/
│       │   ├── whisper_node.py
│       │   └── action_mapper.py
│       ├── config/
│       │   └── intents.json
│       ├── package.xml
│       └── README.md
│
├── isaac_assets/                    # NVIDIA Isaac Sim Assets
│   ├── scenes/
│   │   └── lab04-first-scene/
│   │       ├── warehouse_simple.usd
│   │       ├── humanoid_spawn_point.json
│   │       └── README.md
│   └── robots/                      # Robot USD files (if custom)
│
├── hardware/                        # Hardware Setup Guides
│   ├── jetson/
│   │   ├── orin-nano-setup.md       # Flash, install ROS 2, configure
│   │   └── orin-nx-setup.md
│   └── sensors/
│       ├── realsense-d435i-setup.md
│       └── respeaker-setup.md
│
├── scripts/
│   ├── deployment/
│   └── rag/
│       └── seed-vector-db.py        # Index modules + labs to Qdrant
│
├── database/                        # PostgreSQL Schemas (unchanged)
│   ├── schema/
│   └── migrations/
│
├── .github/
│   └── workflows/
│       ├── deploy-book.yml          # Deploy Docusaurus to GitHub Pages
│       └── deploy-api.yml           # Deploy FastAPI to Render
│
├── package.json                     # Root workspace configuration
├── docker-compose.yml               # Multi-container orchestration
├── Dockerfile.frontend
├── Dockerfile.backend
├── README.md                        # Project documentation
└── .gitignore
```

**Structure Decision**: Module-based content architecture with embedded labs. Key changes from original plan:

1. **Content Organization**: Shifted from `chapters/chapter-01-foundations.md` to `module-01-ros2/01-ros2-fundamentals/1.1.1-architecture.md` (progressive module structure)
2. **Lab Integration**: Each module references labs/ directory with complete starter code, solutions, and tests (not orphaned)
3. **ROS 2 Workspace**: Populated ros2_packages/ with actual packages (humanoid_description, voice_to_action) instead of empty folders
4. **Isaac Assets**: Included isaac_assets/scenes/ with USD files and spawn configs
5. **Hardware Guides**: hardware/jetson/ contains Orin Nano/NX-specific setup (NO Thor references)
6. **Component Addition**: Added LabCallout.tsx component for rendering inline 🧪 Lab boxes

This structure supports the code-first pedagogy: students read theory in modules, see code examples inline, click lab links to work with actual ROS 2 packages, and progress from Module 1 (ROS basics) to Module 4 (voice-driven autonomous robot).
│ │ ├── 03-tech-pillars.md
│ │ └── 04-market-inflection.md
│ ├── chapter-02/ # Chapter 2: Mechanical Design
│ │ ├── index.md
│ │ ├── 01-kinematic-chains.md
│ │ ├── 02-actuation-technologies.md
│ │ └── 03-structural-design.md
│ ├── chapter-03/ # Chapter 3: Edge Compute
│ │ ├── index.md
│ │ ├── 01-edge-computing.md
│ │ ├── 02-sensor-fusion.md
│ │ ├── 03-high-performance-platforms.md
│ │ └── 04-perception-pipeline.md
│ ├── chapter-04/ # Chapter 4: ROS 2 Ecosystem
│ ├── chapter-05/ # Chapter 5: Whole-Body Control
│ ├── chapter-06/ # Chapter 6: Generative Physical AI
│ ├── chapter-07/ # Chapter 7: Deployment
│ ├── chapter-08/ # Chapter 8: Commercial Landscape
│ ├── chapter-09/ # Chapter 9: Applications
│ ├── chapter-10/ # Chapter 10: Future Directions
│ └── resources.md # Further reading, citations
│
├── src/ # Docusaurus customization
│ ├── components/
│ │ └── ChatWidget/ # RAG chatbot UI component
│ │ ├── ChatWidget.tsx # React component
│ │ ├── ChatWidget.module.css
│ │ └── index.ts
│ ├── css/
│ │ └── custom.css # Global styles, theme overrides
│ └── pages/
│ └── index.tsx # Custom homepage (hero section)
│
├── api/ # FastAPI backend
│ ├── main.py # API routes, CORS config
│ ├── embeddings.py # OpenAI embedding functions
│ ├── qdrant_client.py # Vector DB connection
│ ├── models.py # Pydantic models
│ ├── config.py # Environment variable loading
│ ├── requirements.txt # Python dependencies
│ └── .env.example # Environment template
│
├── scripts/
│ └── index_content.py # Index book chapters to Qdrant
│
├── static/
│ ├── img/ # Images, logos
│ └── data/ # Datasets (if any)
│
├── .github/
│ └── workflows/
│ ├── deploy-book.yml # Deploy Docusaurus to GitHub Pages
│ └── deploy-api.yml # Deploy FastAPI to Render
│
├── docusaurus.config.js # Docusaurus configuration
├── sidebars.js # Sidebar navigation structure
├── package.json # Node.js dependencies
├── tsconfig.json # TypeScript configuration
├── README.md # Project documentation
└── .gitignore

```

**Structure Decision**: Web application with separate frontend (Docusaurus static site) and backend (FastAPI REST API). Frontend is built and deployed to GitHub Pages as static files. Backend is containerized and deployed to Render. This separation allows independent scaling and adheres to JAMstack principles (decoupled frontend/backend, CDN delivery, API-based functionality).

## Complexity Tracking

No constitutional violations - all complexity is justified by requirements.
├── models/
├── services/
├── cli/
└── lib/

tests/
├── contract/
├── integration/
└── unit/

# [REMOVE IF UNUSED] Option 2: Web application (when "frontend" + "backend" detected)

backend/
├── src/
│ ├── models/
│ ├── services/
│ └── api/
└── tests/

frontend/
├── src/
│ ├── components/
│ ├── pages/
│ └── services/
└── tests/

# [REMOVE IF UNUSED] Option 3: Mobile + API (when "iOS/Android" detected)

api/
└── [same as backend above]

ios/ or android/
└── [platform-specific structure: feature modules, UI flows, platform tests]

```

**Structure Decision**: [Document the selected structure and reference the real
directories captured above]

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation                  | Why Needed         | Simpler Alternative Rejected Because |
| -------------------------- | ------------------ | ------------------------------------ |
| [e.g., 4th project]        | [current need]     | [why 3 projects insufficient]        |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient]  |

```

```
