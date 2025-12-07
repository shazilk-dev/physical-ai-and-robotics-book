# Tasks: Physical AI & Humanoid Robotics Educational Book

**Input**: Design documents from `/specs/001-physical-ai-book/`  
**Prerequisites**: plan.md ✅, spec.md ✅, research.md ✅, data-model.md ✅, contracts/ ✅

**Tests**: NOT included - specification does not require automated testing due to hackathon time constraints. Manual testing and validation against constitutional quality gates will be performed.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story. Each user story can be delivered as an independent increment.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4, US5)
- Include exact file paths in descriptions

## Path Conventions

Based on plan.md project structure:

- **Frontend**: `docs/` (Docusaurus project root)
- **Backend**: `api/` (FastAPI project root)
- **Content**: `docs/docs/chapters/` (Markdown chapter files)
- **Components**: `docs/src/components/` (React components)
- **Specs**: `specs/001-physical-ai-book/` (Design documents)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize project structure, install dependencies, configure development environment

**Time Estimate**: 1-2 hours

- [ ] T001 Initialize Docusaurus project in docs/ with preset-classic template
- [ ] T002 [P] Install Docusaurus dependencies: @docusaurus/theme-mermaid, docusaurus-theme-search-algolia in docs/package.json
- [ ] T003 [P] Initialize FastAPI project structure in api/ with main.py, requirements.txt, .env.template
- [ ] T004 [P] Install FastAPI dependencies: fastapi==0.115.5, uvicorn[standard]==0.32.1, qdrant-client==1.12.1, openai==1.57.2, pydantic==2.10.3, python-dotenv==1.0.1 in api/requirements.txt
- [ ] T005 Configure Docusaurus theme in docs/docusaurus.config.ts: enable dark mode, Mermaid diagrams, Prism syntax highlighting
- [ ] T006 [P] Create docs/docs/chapters/ directory structure for 10 chapters
- [ ] T007 [P] Create api/models.py for Pydantic models (RAGQueryRequest, RAGQueryResponse, SourceCitation)
- [ ] T008 [P] Create api/config.py for environment variable management (OpenAI keys, Qdrant URL)
- [ ] T009 [P] Create docs/src/components/ directory for custom React components
- [ ] T010 Create docs/static/img/ directory for images and diagrams
- [ ] T011 [P] Configure CORS middleware in api/main.py to allow requests from localhost:3000

**Checkpoint**: Project structure initialized, dependencies installed, development servers can start

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**Time Estimate**: 2-3 hours

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T012 Implement Qdrant client connection logic in api/rag.py with error handling
- [ ] T013 [P] Implement OpenAI embedding generation function in api/rag.py using text-embedding-3-small model
- [ ] T014 [P] Implement OpenAI chat completion function in api/rag.py using gpt-4o-mini model
- [ ] T015 Create Qdrant collection "physical_ai_book" with 1536-dimension vectors in api/scripts/setup_qdrant.py
- [ ] T016 [P] Implement chunking logic in api/scripts/chunk_content.py (500 tokens per chunk, 50-token overlap)
- [ ] T017 Create homepage layout in docs/src/pages/index.tsx with hero section and book introduction
- [ ] T018 [P] Configure sidebar navigation structure in docs/sidebars.ts for 10 chapters
- [ ] T019 [P] Create custom CSS theme in docs/src/css/custom.css for dark mode and brand colors
- [ ] T020 Implement health check endpoint GET /health in api/main.py with OpenAI and Qdrant connectivity checks

**Checkpoint**: Foundation ready - Docusaurus renders pages, FastAPI serves endpoints, Qdrant connection works

---

## Phase 3: User Story 1 - Read Comprehensive Book Content (Priority: P1) 🎯 MVP

**Goal**: Deliver all 10 chapters of educational content (66 pages total) with diagrams, tables, code examples, and citations per MASTER-BLUEPRINT.md specification

**Independent Test**: Navigate to localhost:3000, view all 10 chapters in sidebar, click through each chapter verifying 3-4 subchapters, 5-8 pages rendered, ≥1 Mermaid diagram, comparison tables with real data, code examples syntax-highlighted, all facts cited

**Time Estimate**: 20-30 hours (content creation is the most time-intensive phase)

### Priority Content: Critical Path Chapters (1, 3, 6)

- [ ] T021 [P] [US1] Write Chapter 1 (Foundations of Physical AI) in docs/docs/chapters/chapter-01-foundations.md - 6 pages with 4 subchapters (1.1 What is Physical AI, 1.2 Three Eras of Robotics, 1.3 Core Enabling Technologies, 1.4 Market Inflection Point)
- [ ] T022 [US1] Add Table 1.1 "Humanoid Robot Specifications 2025" to Chapter 1 with Tesla Optimus Gen 2, Figure 02, Apptronik Apollo, 1X NEO, Unitree H1 specifications (height, DOF, runtime, price)
- [ ] T023 [US1] Create Mermaid Diagram 1.1 "Physical AI Ecosystem" in Chapter 1 showing convergence of AI, Robotics, IoT → Embodied Intelligence
- [ ] T024 [US1] Add Timeline graphic to Chapter 1 section 1.2 showing three eras (1960s-1990s Symbolic AI, 2000s-2010s Statistical Learning, 2020s+ Foundation Models)
- [ ] T025 [US1] Add citations to Chapter 1 for market data (Grand View Research), technical specs (manufacturer websites), academic references (embodied cognition papers)
- [ ] T026 [P] [US1] Write Chapter 3 (Edge Compute & Perception Systems) in docs/docs/chapters/chapter-03-edge-compute.md - 8 pages with 4 subchapters (3.1 Real-Time Requirements, 3.2 Sensor Modalities & Fusion, 3.3 High-Performance Edge Platforms, 3.4 Vision Transformers at the Edge)
- [ ] T027 [US1] Add Table 3.1 "Edge vs Cloud Compute" comparison to Chapter 3 with latency, bandwidth, privacy, cost, offline capability columns
- [ ] T028 [US1] Create Mermaid Diagram 3.1 "Sensor Fusion Architecture" in Chapter 3 showing IMU, LiDAR, Cameras → Kalman Filter → State Estimation
- [ ] T029 [US1] Create Mermaid Diagram 3.2 "NVIDIA Jetson Thor Architecture" in Chapter 3 showing Blackwell GPU, Grace CPU, 2070 TFLOPS compute
- [ ] T030 [US1] Add Python code example "Complementary Filter for Sensor Fusion" to Chapter 3 section 3.2 (10-15 lines, syntax-highlighted)
- [ ] T031 [US1] Add citations to Chapter 3 for NVIDIA specs, sensor fusion papers, edge computing benchmarks
- [ ] T032 [P] [US1] Write Chapter 6 (Generative Physical AI & Learning) in docs/docs/chapters/chapter-06-generative-ai.md - 7 pages with 4 subchapters (6.1 From Language Models to Behavior Models, 6.2 Vision-Language-Action Models, 6.3 Training Paradigms, 6.4 Simulation-to-Reality Transfer)
- [ ] T033 [US1] Create Mermaid Diagram 6.1 "VLA Training Pipeline" in Chapter 6 showing data collection → pretraining → fine-tuning → deployment flow
- [ ] T034 [US1] Add Python code example "Reinforcement Learning Policy Training" to Chapter 6 section 6.3 (20-25 lines with PPO algorithm)
- [ ] T035 [US1] Add Python code example "Imitation Learning from Demonstrations" to Chapter 6 section 6.3 (15-20 lines with behavioral cloning)
- [ ] T036 [US1] Add citations to Chapter 6 for VLA papers (RT-1, RT-2, PaLM-E), RL frameworks (Stable Baselines3), sim2real research

### Core Technical Content: Chapters 2, 4, 5

- [ ] T037 [P] [US1] Write Chapter 2 (Mechanical Design & Kinematics) in docs/docs/chapters/chapter-02-mechanical-design.md - 7 pages with 4 subchapters (2.1 Humanoid Form Factor, 2.2 Actuator Technologies, 2.3 Forward & Inverse Kinematics, 2.4 Structural Materials)
- [ ] T038 [US1] Add Table 2.1 "Actuator Comparison" to Chapter 2 comparing electric motors, hydraulic actuators, series elastic actuators (torque density, efficiency, cost, control complexity)
- [ ] T039 [US1] Create Mermaid Diagram 2.1 "Humanoid Kinematic Chain" in Chapter 2 showing joints from torso → shoulder → elbow → wrist
- [ ] T040 [US1] Add C++ code example "Inverse Kinematics Solver" to Chapter 2 section 2.3 (25-30 lines using Eigen library)
- [ ] T041 [US1] Add citations to Chapter 2 for actuator specs, kinematics textbooks, materials science papers
- [ ] T042 [P] [US1] Write Chapter 4 (Robotics Software Ecosystem) in docs/docs/chapters/chapter-04-ros2.md - 6 pages with 4 subchapters (4.1 Why ROS 2, 4.2 Core Concepts, 4.3 Real-Time Performance, 4.4 Integration with AI Frameworks)
- [ ] T043 [US1] Add Table 4.1 "ROS 2 vs ROS 1 Comparison" to Chapter 4 (real-time support, security, multi-robot, DDS middleware)
- [ ] T044 [US1] Create Mermaid Diagram 4.1 "ROS 2 Node Graph" in Chapter 4 showing publisher/subscriber pattern with nodes and topics
- [ ] T045 [US1] Add Python code example "ROS 2 Publisher Node" to Chapter 4 section 4.2 (15-20 lines with rclpy)
- [ ] T046 [US1] Add citations to Chapter 4 for ROS 2 documentation, DDS standards, real-time Linux papers
- [ ] T047 [P] [US1] Write Chapter 5 (Whole-Body Control & Locomotion) in docs/docs/chapters/chapter-05-control-locomotion.md - 8 pages with 4 subchapters (5.1 Bipedal Locomotion Challenges, 5.2 Model Predictive Control, 5.3 Zero-Moment Point, 5.4 Learned Locomotion Policies)
- [ ] T048 [US1] Create Mermaid Diagram 5.1 "MPC Control Loop" in Chapter 5 showing state estimation → optimization → actuation → feedback
- [ ] T049 [US1] Add Python code example "ZMP Calculation" to Chapter 5 section 5.3 (20-25 lines with NumPy)
- [ ] T050 [US1] Add C++ code example "MPC Trajectory Optimization" to Chapter 5 section 5.2 (30-35 lines using CasADi)
- [ ] T051 [US1] Add citations to Chapter 5 for bipedal robotics papers, MPC frameworks, locomotion research

### Context & Application Chapters: 7, 8, 9, 10

- [ ] T052 [P] [US1] Write Chapter 7 (Deployment & Optimization) in docs/docs/chapters/chapter-07-deployment.md - 6 pages with 4 subchapters (7.1 Hardware Platform Selection, 7.2 Model Optimization, 7.3 Safety & Fail-Safes, 7.4 OTA Updates)
- [ ] T053 [US1] Add Table 7.1 "Edge Platform Comparison" to Chapter 7 (Jetson Orin, Jetson Thor, Qualcomm RB6, Intel NUC specs)
- [ ] T054 [US1] Create Mermaid Diagram 7.1 "Safety Architecture" in Chapter 7 showing emergency stop chain and redundancy systems
- [ ] T055 [US1] Add citations to Chapter 7 for deployment best practices, model quantization papers, safety standards (ISO 13482)
- [ ] T056 [P] [US1] Write Chapter 8 (Commercial Landscape & Market Dynamics) in docs/docs/chapters/chapter-08-market-dynamics.md - 7 pages with 4 subchapters (8.1 Market Size & Growth, 8.2 Key Players, 8.3 Use Cases, 8.4 Economic Viability)
- [ ] T057 [US1] Add Table 8.1 "Humanoid Robot Manufacturers 2025" to Chapter 8 (Tesla, Figure AI, Apptronik, 1X, Unitree, Sanctuary AI - funding, production status, target markets)
- [ ] T058 [US1] Create Mermaid Diagram 8.1 "Market Segmentation" in Chapter 8 showing manufacturing, logistics, healthcare, retail, consumer verticals
- [ ] T059 [US1] Add citations to Chapter 8 for market reports (Grand View Research, IDC), company financials, industry analyst reports
- [ ] T060 [P] [US1] Write Chapter 9 (Applications & Real-World Case Studies) in docs/docs/chapters/chapter-09-case-studies.md - 6 pages with 4 subchapters (9.1 Manufacturing, 9.2 Logistics & Warehousing, 9.3 Healthcare & Elder Care, 9.4 Research Platforms)
- [ ] T061 [US1] Add Table 9.1 "Case Study Comparison" to Chapter 9 (Boston Dynamics in manufacturing, Figure AI in BMW, Apollo in disaster response - ROI, deployment timeline, outcomes)
- [ ] T062 [US1] Add citations to Chapter 9 for case study sources, deployment data, ROI analyses
- [ ] T063 [P] [US1] Write Chapter 10 (Future Directions & Getting Started) in docs/docs/chapters/chapter-10-future.md - 5 pages with 3 subchapters (10.1 Research Frontiers, 10.2 Ethical Considerations, 10.3 How to Get Started)
- [ ] T064 [US1] Create Mermaid Diagram 10.1 "Learning Roadmap" in Chapter 10 showing progression from foundations → ROS 2 → simulation → hardware
- [ ] T065 [US1] Add resources list to Chapter 10 section 10.3 (online courses, simulation tools, hardware platforms, communities)
- [ ] T066 [US1] Add citations to Chapter 10 for future trends research, ethics papers, educational resources

### Content Validation & Polish

- [ ] T067 [US1] Review all 10 chapters against constitutional checklist: 3-4 subchapters, 5-8 pages, ≥1 diagram, citations present
- [ ] T068 [US1] Verify all Mermaid diagrams render correctly in both light and dark modes in Docusaurus
- [ ] T069 [US1] Verify all code examples have proper syntax highlighting (Python, C++, bash) in Docusaurus Prism config
- [ ] T070 [US1] Run spell check and grammar check across all chapters
- [ ] T071 [US1] Verify all citations are clickable links with 2025-current accessed dates
- [ ] T072 [US1] Calculate page count for each chapter using Docusaurus build and validate 5-8 pages per chapter
- [ ] T073 [US1] Test mobile responsiveness for all chapters (375px width minimum)
- [ ] T074 [US1] Verify WCAG AA compliance: alt text for all diagrams, heading hierarchy (H1 → H2 → H3), color contrast

**Checkpoint**: All 10 chapters (66 pages) complete with diagrams, tables, code, citations. Book content fully functional.

---

## Phase 4: User Story 2 - Query RAG Chatbot About Book Content (Priority: P1) 🎯 MVP

**Goal**: Implement FastAPI backend with RAG endpoints, Qdrant vector search, and OpenAI chat completions to answer user questions about book content with source citations in <3s

**Independent Test**: Start FastAPI server at localhost:8000, use /docs Swagger UI to POST /api/v1/query with {"query": "What is Physical AI?"}, verify response includes answer from Chapter 1 with source citation (Chapter 1, Section 1.1), response time <3 seconds

**Time Estimate**: 8-10 hours

### RAG Backend Implementation

- [ ] T075 [P] [US2] Implement POST /api/v1/query endpoint in api/main.py with RAGQueryRequest schema validation
- [ ] T076 [P] [US2] Implement POST /api/v1/query-selection endpoint in api/main.py for answering questions about selected text with RAGSelectionQueryRequest schema
- [ ] T077 [US2] Implement query embedding generation in api/rag.py: call OpenAI text-embedding-3-small with query text, return 1536-dim vector
- [ ] T078 [US2] Implement Qdrant similarity search in api/rag.py: query "physical_ai_book" collection with embedding vector, retrieve top 5 chunks by cosine similarity
- [ ] T079 [US2] Implement context assembly in api/rag.py: concatenate retrieved chunks with section metadata into prompt context (max 2000 tokens)
- [ ] T080 [US2] Implement chat completion in api/rag.py: call OpenAI gpt-4o-mini with system prompt + context + user query, return answer with reasoning
- [ ] T081 [US2] Implement source citation extraction in api/rag.py: parse retrieved chunk metadata (chapter, section, page_range) into SourceCitation objects
- [ ] T082 [US2] Implement error handling in api/main.py: catch OpenAI API errors (rate limits, timeouts), Qdrant connection errors, return appropriate HTTP status codes (400, 500, 503)
- [ ] T083 [US2] Implement response time logging in api/main.py: measure processing_time_ms from query to response, include in RAGQueryResponse
- [ ] T084 [US2] Add input validation in api/main.py: query length 5-500 chars, selected_text length 10-2000 chars, return 400 error if invalid

### Content Indexing

- [ ] T085 [US2] Create indexing script api/scripts/index_content.py: read all Markdown files from docs/docs/chapters/
- [ ] T086 [US2] Implement Markdown parsing in api/scripts/index_content.py: extract chapter number, section IDs, content text, remove frontmatter
- [ ] T087 [US2] Implement chunking in api/scripts/index_content.py: split each chapter into 500-token chunks with 50-token overlap using tiktoken library
- [ ] T088 [US2] Implement batch embedding in api/scripts/index_content.py: call OpenAI API to embed all chunks (cost: ~$1.06 for 53K tokens)
- [ ] T089 [US2] Implement Qdrant upload in api/scripts/index_content.py: insert embeddings + metadata (chapter, section, page_range, chunk_index) into "physical_ai_book" collection
- [ ] T090 [US2] Add progress logging in api/scripts/index_content.py: show "Processing Chapter X... Y chunks embedded... Z KB used"
- [ ] T091 [US2] Run indexing script after all chapters complete: python api/scripts/index_content.py (verify ~110 documents indexed, ~730 KB Qdrant storage)

### Testing & Validation

- [ ] T092 [US2] Test health check endpoint: GET /health returns 200 with OpenAI and Qdrant status "healthy"
- [ ] T093 [US2] Test basic RAG query: POST /api/v1/query with "What is Physical AI?" returns answer from Chapter 1 with citation
- [ ] T094 [US2] Test technical query: POST /api/v1/query with "How does sensor fusion work?" returns answer from Chapter 3 with code example reference
- [ ] T095 [US2] Test selected text query: POST /api/v1/query-selection with selected_text "NVIDIA Jetson Thor" and query "What are the specs?" returns targeted answer
- [ ] T096 [US2] Test out-of-scope query: POST /api/v1/query with "How do I code in Java?" returns "I can only answer questions about Physical AI and Humanoid Robotics"
- [ ] T097 [US2] Test error handling: stop Qdrant service, POST /api/v1/query returns 503 "Service Unavailable" with clear error message
- [ ] T098 [US2] Measure response times: run 20 queries, verify 95th percentile <3 seconds (log processing_time_ms values)
- [ ] T099 [US2] Verify Qdrant capacity: check collection stats in Qdrant dashboard, confirm storage <1GB (expect ~730 KB)

**Checkpoint**: RAG backend fully functional, returns accurate answers with citations in <3s, all ~110 chunks indexed in Qdrant

---

## Phase 5: User Story 5 - Chat Widget Frontend (Priority: P2)

**Goal**: Create React ChatWidget component integrated into Docusaurus that sends queries to FastAPI backend and displays responses with citations

**Independent Test**: Open localhost:3000, click floating chat button (bottom-right), type "What is Physical AI?" in chat input, verify response appears with citation links that scroll to relevant chapter sections

**Time Estimate**: 6-8 hours

### Chat Widget Implementation

- [ ] T100 [P] [US5] Create ChatWidget React component in docs/src/components/ChatWidget/index.tsx with state management for messages array
- [ ] T101 [US5] Implement chat UI in ChatWidget: message list display, text input field, send button, typing indicator, floating action button to open/close
- [ ] T102 [US5] Implement API integration in ChatWidget: fetch POST to http://localhost:8000/api/v1/query with query text, handle JSON response
- [ ] T103 [US5] Implement citation rendering in ChatWidget: parse SourceCitation objects, create clickable links to chapter sections (e.g., "/chapters/chapter-01-foundations#section-11")
- [ ] T104 [US5] Implement selected text feature in ChatWidget: detect text selection on page, show "Ask about selected text" button, call /api/v1/query-selection endpoint
- [ ] T105 [US5] Create ChatWidget styles in docs/src/components/ChatWidget/styles.module.css: dark mode support, mobile responsive (min 375px width), z-index for floating button
- [ ] T106 [US5] Implement error handling in ChatWidget: show user-friendly messages for API errors ("Unable to connect. Please try again."), network timeouts, rate limits
- [ ] T107 [US5] Add loading state in ChatWidget: show typing indicator with animated dots while waiting for response
- [ ] T108 [US5] Integrate ChatWidget into Docusaurus theme: create docs/src/theme/Root.tsx wrapper, render ChatWidget on all pages
- [ ] T109 [US5] Configure CORS in Docusaurus: update docs/docusaurus.config.ts to allow API calls to localhost:8000 during development

### Testing & Polish

- [ ] T110 [US5] Test chat widget basic functionality: open widget, send query, verify response displays with citation
- [ ] T111 [US5] Test citation links: click citation in response, verify page scrolls to correct chapter section
- [ ] T112 [US5] Test selected text feature: highlight text on page, click "Ask about selected text", verify contextual answer
- [ ] T113 [US5] Test dark mode: toggle dark mode in Docusaurus, verify chat widget colors adapt correctly
- [ ] T114 [US5] Test mobile responsiveness: open on 375px width device, verify chat widget usable with touch input
- [ ] T115 [US5] Test error scenarios: disconnect API server, verify error message displays, attempt to send query during loading state
- [ ] T116 [US5] Test multi-turn conversation: send 3-4 queries in sequence, verify message history persists, scroll works correctly

**Checkpoint**: Chat widget fully functional, integrated into all pages, handles queries and displays citations correctly

---

## Phase 6: User Story 3 - Navigate and Search Book Efficiently (Priority: P2)

**Goal**: Enhance Docusaurus with sidebar navigation, table-of-contents within pages, and Algolia DocSearch for full-text search across all chapters

**Independent Test**: Open localhost:3000, use sidebar to navigate between chapters, click table-of-contents links to jump to subchapters, use search bar to find "Jetson Thor" and verify results link to Chapter 3

**Time Estimate**: 3-4 hours

### Navigation & Search Implementation

- [ ] T117 [P] [US3] Configure sidebar navigation in docs/sidebars.ts: define structure with 10 chapters, each with 3-4 subchapter links
- [ ] T118 [US3] Enable table-of-contents component in docs/docusaurus.config.ts: configure minHeadingLevel: 2, maxHeadingLevel: 3
- [ ] T119 [US3] Add smooth scrolling to docs/src/css/custom.css: scroll-behavior: smooth for anchor links
- [ ] T120 [P] [US3] Configure Algolia DocSearch in docs/docusaurus.config.ts: add Algolia app ID, API key, index name (request from Algolia for open-source project)
- [ ] T121 [US3] Add search bar to navbar in docs/docusaurus.config.ts: configure navbar items with type: 'search'
- [ ] T122 [US3] Verify navigation: test sidebar links navigate without full page reload (client-side routing), verify active chapter highlighted
- [ ] T123 [US3] Verify table-of-contents: test TOC links scroll to correct subchapter headings, verify TOC highlights current section on scroll
- [ ] T124 [US3] Verify search: test Algolia search finds "Jetson Thor" in Chapter 3, "Physical AI" in Chapter 1, verify preview snippets display

**Checkpoint**: Navigation smooth and intuitive, search finds content across all chapters

---

## Phase 7: User Story 4 - Access Book from Any Device/Environment (Priority: P2)

**Goal**: Optimize Docusaurus build for performance (Lighthouse ≥90), ensure mobile responsiveness, validate WCAG AA accessibility compliance

**Independent Test**: Run Lighthouse audit on localhost:3000, verify Performance ≥90, Accessibility ≥90, run axe DevTools, verify no critical accessibility violations, test on mobile device (375px width)

**Time Estimate**: 4-5 hours

### Performance Optimization

- [ ] T125 [P] [US4] Enable code splitting in docs/docusaurus.config.ts: configure webpack to split by route
- [ ] T126 [P] [US4] Optimize images in docs/static/img/: convert PNG to WebP format, add responsive image sizes
- [ ] T127 [US4] Configure bundle analyzer in docs/package.json: add script to visualize bundle size, target <300KB gzipped per page
- [ ] T128 [US4] Lazy load Mermaid diagrams in docs/docusaurus.config.ts: configure mermaid plugin with lazy: true
- [ ] T129 [US4] Add preconnect hints in docs/docusaurus.config.ts: add <link rel="preconnect"> for OpenAI API, Qdrant domain
- [ ] T130 [US4] Run Lighthouse audit on homepage: verify FCP <1.5s, TTI <2s, Performance score ≥90
- [ ] T131 [US4] Run Lighthouse audit on Chapter 1 page: verify same performance targets met with content and diagrams

### Mobile Responsiveness

- [ ] T132 [US4] Test mobile layout on 375px width: verify all chapters readable, no horizontal scroll, touch targets ≥44px
- [ ] T133 [US4] Test tablet layout on 768px width: verify sidebar collapses to hamburger menu, content full-width
- [ ] T134 [US4] Test landscape orientation on mobile: verify layout adapts correctly, chat widget remains accessible
- [ ] T135 [US4] Optimize mobile bundle size: verify mobile-specific code splitting reduces initial load

### Accessibility Validation

- [ ] T136 [US4] Run axe DevTools on homepage: fix any critical or serious accessibility violations (color contrast, keyboard navigation)
- [ ] T137 [US4] Run axe DevTools on Chapter 1: verify all images have alt text, headings hierarchical (H1 → H2 → H3), links have descriptive text
- [ ] T138 [US4] Test keyboard navigation: verify all interactive elements (nav links, chat widget, code copy buttons) accessible via Tab key
- [ ] T139 [US4] Test screen reader (NVDA/JAWS): verify all content announced correctly, Mermaid diagrams have aria-labels
- [ ] T140 [US4] Verify WCAG AA color contrast: check all text has ≥4.5:1 contrast ratio in both light and dark modes

**Checkpoint**: Performance optimized (Lighthouse ≥90), mobile responsive (375px+), WCAG AA compliant

---

## Phase 8: User Story 7 - Deploy and Maintain Book Infrastructure (Priority: P3)

**Goal**: Configure GitHub Actions CI/CD for Docusaurus deployment to GitHub Pages, deploy FastAPI backend to Render free tier, run final indexing to production Qdrant, validate end-to-end system

**Independent Test**: Push commit to main branch, verify GitHub Actions builds Docusaurus and deploys to GitHub Pages (https://[username].github.io/physical-ai-book/), verify Render deploys FastAPI backend (https://physical-ai-api.onrender.com), test production chatbot with live query

**Time Estimate**: 5-6 hours

### GitHub Actions CI/CD

- [ ] T141 [P] [US7] Create GitHub Actions workflow .github/workflows/deploy-docs.yml: build Docusaurus on Node.js 18.x, deploy to gh-pages branch
- [ ] T142 [US7] Configure GitHub Pages in repository settings: set source to gh-pages branch, custom domain (optional)
- [ ] T143 [US7] Add build optimization to deploy-docs.yml: cache node_modules, parallel builds if multiple languages
- [ ] T144 [US7] Test deployment workflow: push commit to main, verify workflow runs successfully, site live within 5 minutes

### Render Backend Deployment

- [ ] T145 [P] [US7] Create render.yaml configuration: define web service for FastAPI with Python 3.11, buildCommand: pip install -r requirements.txt, startCommand: uvicorn main:app --host 0.0.0.0 --port $PORT
- [ ] T146 [US7] Connect GitHub repository to Render: authorize Render app, select physical-ai-book repo, auto-deploy on push to main
- [ ] T147 [US7] Configure environment variables in Render dashboard: add OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, QDRANT_COLLECTION_NAME
- [ ] T148 [US7] Deploy FastAPI backend to Render: trigger manual deploy, verify service starts successfully, health check returns 200
- [ ] T149 [US7] Update CORS configuration in api/main.py: allow requests from production domain (https://[username].github.io), not just localhost

### Production Indexing

- [ ] T150 [US7] Update Qdrant configuration in api/config.py: use production Qdrant Cloud cluster URL (not local Docker)
- [ ] T151 [US7] Run production indexing: execute python api/scripts/index_content.py against production Qdrant, verify ~110 documents uploaded
- [ ] T152 [US7] Verify production Qdrant storage: check dashboard, confirm ~730 KB used (well under 1GB limit)
- [ ] T153 [US7] Update frontend API URL: change ChatWidget fetch URL from localhost:8000 to production Render URL (https://physical-ai-api.onrender.com)

### End-to-End Testing

- [ ] T154 [US7] Test production homepage: visit https://[username].github.io/physical-ai-book/, verify loads in <2s, hero section displays
- [ ] T155 [US7] Test production chatbot: open chat widget on production site, ask "What is Physical AI?", verify response with citation in <3s
- [ ] T156 [US7] Test production navigation: click through all 10 chapters, verify sidebar works, TOC scrolls correctly
- [ ] T157 [US7] Test production search: search for "Jetson Thor", verify Algolia returns results from Chapter 3
- [ ] T158 [US7] Test production mobile: open site on mobile device, verify responsive layout, chat widget works on touch
- [ ] T159 [US7] Test production performance: run Lighthouse on production URL, verify Performance ≥90, Accessibility ≥90
- [ ] T160 [US7] Monitor production costs: check OpenAI API usage dashboard, verify <$10/month, check Render dashboard for RAM usage <512MB

### Final Polish

- [ ] T161 [US7] Add README.md to repository root: project description, setup instructions (link to quickstart.md), deployment status badges
- [ ] T162 [US7] Create CONTRIBUTING.md: guidelines for adding new chapters, content standards (constitutional requirements), PR review process
- [ ] T163 [US7] Verify all repository documentation: update specs/001-physical-ai-book/quickstart.md with production URLs, deployment commands
- [ ] T164 [US7] Create demo video: record 2-3 minute walkthrough showing book navigation, chatbot query with citation, mobile responsiveness
- [ ] T165 [US7] Prepare hackathon submission: compile project description, technical stack, key features, demo URL, GitHub repo URL

**Checkpoint**: Book deployed to production (GitHub Pages + Render), chatbot functional with <3s responses, all quality gates passed, ready for hackathon submission

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies - can start immediately
- **Phase 2 (Foundational)**: Depends on Phase 1 completion - BLOCKS all user stories
- **Phase 3 (US1 - Content)**: Depends on Phase 2 completion - can proceed independently
- **Phase 4 (US2 - RAG Backend)**: Depends on Phase 2 completion - can proceed independently of US1 (but indexing in T091 requires US1 chapters complete)
- **Phase 5 (US5 - Chat Widget)**: Depends on Phase 4 completion (needs RAG backend endpoints)
- **Phase 6 (US3 - Navigation)**: Depends on Phase 3 completion (needs chapters to navigate)
- **Phase 7 (US4 - Performance)**: Depends on Phase 3, 5, 6 completion (optimizes existing content)
- **Phase 8 (US7 - Deployment)**: Depends on all previous phases completion (deploys complete system)

### User Story Dependencies

- **US1 (Content)**: Independent - can start after Foundational (Phase 2)
- **US2 (RAG Backend)**: Independent - can start after Foundational (Phase 2), but T091 (indexing) requires US1 chapters complete
- **US5 (Chat Widget)**: Depends on US2 (needs backend API)
- **US3 (Navigation)**: Depends on US1 (needs chapters)
- **US4 (Performance)**: Depends on US1, US5 (optimizes content and chat)
- **US7 (Deployment)**: Depends on all user stories (deploys complete system)

### Critical Path

**Minimum Viable Product (MVP)**: US1 (Content) + US2 (RAG Backend) + US5 (Chat Widget)

1. Phase 1 (Setup) → 1-2 hours
2. Phase 2 (Foundational) → 2-3 hours
3. Phase 3 (US1 - Content) → 20-30 hours **[CRITICAL PATH - Longest Duration]**
4. Phase 4 (US2 - RAG Backend) → 8-10 hours (can partially overlap with Phase 3)
5. Phase 5 (US5 - Chat Widget) → 6-8 hours
6. Phase 8 (US7 - Deployment) → 5-6 hours

**Total MVP Time Estimate**: 42-59 hours

**Full Feature Set**: Add US3 (Navigation) + US4 (Performance) = +7-9 hours → **49-68 hours total**

### Parallel Opportunities

**Within Phase 1 (Setup)**:

- T002, T003, T004 (dependency installation) can run in parallel
- T007, T008, T009, T010, T011 (directory/file creation) can run in parallel

**Within Phase 2 (Foundational)**:

- T013, T014 (OpenAI functions) can run in parallel with T012 (Qdrant setup)
- T017, T018, T019 (frontend setup) can run in parallel with T012-T016 (backend setup)

**Within Phase 3 (US1 - Content)**:

- T021-T025 (Chapter 1), T026-T031 (Chapter 3), T032-T036 (Chapter 6) can be written in parallel by different authors
- T037-T041 (Chapter 2), T042-T046 (Chapter 4), T047-T051 (Chapter 5) can be written in parallel
- T052-T055 (Chapter 7), T056-T059 (Chapter 8), T060-T062 (Chapter 9), T063-T066 (Chapter 10) can be written in parallel

**Phase 3 + Phase 4 Overlap**:

- Phase 4 (RAG Backend) tasks T075-T084 can start while Phase 3 (Content) is in progress
- Only T085-T091 (indexing) requires Phase 3 completion

**Within Phase 4 (US2 - RAG Backend)**:

- T075, T076 (endpoint definitions) can run in parallel
- T077, T078, T079, T080, T081 (RAG logic) can be developed in parallel by function

**Within Phase 5 (US5 - Chat Widget)**:

- T100 (component structure) can run in parallel with T105 (CSS styles)

**Within Phase 7 (US4 - Performance)**:

- T125, T126, T127, T128, T129 (optimization tasks) can run in parallel

**Within Phase 8 (US7 - Deployment)**:

- T141-T144 (GitHub Actions) can run in parallel with T145-T149 (Render setup)

---

## Parallel Example: Phase 3 (Content Creation)

**Scenario**: 3 developers working on content simultaneously

```bash
# Developer 1: Priority Chapters
git checkout -b content/priority-chapters
# Work on T021-T036 (Chapters 1, 3, 6)
git commit -m "Add Chapters 1, 3, 6 with diagrams and citations"
git push origin content/priority-chapters

# Developer 2: Core Technical Chapters
git checkout -b content/core-technical
# Work on T037-T051 (Chapters 2, 4, 5)
git commit -m "Add Chapters 2, 4, 5 with code examples"
git push origin content/core-technical

# Developer 3: Context Chapters
git checkout -b content/context-chapters
# Work on T052-T066 (Chapters 7, 8, 9, 10)
git commit -m "Add Chapters 7, 8, 9, 10 with market data"
git push origin content/context-chapters

# All branches merge independently (no conflicts - different files)
```

---

## Implementation Strategy

### MVP-First Approach

**Goal**: Deliver functional book with chatbot in minimum time

**Phase 1 (Setup)**: 1-2 hours - Complete all T001-T011

**Phase 2 (Foundational)**: 2-3 hours - Complete all T012-T020

**Phase 3 (US1 - Content) - MVP Subset**: 12-15 hours

- Complete T021-T036 (Priority Chapters 1, 3, 6) - **18 pages**
- Skip T037-T066 initially (remaining 48 pages can be added post-MVP)

**Phase 4 (US2 - RAG Backend)**: 8-10 hours - Complete all T075-T099

**Phase 5 (US5 - Chat Widget)**: 6-8 hours - Complete all T100-T116

**Phase 8 (US7 - Deployment) - MVP Subset**: 3-4 hours

- Complete T141-T153 (CI/CD and indexing)
- Skip T154-T165 (comprehensive testing and polish can be done post-deployment)

**MVP Total**: 32-42 hours - Delivers 3 chapters + functional chatbot + deployment

### Full Feature Completion

**After MVP Deployed**:

- Add remaining 7 chapters (T037-T066): +12-15 hours
- Add navigation and search (T117-T124): +3-4 hours
- Optimize performance and accessibility (T125-T140): +4-5 hours
- Final testing and polish (T154-T165): +2-3 hours

**Full Feature Total**: 53-69 hours

### Quality Gates

**Before Phase 3 Completion**:

- [ ] All chapters have 3-4 subchapters (constitutional requirement)
- [ ] All chapters have 5-8 pages rendered (constitutional requirement)
- [ ] All chapters have ≥1 Mermaid diagram (constitutional requirement)
- [ ] All technical facts cited with 2025-current sources (constitutional requirement)
- [ ] All code examples syntax-highlighted and tested (constitutional requirement)

**Before Phase 5 Completion**:

- [ ] RAG responses include source citations (constitutional requirement)
- [ ] 95th percentile response time <3s (performance requirement)
- [ ] Error messages user-friendly (UX requirement)

**Before Phase 8 Completion**:

- [ ] Lighthouse Performance ≥90 (performance requirement)
- [ ] Lighthouse Accessibility ≥90 (WCAG AA requirement)
- [ ] Mobile responsive 375px+ (accessibility requirement)
- [ ] Production chatbot functional with citations (functional requirement)
- [ ] Qdrant storage <1GB (infrastructure constraint)
- [ ] OpenAI API cost <$10/month (budget constraint)

---

## Risk Mitigation

**Risk 1: Content creation takes longer than estimated (20-30 hours)**

- **Mitigation**: Implement MVP-first approach with only Chapters 1, 3, 6 (12-15 hours), add remaining chapters post-MVP
- **Contingency**: Use MASTER-BLUEPRINT.md as detailed outline to accelerate writing

**Risk 2: RAG response times exceed 3s target**

- **Mitigation**: Optimize chunking strategy (reduce from 500 to 400 tokens), reduce top_k from 5 to 3 documents retrieved
- **Contingency**: Cache common queries using LRU cache in FastAPI

**Risk 3: Qdrant storage exceeds 1GB free tier**

- **Mitigation**: Current calculation shows 730 KB usage (27% of limit), significant headroom available
- **Contingency**: Reduce chunk overlap from 50 to 25 tokens (~94 chunks vs ~110), or reduce embedding dimensions with PCA

**Risk 4: OpenAI API costs exceed $10/month budget**

- **Mitigation**: Current projection is $1.06 one-time + $0.20/month (2% of budget)
- **Contingency**: Implement rate limiting (10 queries/minute per user), add query cost logging

**Risk 5: GitHub Actions or Render deployment fails**

- **Mitigation**: Test deployment workflows early with minimal content (Chapter 1 only)
- **Contingency**: Manual deployment fallback (build locally, upload to Render)

**Risk 6: Accessibility violations found in final testing**

- **Mitigation**: Run axe DevTools incrementally after each chapter added (T067-T074)
- **Contingency**: Quick fixes for color contrast (CSS variables), alt text (manual addition), heading hierarchy (Markdown adjustment)

---

## Success Criteria

**MVP Success** (Phases 1-5, 8 MVP subset):

- [ ] 3 priority chapters (1, 3, 6) published with diagrams, tables, code examples, citations
- [ ] Chatbot functional with <3s responses and source citations
- [ ] Deployed to GitHub Pages + Render
- [ ] Qdrant storage <1GB, OpenAI cost <$10/month
- [ ] Total time <45 hours

**Full Feature Success** (All phases):

- [ ] All 10 chapters (66 pages) published per MASTER-BLUEPRINT.md
- [ ] All constitutional requirements met (3-4 subchapters, 5-8 pages, ≥1 diagram per chapter)
- [ ] Chatbot with <3s responses, source citations, selected text feature
- [ ] Navigation and search functional
- [ ] Lighthouse Performance ≥90, Accessibility ≥90
- [ ] Mobile responsive (375px+), WCAG AA compliant
- [ ] Deployed to production, end-to-end tested
- [ ] Total time <70 hours

---

## Next Steps

1. **Review this task breakdown**: Confirm priorities, time estimates, and parallel opportunities align with project goals
2. **Allocate resources**: Decide on MVP-first vs full feature approach based on hackathon timeline
3. **Start Phase 1**: Initialize project structure (T001-T011) - estimated 1-2 hours
4. **Begin content creation**: Highest priority is T021-T036 (Chapters 1, 3, 6) as they block RAG indexing
5. **Track progress**: Update task checkboxes as work completes, adjust estimates based on actuals
