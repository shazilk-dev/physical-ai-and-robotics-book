# Feature Specification: Physical AI & Humanoid Robotics Educational Book

**Feature Branch**: `001-physical-ai-book`  
**Created**: 2025-12-07  
**Updated**: 2025-12-07 (Module-based restructure)
**Status**: Active Development  
**Input**: User description: "Build code-first, hands-on Physical AI & Humanoid Robotics textbook with 4 progressive modules and integrated RAG chatbot"

## Educational Philosophy

**Core Principle**: Code-first, hands-on learning where students build real systems from Day 1.

**Structure**: 4 Progressive Modules (50 pages total)

- Module 1: ROS 2 Nervous System (12 pages) - Code-first ROS 2, URDF, sensors, Jetson Orin deployment
- Module 2: Core Robot Architecture (14 pages) - Mechanical design, actuators, edge compute, perception stack
- Module 3: NVIDIA Isaac & Simulation (13 pages) - Isaac Sim, Isaac ROS, VSLAM, Nav2, sim-to-real
- Module 4: Vision-Language-Action & Voice (11 pages) - VLA models, Whisper integration, capstone project

**Learning Pattern per Module**: Theory (300-400 words) → Visual (diagram/table) → Code Example (10-20 lines) → 🧪 Lab (link to starter code) → ✅ Checkpoint (what you can do now)

**Hardware Focus**: Jetson Orin Nano/NX edge deployment, Unitree robots as learning platforms, RealSense cameras, Whisper for voice

## User Scenarios & Testing _(mandatory)_

### User Story 1 - Learn Through Progressive Modules with Embedded Labs (Priority: P1) 🎯 MVP

A student begins with Module 1 (ROS 2), completes hands-on labs building real nodes, progresses through Modules 2-3 (architecture, simulation), and culminates in Module 4 with a voice-controlled autonomous robot capstone.

**Why this priority**: This is the core educational experience—progressive, code-first learning with embedded labs. The entire value proposition depends on students building real systems, not just reading theory.

**Independent Test**: Can be fully tested by navigating Module 1 → completing Lab 1 (build heartbeat node) → seeing inline lab references throughout content → progressing to Module 2-4 → accessing capstone starter code.

**Acceptance Scenarios**:

1. **Given** a user visits the homepage, **When** they browse the table of contents, **Then** they see:

   - 4 modules organized sequentially (Module 1: ROS 2 → Module 2: Architecture → Module 3: Isaac → Module 4: VLA)
   - Each module shows chapters (e.g., Module 1 has 1.1, 1.2, 1.3)
   - Clear learning objectives per module
   - Estimated time commitments per module

2. **Given** a user opens Module 1.1 (ROS 2 Fundamentals), **When** they scroll through the content, **Then** they see:

   - Section 1.1.1: Theory content (~300 words on ROS 2 architecture)
   - Mermaid diagram (ROS 2 runtime graph)
   - Code example (12-16 line rclpy heartbeat node)
   - **🧪 Lab 1.1.1** callout with "Build Your First ROS 2 Node" link to `labs/lab01-ros2-basics/`
   - ✅ Checkpoint summary ("You can now create ROS 2 packages and write publishers")

3. **Given** a user clicks "Start Lab 1.1.1" link, **When** they navigate to the lab, **Then** they find:

   - `labs/lab01-ros2-basics/starter/` with actual ROS 2 package (package.xml, setup.py, src/heartbeat_node.py with TODO comments)
   - `labs/lab01-ros2-basics/solutions/` with complete working code
   - `labs/lab01-ros2-basics/README.md` with instructions, learning objectives, and expected output
   - `labs/lab01-ros2-basics/tests/` with pytest assertions to validate their solution

4. **Given** a user completes Module 1 labs, **When** they progress to Module 3.1 (Isaac Sim), **Then** they find:

   - Isaac Sim USD scene starter file in `isaac_assets/scenes/lab04-first-scene/`
   - Step-by-step guide to load scene, spawn robot, run VSLAM
   - Nav2 configuration file linking to prior ROS 2 knowledge from Module 1
   - Performance metrics showing sim vs real-world transfer

5. **Given** a user reaches Module 4.3 (Capstone Project), **When** they review requirements, **Then** they see:

   - Full pipeline spec: Voice command (Whisper) → LLM plan → Nav2 navigation → Isaac ROS perception → object identification → manipulation
   - Starter code in `ros2_packages/voice_to_action/` with Whisper integration stubs
   - Grading rubric with measurable criteria (voice recognition accuracy, navigation success rate, manipulation completion)
   - Demo checklist and video recording requirements

6. **Given** a user reads Module 2.3 (Edge Compute), **When** they review hardware specs, **Then** they find:
   - **Only Jetson Orin Nano (8GB) and Orin NX (16GB)** references (no Thor)
   - Comparison table: Orin Nano vs Orin NX with real specs (TOPS, RAM, power, price)
   - Deployment guide: Flash OS, install ROS 2, run inference on Orin
   - Code example showing TensorRT model optimization for Orin constraints

---

### User Story 2 - Query RAG Chatbot About Module Content (Priority: P1) 🎯 MVP

A reader working through modules asks the chatbot questions like "How do I tune QoS for sensor data?" or "Show me the Whisper integration code" and receives answers grounded in book content with chapter/section citations.

**Why this priority**: The RAG chatbot transforms passive reading into active problem-solving. Students can clarify concepts, debug code examples, and get instant help while working through labs—critical for hackathon requirements.

**Independent Test**: Can be tested by opening chatbot, asking module-specific questions ("What's the difference between Orin Nano and Orin NX?", "How do I integrate Whisper with ROS 2?"), and verifying responses cite correct module sections with accurate technical details.

**Acceptance Scenarios**:

1. **Given** a user is reading Module 1.1.4 (QoS Tuning), **When** they ask chatbot "How do I set QoS for IMU data?", **Then** they receive:

   - Answer extracted from Module 1.1.4 content (Best Effort, Volatile, Keep Last 1)
   - Citation showing "Source: Module 1, Section 1.1.4 - Real-Time Considerations & QoS Tuning"
   - QoS settings table from the module
   - Response delivered in <3 seconds

2. **Given** a user is stuck on Lab 1 (heartbeat node), **When** they ask "How do I create a ROS 2 publisher?", **Then** chatbot:

   - Returns the code example from Section 1.1.2 (rclpy publisher pattern)
   - Explains the pattern with reference to `create_publisher()` method
   - Suggests checking Lab 1 starter code for TODO comments
   - Does NOT provide external ROS 2 documentation (book-only policy)

3. **Given** a user highlights text "Jetson Orin Nano (8GB) capabilities", **When** they click "Ask about selected text", **Then** chatbot:
   - Understands query is scoped to Orin Nano specifications
   - Answers with Module 2.3.1 content (TOPS, power, recommended use cases)
   - Provides comparison with Orin NX if relevant
   - Cites exact section (Module 2, Section 2.3.1)

---

### User Story 3 - Navigate and Search Book Efficiently (Priority: P2)

A user needs to quickly find specific information across all chapters without reading sequentially, using navigation tools and search functionality.

**Why this priority**: Essential for usability and knowledge retrieval, but the book can function without these features. Users could still read linearly, though less efficiently.

**Independent Test**: Can be tested by using the sidebar navigation to jump between chapters, clicking on table-of-contents links within a page, and using the search bar to find keywords like "ROS 2" or "actuators" across all content.

**Acceptance Scenarios**:

1. **Given** a user views the sidebar, **When** they click on "Chapter 5: Whole-Body Control", **Then** they navigate immediately to Chapter 5 without full page reload
2. **Given** a user is reading Chapter 4, **When** they click a "Table of Contents" link within the page, **Then** the page scrolls smoothly to the clicked subchapter (e.g., 4.2 Real-Time Performance)
3. **Given** a user types "Jetson Thor" in the search bar, **When** search executes, **Then** they see:
   - All pages containing "Jetson Thor" listed
   - Preview snippets showing context
   - Ability to click and navigate to exact location
4. **Given** a user searches for "kinematics", **When** results display, **Then** they find matches in both Chapter 2 (Mechanical Design) and Chapter 5 (Whole-Body Control)

---

### User Story 4 - Access Book from Any Device/Environment (Priority: P2)

A user accesses the book from various devices (desktop, mobile, tablet) and network conditions (fast/slow internet) and experiences consistent, fast performance.

**Why this priority**: Accessibility and performance are quality attributes that enhance user experience but don't block core learning functionality.

**Independent Test**: Can be tested by loading the book on multiple devices, throttling network to 4G speeds, running Lighthouse audits, and verifying all metrics meet constitutional requirements (<2s load, WCAG AA compliance).

**Acceptance Scenarios**:

1. **Given** a user on a desktop with fast internet, **When** they load the homepage, **Then** First Contentful Paint (FCP) occurs in <1.5 seconds
2. **Given** a user on a mobile device with 4G connection, **When** they load Chapter 3, **Then** Time to Interactive (TTI) is <2 seconds
3. **Given** a user using a screen reader, **When** they navigate the book, **Then**:
   - All images have descriptive alt text
   - Heading hierarchy is logical (H1 → H2 → H3)
   - Code blocks are announced with language identifier
   - WCAG AA compliance verified
4. **Given** a user loads any page, **When** performance is measured, **Then**:
   - Lighthouse Performance score ≥90
   - Lighthouse Accessibility score ≥90
   - Cumulative Layout Shift (CLS) <0.1
   - Largest Contentful Paint (LCP) <2.5s

---

### User Story 5 - Deploy and Maintain Book Infrastructure (Priority: P3)

A maintainer needs to deploy book updates, monitor system health, and manage infrastructure costs effectively using free-tier services.

**Why this priority**: Critical for project sustainability but not user-facing. Initial deployment can be manual; automation improves efficiency over time.

**Independent Test**: Can be tested by pushing a content update to the repository, verifying GitHub Actions automatically deploys to GitHub Pages, confirming RAG API deploys to Render, and validating Qdrant vector database updates with new content.

**Acceptance Scenarios**:

1. **Given** a maintainer pushes new content to the main branch, **When** GitHub Actions triggers, **Then**:
   - Docusaurus build completes successfully
   - Static site deploys to GitHub Pages
   - New content is live within 5 minutes
2. **Given** a maintainer updates RAG backend code, **When** code is pushed to repository, **Then**:
   - Render automatically rebuilds and deploys the FastAPI application
   - API health check endpoint returns 200 OK
   - Qdrant connection remains functional
3. **Given** a new chapter is added, **When** the RAG indexing process runs, **Then**:
   - Chapter content is embedded and stored in Qdrant
   - Chatbot can answer questions about new chapter
   - Vector database remains within 1GB free tier limit
4. **Given** the book receives 1000 page views per day, **When** costs are calculated, **Then**:
   - GitHub Pages: $0 (free hosting)
   - Render API: $0 (free tier supports traffic)
   - Qdrant: $0 (within 1GB limit)
   - OpenAI API: <$5/month (estimated based on expected query volume)

---

### Edge Cases

- **What happens when a user asks the RAG chatbot a question in a language other than English?**
  - System responds: "This book is available in English only. Please ask your question in English."
- **What happens when the RAG API is down or times out?**

  - Chatbot displays: "Unable to connect to the question-answering service. Please try again in a moment."
  - User can still read all book content normally

- **What happens when a user tries to access the book from an extremely old browser (e.g., IE11)?**

  - Display a banner: "This book requires a modern browser. Please update to Chrome, Firefox, Safari, or Edge for the best experience."
  - Graceful degradation: core content remains readable, but interactive features may not work

- **What happens when Qdrant vector database reaches capacity (>1GB)?**

  - Monitor dashboard alerts maintainer
  - System either: (1) upgrades to paid tier, or (2) optimizes embeddings (reduce dimensions/precision)
  - Chatbot continues functioning with existing data until resolved

- **What happens when a code example in the book contains a syntax error?**

  - Constitution requires all code be tested before publication
  - Validation process catches errors during pre-deployment quality gates
  - If error reaches production: user can report via GitHub issues

- **What happens when a user highlights text that spans multiple paragraphs and asks a question?**

  - RAG system uses the entire selected text as context
  - If selection is too long (>2000 characters), system uses first/last portions and middle summary

- **What happens when multiple users cite the same chapter simultaneously?**
  - Qdrant handles concurrent reads efficiently (no locking needed)
  - Each user receives independent responses
  - No performance degradation expected up to 100 concurrent users

## Requirements _(mandatory)_

### Functional Requirements

#### Content Requirements (Module-Based Structure)

- **FR-001**: System MUST provide 4 complete modules (50 pages total) covering Physical AI progression: Module 1 (ROS 2, 12p) → Module 2 (Architecture, 14p) → Module 3 (Isaac, 13p) → Module 4 (VLA/Whisper, 11p) as specified in MASTER-BLUEPRINT.md
- **FR-002**: Each module MUST contain 2-3 chapters; each chapter MUST contain 3-5 sections following pattern: Theory (300-400 words) → Visual → Code → Lab → Checkpoint
- **FR-003**: System MUST include minimum 8 Mermaid diagrams across modules (ROS 2 graph, kinematic chains, Isaac pipeline, VLA architecture, capstone flow, etc.)
- **FR-004**: System MUST include minimum 8 comparison tables with real 2025-accurate data (QoS settings, actuator comparison, Orin Nano vs NX, simulator selection, Whisper models, etc.)
- **FR-005**: All technical facts MUST be cited with verifiable sources (academic papers, manufacturer specs, ROS 2 docs, NVIDIA Isaac docs)
- **FR-006**: Content MUST follow progressive skill-building from Module 1 (ROS 2 basics) to Module 4 (autonomous voice-driven robot)
- **FR-007**: Code examples MUST be provided as runnable snippets (12-20 lines) in Python for ROS 2 nodes, URDF/xacro, Isaac Sim, Whisper integration
- **FR-008**: Each section MUST include inline lab references (🧪 Lab X.Y.Z) linking to corresponding labs/ directory with starter code

#### Labs & Hands-On Infrastructure Requirements

- **FR-009**: System MUST include complete Lab 1 (ROS 2 basics) with starter/, solutions/, tests/, README.md, and actual ROS 2 package (package.xml, setup.py, src/ with TODO comments)
- **FR-010**: Each lab MUST specify learning objectives, prerequisites, estimated time, and grading criteria
- **FR-011**: System MUST include lab templates for all 4 modules with at least 1 complete lab per module
- **FR-012**: System MUST populate ros2_packages/ with minimum 1 example package (e.g., ros2_packages/humanoid_description/ with URDF)
- **FR-013**: System MUST include Isaac Sim starter scene in isaac_assets/scenes/ with USD file and README
- **FR-014**: System MUST provide Whisper integration starter code in ros2_packages/voice_to_action/ with API stubs and config
- **FR-015**: System MUST include capstone project specification with requirements, rubric, submission checklist, and starter code linking all modules

#### Navigation & UI Requirements

- **FR-016**: System MUST provide homepage with hero section introducing "Code-First Physical AI & Humanoid Robotics"
- **FR-017**: System MUST include sidebar navigation showing 4 modules with expandable chapters and sections
- **FR-018**: Each module page MUST include module overview, learning outcomes, hardware context, and chapter list
- **FR-019**: Each section MUST include inline lab callouts (visually distinct boxes) with "Start Lab →" links
- **FR-020**: System MUST support dark mode toggle that persists user preference
- **FR-021**: System MUST include search functionality across all module content and lab descriptions
- **FR-022**: All navigation MUST work without full page reloads (SPA behavior)

#### RAG Chatbot Requirements

- **FR-023**: System MUST embed a chatbot interface accessible from all module pages
- **FR-024**: Chatbot MUST answer questions using ONLY module content (no external ROS 2 docs, no general robotics knowledge)
- **FR-025**: Chatbot MUST cite sources with module and section numbers (e.g., "Source: Module 1, Section 1.1.4 - QoS Tuning")
- **FR-026**: Chatbot MUST support "answer from selected text" mode for inline help during lab work
- **FR-027**: Chatbot MUST respond to queries in <3 seconds (95th percentile)
- **FR-028**: Chatbot MUST gracefully decline out-of-scope queries: "I can only answer questions about the 4 modules in this book. Try asking about ROS 2, Isaac Sim, or Whisper integration."
- **FR-029**: Chatbot MUST display typing indicator while processing
- **FR-030**: Chatbot MUST show clear error messages if API fails
- **FR-031**: Chatbot MUST handle code-related queries by returning relevant code examples from sections with syntax highlighting

#### Hardware Accuracy Requirements (2025 Standards)

- **FR-032**: System MUST reference ONLY Jetson Orin Nano (8GB) and Orin NX (16GB) for edge compute (NO Thor references)
- **FR-033**: All Orin specifications MUST be accurate: Orin Nano (100 TOPS, 8GB RAM, 10-25W), Orin NX (275 TOPS, 16GB RAM, 15-25W)
- **FR-034**: Workstation GPU recommendations MUST specify RTX 4070 Ti (12GB VRAM) or RTX 3090/4090 (24GB VRAM) for Isaac Sim
- **FR-035**: Sensor references MUST use Intel RealSense D435i/D455, BNO055 IMU, ReSpeaker microphone array
- **FR-036**: Robot platform references MUST use Unitree Go2 (proxy) or Unitree G1 (full humanoid) with accurate pricing (~$2K and ~$16K respectively)

#### Performance Requirements

- **FR-037**: System MUST achieve First Contentful Paint (FCP) <1.5s
- **FR-038**: System MUST achieve Time to Interactive (TTI) <2s on 4G
- **FR-039**: System MUST maintain Lighthouse Performance score ≥90
- **FR-040**: System MUST keep Cumulative Layout Shift (CLS) <0.1
- **FR-041**: System MUST keep gzipped bundle <300KB per page

#### Accessibility Requirements

- **FR-042**: System MUST achieve WCAG AA compliance
- **FR-043**: System MUST maintain Lighthouse Accessibility score ≥90
- **FR-044**: All diagrams (including Mermaid) MUST have descriptive alt text
- **FR-045**: Code blocks MUST be keyboard-navigable
- **FR-046**: Lab callout boxes MUST have sufficient color contrast (4.5:1 text, 3:1 UI)

#### Deployment & Infrastructure Requirements

- **FR-047**: Frontend (Docusaurus) MUST deploy to GitHub Pages via GitHub Actions
- **FR-048**: Backend (FastAPI) MUST deploy to Render free tier (512MB RAM)
- **FR-049**: Vector DB MUST use Qdrant Cloud free tier (1GB limit)
- **FR-050**: Embeddings MUST use OpenAI text-embedding-3-small
- **FR-051**: Chat completion MUST use OpenAI gpt-4o-mini (cost efficiency)
- **FR-052**: All API keys MUST be stored in GitHub Secrets and .env files (never committed)
- **FR-053**: System MUST auto-deploy on push to main branch (frontend + backend)
- **FR-054**: System MUST include seed script to index all module content + labs into Qdrant

### Key Entities _(include if feature involves data)_

#### Module

**Purpose**: Represents a progressive learning unit in the 4-module curriculum

**Attributes**:

- Module number (1-4)
- Title (e.g., "The Robotic Nervous System (ROS 2)")
- Page count (Module 1: 12p, Module 2: 14p, Module 3: 13p, Module 4: 11p)
- Chapters (2-3 per module)
- Learning outcomes (list of skills students will acquire)
- Hardware context (development environment + deployment targets)
- Labs (list of associated labs with IDs)

**Relationships**:

- Contains multiple Chapters
- References multiple Labs
- Prerequisites (Module N requires Module N-1 completion)

---

#### Chapter

**Purpose**: Represents a focused topic within a module (e.g., "ROS 2 Fundamentals" in Module 1)

**Attributes**:

- Chapter number (e.g., 1.1, 1.2, 1.3)
- Title (e.g., "ROS 2 Fundamentals — Code First")
- Page count (target: 3-5 pages per chapter)
- Sections (3-5 per chapter)
- Visual content count (diagrams, tables, code examples)

**Relationships**:

- Contains multiple Subchapters
- Contains multiple VisualContent items
- Referenced by RAGDocument embeddings

#### Subchapter

**Purpose**: Represents a subsection within a chapter covering a specific subtopic

**Attributes**:

- Subchapter identifier (e.g., "1.1", "3.2")
- Title (e.g., "What is Physical AI?")
- Page allocation (e.g., 1.5 pages, 2 pages)
- Content text (markdown)
- Code examples (if applicable)

**Relationships**:

- Belongs to one Chapter
- May contain CodeExample items
- May contain DiagramContent

#### VisualContent

**Purpose**: Represents diagrams, tables, and other visual elements in the book

**Attributes**:

- Type (mermaid_diagram | comparison_table | code_example | image)
- Title/caption
- Source content (mermaid code, markdown table, code snippet)
- Citation (if data-based)

**Relationships**:

- Belongs to one Chapter or Subchapter
- Referenced in RAGDocument for context

#### RAGDocument

**Purpose**: Represents a chunk of book content indexed in the vector database for retrieval

**Attributes**:

- Document ID (unique)
- Content text (chunk from book)
- Embedding vector (1536 dimensions for text-embedding-3-small)
- Source chapter number
- Source section identifier
- Metadata (chapter title, section title, page range)

**Relationships**:

- Maps to one Chapter and Subchapter
- Retrieved by RAGQuery

#### RAGQuery

**Purpose**: Represents a user question asked to the chatbot

**Attributes**:

- Query text (user's question)
- Query embedding (vector representation)
- Selected text context (if using "answer from selection" mode)
- Timestamp

**Relationships**:

- Retrieves multiple RAGDocument items (top-k similar)
- Generates one RAGResponse

#### RAGResponse

**Purpose**: Represents the chatbot's answer to a user query

**Attributes**:

- Response text (generated answer)
- Source citations (list of chapter/section references)
- Confidence score (optional)
- Processing time (milliseconds)

**Relationships**:

- Responds to one RAGQuery
- Cites multiple RAGDocument sources

### Non-Functional Requirements

#### Reliability

- System uptime target: 99.5% (excluding planned maintenance)
- API error rate: <1% of requests
- Graceful degradation: if RAG API fails, book content remains fully accessible

#### Scalability

- Expected traffic: 1000-5000 page views per day initially
- Concurrent user support: 100+ simultaneous readers
- RAG API capacity: 20 queries per minute (Render free tier supports this)

#### Maintainability

- All code MUST follow Docusaurus best practices
- RAG backend MUST use FastAPI with type hints
- Configuration MUST be environment-based (no hardcoded secrets)
- Deployment MUST be automated via CI/CD

#### Security

- HTTPS-only access enforced
- API keys stored securely (GitHub Secrets, environment variables)
- No user data collection (chatbot queries not logged with PII)
- Content Security Policy headers configured

#### Cost Constraints

- Book hosting: $0 (GitHub Pages free tier)
- RAG API hosting: $0 (Render free tier)
- Vector database: $0 (Qdrant free tier, 1GB limit)
- OpenAI API costs: Target <$10/month for embeddings and completions
- Total monthly cost target: <$10

## Out of Scope _(NON-REQUIREMENTS)_

These items are explicitly excluded from the initial implementation due to time constraints. They may be considered for future enhancements:

- **User Authentication**: No user accounts, signup, or login (bonus feature if time allows)
- **Personalized Content**: No customization based on user background/preferences (bonus feature)
- **Comments System**: No ability for users to comment on chapters or discussions
- **Multiple Languages**: Book is English-only (Urdu translation is bonus feature)
- **Offline Mode**: Progressive Web App (PWA) capabilities not required
- **Custom Subagents**: AI agents for specialized tasks (bonus feature if time allows)
- **Analytics Dashboard**: No built-in tracking of user behavior or chapter popularity
- **Content Contributions**: No system for external contributors to suggest edits
- **Mobile Apps**: Native iOS/Android applications not required
- **Video Content**: No embedded videos or multimedia beyond static diagrams
- **Interactive Simulations**: No live ROS 2 or robot simulations in browser
- **Print/PDF Export**: No "export to PDF" functionality
- **Social Sharing**: Beyond basic Open Graph meta tags, no deep social integration

## Success Criteria _(technology-agnostic)_

The feature is considered successful when:

1. **Content Completeness**: All 10 chapters are published with complete content matching the blueprint:

   - Each chapter contains 5-8 pages of educational content
   - All 66 pages total delivered
   - Every chapter includes minimum 1 diagram and appropriate visual content
   - 100% of specified topics (from MASTER-BLUEPRINT.md) covered

2. **Educational Effectiveness**: Readers can learn Physical AI and Humanoid Robotics from beginner to advanced level:

   - A beginner with basic programming knowledge can understand Chapter 1
   - A reader who completes all 10 chapters can explain core concepts, understand technical specifications, and describe implementation approaches
   - Progressive difficulty is maintained (no knowledge gaps between chapters)

3. **RAG Chatbot Functionality**: Users can get accurate answers from book content:

   - Chatbot correctly answers ≥85% of in-scope questions (validated via test set)
   - All responses cite specific chapter/section sources
   - 95th percentile response time <3 seconds
   - Out-of-scope queries are declined gracefully 100% of the time

4. **Technical Performance**: The book delivers a fast, accessible experience:

   - Lighthouse Performance score ≥90 maintained across all pages
   - Lighthouse Accessibility score ≥90 (WCAG AA compliant)
   - Page load time <2 seconds on 4G connections
   - Zero critical console errors in production

5. **User Experience Quality**: Readers find the book easy to navigate and use:

   - Users can find specific information within 3 clicks from any page
   - Dark mode works correctly on 100% of pages
   - Mobile experience is fully functional on devices as small as 375px wide
   - Search returns relevant results for all major topics

6. **Deployment Reliability**: The system deploys automatically and runs stably:

   - GitHub Actions deploy book updates within 5 minutes of merge to main
   - RAG API maintains >99% uptime (excluding initial deployment issues)
   - Zero failed deployments in final week before submission
   - All environment variables and secrets configured correctly

7. **Cost Efficiency**: Infrastructure stays within free tiers:

   - Qdrant vector database usage <1GB (free tier limit)
   - Render API hosting remains within free tier limits
   - OpenAI API costs <$10/month at expected usage levels
   - Total monthly operating cost <$10

8. **Hackathon Compliance**: Project meets all base requirements:
   - Book created using Docusaurus and deployed to GitHub Pages ✓
   - RAG chatbot embedded and functional ✓
   - Chatbot uses OpenAI SDK, FastAPI, Qdrant ✓
   - "Answer from selected text" mode works ✓
   - 90-second demo video created showing all features ✓
   - GitHub repository is public and well-documented ✓

## Assumptions

1. **Content Authority**: MASTER-BLUEPRINT.md is the authoritative source for all chapter structures, topics, and page allocations
2. **Technical Accuracy**: All technical specifications (robot models, compute platforms, market data) are current as of 2025 and will be verified with cited sources
3. **OpenAI API Access**: Project has valid OpenAI API key with sufficient credits for embeddings and completions
4. **Free Tier Sufficiency**: Render free tier provides adequate resources for expected RAG API traffic (<100 concurrent users)
5. **Qdrant Capacity**: 1GB Qdrant free tier is sufficient for embedding all 66 pages of book content (estimated ~200-300 MB after embeddings)
6. **Network Availability**: Target users have internet connectivity (no offline requirements)
7. **Browser Support**: Target users have modern browsers (Chrome, Firefox, Safari, Edge from last 2 years)
8. **Development Environment**: Developer has access to Claude Code and Spec-Kit Plus tooling
9. **Deployment Permissions**: Developer has admin access to GitHub repository for Pages and Actions configuration
10. **Content Quality**: All code examples will be tested and validated before inclusion in book
11. **Citation Availability**: Verifiable sources exist for all technical facts and can be accessed for citation
12. **Time Constraint**: Project completion is time-boxed (hackathon deadline), prioritizing core functionality over bonus features

## Dependencies

### External Dependencies

- **Docusaurus 3.9**: Static site generator framework (must be stable release)
- **OpenAI API**: For embeddings (text-embedding-3-small) and chat completions (gpt-4o-mini)
- **Qdrant Cloud**: Vector database service (free tier account required)
- **Render**: PaaS for hosting FastAPI backend (free tier account required)
- **GitHub Pages**: Static site hosting (enabled in repository settings)
- **GitHub Actions**: CI/CD for automated deployments (enabled in repository)
- **React 18+**: JavaScript framework (dependency of Docusaurus)
- **Mermaid JS**: Diagram rendering library (must be integrated with Docusaurus)
- **FastAPI**: Python web framework for RAG backend
- **Qdrant Python Client**: SDK for vector database operations

### Internal Dependencies

- **Constitution Document**: Defines quality standards, principles, and governance for the project
- **MASTER-BLUEPRINT.md**: Contains complete chapter structure and content specifications
- **Spec-Kit Plus**: Tooling for spec-driven development workflow
- **Template Files**: spec-template.md, plan-template.md, tasks-template.md for workflow structure

### Content Dependencies

- **Chapter Sequence**: Each chapter builds on previous chapters (must be developed in order or with awareness of dependencies)
- **Visual Content**: Mermaid diagrams and tables must be created as chapters are written
- **Code Examples**: All code samples must be tested and validated before inclusion
- **Citations**: Source materials must be accessible for fact verification

### Infrastructure Dependencies

- **GitHub Repository**: Must be public for GitHub Pages deployment
- **Domain Configuration**: GitHub Pages URL or custom domain must be configured
- **Environment Secrets**: API keys must be stored securely in GitHub Secrets and Render environment variables
- **Network Access**: Deployment services must be accessible from development environment

## Constraints

### Technical Constraints

- **Docusaurus Framework**: Must use Docusaurus 3.9 specifically (not earlier/later versions)
- **Free Tier Limits**:
  - Qdrant: 1GB storage maximum
  - Render: Limited CPU/memory for free tier
  - GitHub Pages: 1GB site size maximum, 100GB/month bandwidth
- **OpenAI API Limits**:
  - Rate limits: 3 requests per minute (tier 1 free account)
  - Token limits: 200,000 tokens per day
- **Browser Compatibility**: Modern browsers only (no IE11 support required)
- **Mobile Responsiveness**: Minimum viewport width 375px (iPhone SE size)

### Content Constraints

- **Page Count**: Each chapter must be 5-8 pages (strict requirement from constitution)
- **Blueprint Adherence**: Cannot deviate from MASTER-BLUEPRINT.md structure without constitutional amendment
- **Citation Requirement**: All facts must be cited (no exceptions per constitution)
- **2025 Currency**: All technical data must be current as of 2025
- **English Only**: Content is provided in English (translations are bonus features)

### Performance Constraints

- **Load Time**: <2 seconds on 4G connection (constitutional requirement)
- **Response Time**: RAG chatbot <3 seconds 95th percentile (user requirement)
- **Lighthouse Scores**: ≥90 on all metrics (constitutional requirement)
- **Bundle Size**: <300KB per page gzipped (constitutional requirement)

### Time Constraints

- **Hackathon Deadline**: Project must be complete by submission deadline
- **Sequential Development**: Chapters should be developed in priority order (1, 3, 6 first per blueprint)
- **Quality Gates**: Each chapter must pass quality checklist before moving to next

### Cost Constraints

- **Budget**: Total monthly operating cost must be <$10
- **Free Tier Priority**: Must use free tiers wherever possible
- **API Cost Management**: OpenAI API usage must be monitored and optimized

### Accessibility Constraints

- **WCAG AA Compliance**: Mandatory (not optional)
- **Screen Reader Support**: Must work with NVDA/JAWS/VoiceOver
- **Keyboard Navigation**: All features must be keyboard-accessible
- **Color Contrast**: Must meet WCAG AA contrast ratios in all themes

### Deployment Constraints

- **GitHub Pages Only**: Book hosting must use GitHub Pages (no alternative platforms)
- **Automated Deployment**: Manual deployment not acceptable (CI/CD required)
- **Public Repository**: Code must be public for hackathon evaluation
- **Environment Isolation**: Development/staging/production environments must use separate configurations

## Risks & Mitigations

### High-Priority Risks

1. **Risk**: Qdrant free tier (1GB) insufficient for all chapter embeddings

   - **Impact**: RAG chatbot cannot index full book content
   - **Mitigation**:
     - Calculate embedding size early (test with first 3 chapters)
     - Optimize chunk size (smaller chunks = more embeddings)
     - Use lower-dimensional embeddings if needed
     - Monitor storage usage after each chapter

2. **Risk**: OpenAI API costs exceed budget (<$10/month)

   - **Impact**: Project becomes financially unsustainable
   - **Mitigation**:
     - Use gpt-4o-mini (cheapest model) for chat completions
     - Implement caching for common queries
     - Set hard rate limits on API calls
     - Monitor costs daily via OpenAI dashboard

3. **Risk**: Time constraint prevents completing all 10 chapters before deadline
   - **Impact**: Incomplete book, hackathon disqualification
   - **Mitigation**:
     - Prioritize critical chapters (1, 3, 6 first per blueprint)
     - Use Claude Code and Spec-Kit Plus for development acceleration
     - Reuse existing blueprint content (diagrams, tables already specified)
     - Set internal milestones (2 chapters per week)

### Medium-Priority Risks

4. **Risk**: RAG chatbot retrieval accuracy <85% (fails success criteria)

   - **Impact**: Poor user experience, hackathon points deduction
   - **Mitigation**:
     - Create test question set early (30+ questions covering all chapters)
     - Iterate on chunking strategy (test different sizes/overlaps)
     - Tune retrieval parameters (top-k, similarity threshold)
     - Add metadata to improve context awareness

5. **Risk**: Performance benchmarks not met (Lighthouse <90, load time >2s)

   - **Impact**: Constitutional violation, poor user experience
   - **Mitigation**:
     - Run Lighthouse audits after every chapter addition
     - Optimize images (WebP format, lazy loading)
     - Code split by route (one bundle per chapter)
     - Use Docusaurus performance best practices

6. **Risk**: Mermaid diagrams don't render correctly in dark mode
   - **Impact**: Constitutional violation (dark mode compatibility required)
   - **Mitigation**:
     - Test every diagram in both light and dark themes
     - Use theme-aware color variables
     - Provide fallback styling if needed
     - Document theme constraints for diagram creation

### Low-Priority Risks

7. **Risk**: GitHub Actions deployment fails intermittently

   - **Impact**: Delayed updates, manual intervention required
   - **Mitigation**:
     - Test deployment pipeline early with sample content
     - Add retry logic to deployment script
     - Monitor GitHub Actions status page for outages
     - Document manual deployment procedure as backup

8. **Risk**: Render free tier has insufficient resources for RAG API

   - **Impact**: Slow responses, API timeouts
   - **Mitigation**:
     - Load test API with expected traffic (100 concurrent users)
     - Optimize FastAPI code (async endpoints, efficient queries)
     - Consider upgrading to paid tier if necessary (<$7/month)
     - Implement request queueing if needed

9. **Risk**: Code examples contain errors or don't run correctly
   - **Impact**: Constitutional violation (all code must be tested), user confusion
   - **Mitigation**:
     - Test every code example before inclusion
     - Use linters (flake8, mypy for Python)
     - Include environment/version information in code comments
     - Provide "run this code" links to Colab/Replit where applicable

## Questions for Clarification

_None at this time. All requirements are clear from the user input, MASTER-BLUEPRINT.md, and hackathon document. Informed assumptions have been made for minor details (e.g., OpenAI model selection: gpt-4o-mini for cost, text-embedding-3-small for embeddings) based on specified goals (cost efficiency, quality)._
