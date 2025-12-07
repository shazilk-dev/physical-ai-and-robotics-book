# 🤖 Physical AI & Humanoid Robotics - Interactive Textbook

> **Hackathon Project:** An AI-enhanced educational platform combining comprehensive robotics content with RAG-powered interactive learning.

[![Docusaurus](https://img.shields.io/badge/Docusaurus-3.9.3-green.svg)](https://docusaurus.io/)
[![FastAPI](https://img.shields.io/badge/FastAPI-0.109.0-009688.svg)](https://fastapi.tiangolo.com/)
[![OpenAI](https://img.shields.io/badge/OpenAI-GPT--4o--mini-412991.svg)](https://openai.com/)
[![Qdrant](https://img.shields.io/badge/Qdrant-Vector_DB-DC244C.svg)](https://qdrant.tech/)

## 🌟 What Makes This Special

This isn't just another robotics textbook. It's an **AI-enhanced learning platform** that combines:

1. **📚 High-Quality Educational Content**

   - 10 comprehensive chapters (~38,000 words)
   - 17+ Mermaid diagrams for visual learning
   - Real-world analogies (restaurant = ROS 2 architecture)
   - Step-by-step installation guides
   - Line-by-line code explanations

2. **🤖 RAG-Powered Chatbot**

   - Ask questions in natural language
   - Get contextual answers from textbook content
   - **Verifiable citations** - click to jump to source sections
   - Powered by GPT-4o-mini + Qdrant vector search

3. **💻 Hands-On Labs**
   - Starter code with TODO hints
   - Complete solutions for reference
   - Automated tests
   - Real ROS 2 packages you can run

## 🚀 Quick Start

### Prerequisites

- **Node.js** 18+ (for frontend)
- **Python** 3.10+ (for backend)
- **OpenAI API Key** ([get one here](https://platform.openai.com/api-keys))
- **Qdrant Cloud** (free tier) OR local Docker

### 1. Clone Repository

```bash
git clone https://github.com/your-username/physical-ai-robotics-book.git
cd physical-ai-robotics-book
```

### 2. Backend Setup

```bash
cd backend

# Install dependencies
pip install -r requirements.txt

# Create .env file
cat > .env << EOF
OPENAI_API_KEY=sk-your-key-here
QDRANT_URL=https://your-cluster.qdrant.io:6333
QDRANT_API_KEY=your-api-key
QDRANT_COLLECTION=physical_ai_textbook
EOF

# Seed vector database (one-time setup)
python scripts/seed_vector_db.py

# Start backend server
uvicorn app.main:app --reload --port 8000
```

**Backend runs at:** `http://localhost:8000`

### 3. Frontend Setup

```bash
cd frontend

# Install dependencies
npm install

# Start development server
npm run start
```

**Frontend runs at:** `http://localhost:3000`

### 4. Test the Chatbot

1. Open `http://localhost:3000`
2. Click the **💬 floating button** (bottom-right)
3. Ask: **"What is ROS 2?"**
4. Click a **citation** to jump to the source section

## 📖 Content Overview

### Module 1: ROS 2 - The Robot Nervous System ✅ (Complete)

**Chapter 1.1: ROS 2 Fundamentals**

- 1.1.1 Architecture - Nodes, topics, services, actions
- 1.1.2 rclpy Patterns - Publishers, subscribers, timers
- 1.1.3 Parameters & Launch Files - Configuration management
- 1.1.4 QoS & Real-Time - Reliability, latency budgeting

**Chapter 1.2: URDF & Robot Description**

- 1.2.1 URDF Basics - Links, joints, coordinate frames
- 1.2.2 Sensors in URDF - Cameras, IMU, LiDAR integration
- 1.2.3 Validating Kinematics - FK/IK, collision checking
- 1.2.4 Package Testing - pytest, CI/CD, pre-commit hooks

**Lab 1: ROS 2 Basics** ✅

- Heartbeat node with publisher pattern
- Starter code with 7 guided TODOs
- Complete solution and automated tests

### Modules 2-4 (Coming Soon)

- **Module 2:** Core Architecture (sensing, control, planning)
- **Module 3:** Isaac Sim & Digital Twins
- **Module 4:** Vision-Language-Action Models

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    User Interface                       │
│  Docusaurus 3.9.3 + React 19 + ChatWidget Component   │
└────────────────┬────────────────────────────────────────┘
                 │
                 │ HTTP POST /api/v1/query
                 │
┌────────────────▼────────────────────────────────────────┐
│                   FastAPI Backend                       │
│  - RAGService (singleton)                              │
│  - Routes: /query, /health, /stats                     │
└────────────────┬────────────────────────────────────────┘
                 │
        ┌────────┴────────┐
        │                 │
┌───────▼─────────┐  ┌───▼──────────────┐
│  OpenAI API     │  │  Qdrant Cloud    │
│  - Embeddings   │  │  - Vector DB     │
│  - GPT-4o-mini  │  │  - 1536-dim      │
│  (Generation)   │  │  - COSINE        │
└─────────────────┘  └──────────────────┘
```

### RAG Pipeline Flow

```
1. User asks: "What is ROS 2?"
                ↓
2. Generate embedding (text-embedding-3-small)
                ↓
3. Search Qdrant (top-5 similar chunks)
                ↓
4. Build context from retrieved chunks
                ↓
5. Generate answer (GPT-4o-mini)
                ↓
6. Extract citations (section numbers)
                ↓
7. Return {answer, sources, citations}
```

## 🎨 Features Showcase

### 1. Visual Learning with Mermaid Diagrams

Every concept includes visual representations:

- ROS 2 architecture (nodes, topics, middleware)
- Communication patterns (pub/sub, request/response, actions)
- URDF robot structure trees
- QoS decision trees
- Test workflows and CI pipelines

### 2. Beginner-Friendly Explanations

**Example: ROS 2 = Restaurant Kitchen**

- **Chef** = Node (independent worker)
- **Orders** = Topics (broadcast messages)
- **Waiter** = Service (request → response)
- **Delivery Tracking** = Action (long-running with feedback)

### 3. Line-by-Line Code Walkthrough

```python
# Create publisher
self.publisher = self.create_publisher(
    String,              # ← Message type
    '/robot/ready',      # ← Topic name
    10                   # ← QoS (queue size)
)
```

### 4. Interactive ChatWidget

- **Floating button** - Non-intrusive, always accessible
- **Smooth animations** - Slide-up panel, fade-in messages
- **Loading states** - Animated dots while thinking
- **Clickable citations** - Jump directly to source sections
- **Mobile responsive** - Works on all screen sizes

## 🧪 Testing the RAG System

### Sample Queries & Expected Results

**Query 1:** _"What is ROS 2 and why do we need it?"_

- **Answer:** Explanation of ROS 2 as middleware framework
- **Citations:** 1.1.1 Architecture, 1.1.2 Rclpy Patterns
- **Sources:** 2-3 chunks from Chapter 1.1

**Query 2:** _"How do I create a publisher in Python?"_

- **Answer:** Code snippet with `create_publisher()` method
- **Citations:** 1.1.2 Rclpy Patterns
- **Sources:** Publisher pattern section with example

**Query 3:** _"What is URDF used for?"_

- **Answer:** Robot description language explanation
- **Citations:** 1.2.1 Urdf Basics
- **Sources:** URDF introduction and blueprint analogy

**Query 4:** _"How do I add a camera to my URDF?"_

- **Answer:** Steps for camera link + Gazebo plugin
- **Citations:** 1.2.2 Sensors Urdf
- **Sources:** Camera integration section with code

### Backend API Endpoints

**Health Check:**

```bash
curl http://localhost:8000/api/v1/health
```

**Query:**

```bash
curl -X POST http://localhost:8000/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?", "num_results": 5}'
```

**Stats:**

```bash
curl http://localhost:8000/api/v1/stats
```

## 📂 Project Structure

```
physical-ai-robotics-book/
├── frontend/                    # Docusaurus textbook
│   ├── docs/
│   │   ├── intro.md            # Main introduction
│   │   └── module-01-ros2/     # Module 1 content
│   │       ├── overview.md     # Setup guide
│   │       ├── ros2-fundamentals/
│   │       │   ├── 1.1.1-architecture.md
│   │       │   ├── 1.1.2-rclpy-patterns.md
│   │       │   ├── 1.1.3-parameters-launch.md
│   │       │   └── 1.1.4-qos-realtime.md
│   │       └── 02-urdf-robot-description/
│   │           ├── 1.2.1-urdf-basics.md
│   │           ├── 1.2.2-sensors-urdf.md
│   │           ├── 1.2.3-validating-kinematics.md
│   │           └── 1.2.4-package-testing.md
│   ├── src/
│   │   ├── components/
│   │   │   └── ChatWidget/    # RAG chatbot component
│   │   │       ├── ChatWidget.tsx
│   │   │       └── ChatWidget.module.css
│   │   └── theme/
│   │       └── Root.tsx        # Theme wrapper
│   └── docusaurus.config.ts
│
├── backend/                     # FastAPI RAG API
│   ├── app/
│   │   ├── main.py             # FastAPI app entry
│   │   ├── config/
│   │   │   └── settings.py     # Environment config
│   │   ├── services/
│   │   │   └── rag.py          # RAGService core
│   │   └── routes/
│   │       └── rag.py          # API endpoints
│   ├── scripts/
│   │   └── seed_vector_db.py   # Content indexing
│   └── requirements.txt
│
├── labs/                        # Hands-on exercises
│   ├── lab01-ros2-basics/
│   │   ├── README.md
│   │   ├── starter/            # Code with TODOs
│   │   ├── solutions/          # Complete reference
│   │   └── tests/
│   └── lab02-urdf-humanoid/
│
├── specs/                       # Project specifications
│   └── 001-physical-ai-book/
│       ├── spec.md
│       ├── plan.md
│       └── tasks.md
│
├── CHATWIDGET-DEMO-GUIDE.md    # Complete testing guide
└── README.md
```

## 🛠️ Tech Stack

### Frontend

- **Docusaurus** 3.9.3 - Documentation framework
- **React** 19.0.0 - UI library
- **TypeScript** 5.6.2 - Type safety
- **Mermaid** - Diagram rendering
- **Prism** - Syntax highlighting

### Backend

- **FastAPI** 0.109.0 - Async web framework
- **Qdrant** 1.7.3 - Vector database
- **OpenAI** 1.10.0 - LLM and embeddings
- **tiktoken** 0.5.2 - Token counting
- **Pydantic** - Data validation

### Infrastructure

- **Qdrant Cloud** - Managed vector DB (1GB free tier)
- **OpenAI API** - GPT-4o-mini + text-embedding-3-small
- **Uvicorn** - ASGI server
- **Node.js** 18+ - JavaScript runtime

## 📊 Performance Metrics

### Current Stats

- **Content:** 10 pages, ~38,000 words, 17 diagrams
- **Vector DB:** ~85-90 chunks (500 tokens each, 50 overlap)
- **Embedding Dimension:** 1536 (text-embedding-3-small)
- **Query Latency:** 2-4 seconds (embedding + search + generation)
- **API Cost:** ~$0.001-0.003 per query

### Optimization Targets

- Response time < 3s (p95)
- Embedding accuracy > 0.7 (cosine similarity)
- 80%+ answer relevance (user feedback)

## 🎯 Roadmap

### Phase 1: Foundation ✅ (Current)

- [x] Module 1 content (10 pages)
- [x] RAG backend (FastAPI + Qdrant)
- [x] ChatWidget frontend (React component)
- [x] Vector DB seeding
- [x] API integration

### Phase 2: Content Expansion (Next)

- [ ] Complete Chapter 1.3 (Sensors & Proprioception)
- [ ] Labs 2-3 (URDF humanoid, sensor fusion)
- [ ] Module 2: Core Architecture (14 pages)
- [ ] Module 3: Isaac Sim (13 pages)
- [ ] Module 4: VLA & Whisper (11 pages)

### Phase 3: Feature Enhancements

- [ ] Conversation history (multi-turn chat)
- [ ] Feedback mechanism (👍/👎 on answers)
- [ ] Query suggestions
- [ ] Code syntax highlighting in responses
- [ ] Module filtering UI

### Phase 4: Production

- [ ] Deploy backend (Railway/Render)
- [ ] Deploy frontend (GitHub Pages/Vercel)
- [ ] Custom domain
- [ ] Analytics (PostHog)
- [ ] Monitoring & alerts

## 🤝 Contributing

Contributions welcome! Areas of interest:

- **Content:** Additional modules, labs, diagrams
- **Code:** Bug fixes, feature enhancements
- **Design:** UI/UX improvements for ChatWidget
- **Testing:** Unit tests, integration tests, E2E tests

## 📄 License

MIT License - See [LICENSE](LICENSE) for details.

## 🙏 Acknowledgments

- **Docusaurus Team** - Excellent documentation framework
- **OpenAI** - GPT-4o-mini and embeddings API
- **Qdrant** - High-performance vector database
- **ROS 2 Community** - Comprehensive robotics middleware
- **Physical AI Hackathon** - Inspiration and motivation

## 📧 Contact

- **GitHub Issues:** [Report bugs](https://github.com/your-username/physical-ai-robotics-book/issues)
- **Discussions:** [Ask questions](https://github.com/your-username/physical-ai-robotics-book/discussions)

---

**Built with ❤️ for the Physical AI & Humanoid Robotics Community**

_"Making robotics education accessible, interactive, and AI-enhanced"_
