# Physical AI & Humanoid Robotics - Interactive Textbook

## Project Overview

This is an AI-enhanced educational platform that combines comprehensive robotics education with RAG-powered interactive learning. The project bridges the gap between artificial intelligence and physical embodiment, teaching how to build intelligent robots that perceive, reason, and act in the real world.

**Key Stats:**
- 10+ comprehensive chapters (~38,000 words)
- 17+ Mermaid diagrams for visual learning
- RAG-powered chatbot with semantic search
- Hands-on labs with automated testing
- Full-stack TypeScript/Python architecture

## Tech Stack

### Frontend
- **Docusaurus 3.9.2** - Static site generator for documentation
- **React 19.0.0** - UI framework
- **TypeScript 5.6.2** - Type safety
- **Mermaid** - Diagram rendering
- **Lucide React** - Modern icons
- **Node.js 20+** - JavaScript runtime

### Backend
- **FastAPI 0.109.0** - Async Python web framework
- **OpenAI API** - GPT-4o-mini (responses), text-embedding-3-small (embeddings)
- **Qdrant 1.7.3+** - Vector database for semantic search
- **PostgreSQL (Neon)** - User authentication database
- **Uvicorn** - ASGI server
- **Python 3.10+** - Backend language

### Infrastructure
- **Docker & Docker Compose** - Containerization
- **Vercel** - Frontend deployment
- **Qdrant Cloud** - Managed vector database (1GB free tier)
- **Environment Variables** - Secure API key management

## Architecture

### High-Level Data Flow

```
User Question (Frontend)
    ↓
POST /api/v1/query
    ↓
RAGService (Backend)
    ├─→ Generate embedding (OpenAI)
    ├─→ Search similar chunks (Qdrant)
    ├─→ Retrieve context + metadata
    └─→ Generate answer (OpenAI GPT-4o-mini)
    ↓
Structured Response {answer, sources, citations}
    ↓
ChatWidget renders answer + clickable citations
```

### RAG Pipeline Details

1. **Embedding Generation**: User query → 1536-dim vector (text-embedding-3-small)
2. **Semantic Search**: Query Qdrant for top-5 similar chunks (cosine similarity)
3. **Context Retrieval**: Get content + metadata (module, chapter, section, file_path)
4. **Answer Generation**: GPT-4o-mini with retrieved context
5. **Citation Extraction**: Parse section numbers from response
6. **Response**: `{answer, sources, citations}` JSON

## Directory Structure

```
physical-ai-robotics-book/
├── frontend/                    # Docusaurus textbook + React UI
│   ├── docs/                   # Educational content (21 markdown files)
│   │   ├── intro.md            # Main introduction
│   │   ├── module-01-ros2/     # ROS 2 Module (8 chapters)
│   │   │   ├── ros2-fundamentals/     # 1.1.1-1.1.4
│   │   │   └── urdf-robot-description/ # 1.2.1-1.2.4
│   │   └── labs/               # Lab instructions
│   ├── src/
│   │   ├── components/
│   │   │   ├── ChatWidget/     # RAG-powered chatbot (main feature)
│   │   │   ├── Auth/           # Authentication UI
│   │   │   ├── HomepageFeatures/
│   │   │   ├── LabCallout/
│   │   │   └── Quiz/
│   │   ├── css/                # Styling
│   │   ├── pages/              # Additional pages
│   │   └── theme/              # Theme customization
│   ├── package.json
│   └── docusaurus.config.ts
│
├── backend/                     # FastAPI RAG backend
│   ├── app/
│   │   ├── main.py             # FastAPI entry point + health check
│   │   ├── config/
│   │   │   ├── settings.py     # Environment config
│   │   │   └── database.py     # PostgreSQL connection pool
│   │   ├── models/
│   │   │   └── user.py         # User models
│   │   ├── services/
│   │   │   ├── rag.py          # RAGService (core logic, singleton)
│   │   │   └── auth.py         # Authentication service
│   │   ├── routes/
│   │   │   ├── rag.py          # API endpoints
│   │   │   └── auth.py         # Auth routes
│   │   └── middleware/
│   ├── scripts/
│   │   └── seed_vector_db.py   # Indexes textbook into Qdrant
│   ├── requirements.txt
│   └── setup_database.py
│
├── labs/                        # Hands-on exercises
│   ├── lab01-ros2-basics/
│   │   ├── starter/            # Skeleton code with TODOs
│   │   ├── solutions/          # Complete implementations
│   │   ├── tests/              # Automated test suite
│   │   └── assets/             # Expected outputs
│   └── lab02-urdf-humanoid/    # (Coming soon)
│
├── docker-compose.yml          # Multi-container orchestration
├── Dockerfile.frontend
├── Dockerfile.backend
└── vercel.json                 # Deployment config
```

## Key Features

### 1. RAG-Powered Chatbot
**The project's centerpiece feature:**

- Semantic search across entire textbook content (~85-90 vector chunks)
- Context-aware answers using GPT-4o-mini
- Clickable citations that jump to source sections
- Floating UI (bottom-right, non-intrusive)
- Mobile-responsive design
- Loading states and smooth animations

**API Endpoints:**
- `POST /api/v1/query` - Main RAG query endpoint
- `GET /api/v1/health` - Health check + Qdrant connection status
- `GET /api/v1/stats` - Vector DB statistics

### 2. Educational Content
**Module 1: The Robotic Nervous System (Complete)**

- **Chapter 1.1: ROS 2 Fundamentals** (4 sections)
  - Architecture, rclpy patterns, parameters, QoS

- **Chapter 1.2: URDF & Robot Description** (4 sections)
  - URDF basics, sensors, kinematics, testing

**Features:**
- Real-world analogies (e.g., "Restaurant Kitchen = ROS 2")
- Step-by-step installation guides
- Line-by-line code explanations
- 17+ Mermaid diagrams

### 3. Hands-On Labs
- Starter code with TODO hints
- Complete solutions for reference
- Automated tests (pytest)
- Real ROS 2 packages
- **Lab 1**: Heartbeat publisher with 7 guided TODOs
- **Lab 2**: URDF humanoid (in progress)

### 4. Authentication System (In Development)
- JWT token-based authentication
- PostgreSQL (Neon) for user persistence
- Better-auth integration
- Login/Signup modals

## Development Setup

### Prerequisites
- **Node.js 20+**
- **Python 3.10+**
- **Docker** (optional, for containerized development)
- **Git**

### Environment Variables

Create `backend/.env`:
```env
OPENAI_API_KEY=your-openai-key
QDRANT_URL=your-qdrant-cloud-url
QDRANT_API_KEY=your-qdrant-api-key
DATABASE_URL=postgresql://user:pass@host/db
JWT_SECRET=your-secret-key
```

### Frontend Setup

```bash
cd frontend
npm install
npm run dev  # Development server on http://localhost:3000
npm run build  # Production build
```

### Backend Setup

```bash
cd backend
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
pip install -r requirements.txt
python -m uvicorn app.main:app --reload --port 8000
```

### Seed Vector Database

```bash
cd backend
python scripts/seed_vector_db.py
```

This indexes all textbook content into Qdrant (creates ~85-90 vector chunks).

### Docker Setup

```bash
docker-compose up --build
# Frontend: http://localhost:3000
# Backend: http://localhost:8000
```

## API Documentation

### POST /api/v1/query
Query the RAG system.

**Request:**
```json
{
  "query": "What is ROS 2?",
  "module_filter": "module-01-ros2",  // optional
  "top_k": 5  // optional, default: 5
}
```

**Response:**
```json
{
  "answer": "ROS 2 is a middleware framework...",
  "sources": [
    {
      "content": "Context chunk 1...",
      "metadata": {
        "module": "module-01-ros2",
        "chapter": "ros2-fundamentals",
        "section": "1.1.1-architecture",
        "file_path": "docs/module-01-ros2/..."
      },
      "score": 0.92
    }
  ],
  "citations": ["1.1.1", "1.1.2"],
  "query_time_ms": 2341
}
```

### GET /api/v1/health
Check backend and Qdrant health.

**Response:**
```json
{
  "status": "healthy",
  "qdrant_connected": true,
  "timestamp": "2025-12-15T12:00:00Z"
}
```

### GET /api/v1/stats
Get vector database statistics.

**Response:**
```json
{
  "collection_name": "physical_ai_textbook",
  "points_count": 87,
  "status": "green"
}
```

## Code Conventions

### Frontend
- **React Components**: Functional components with TypeScript
- **Styling**: Module-scoped CSS (`ComponentName.module.css`)
- **State Management**: React hooks (useState, useEffect)
- **API Calls**: Fetch API with async/await
- **Naming**: PascalCase for components, camelCase for functions/variables

### Backend
- **Async/Await**: All I/O operations are async
- **Type Hints**: Full Python type annotations
- **Error Handling**: Try-except with proper HTTP status codes
- **Logging**: Structured logging with context
- **Singleton Pattern**: RAGService uses singleton for client reuse
- **Naming**: snake_case for all Python code

### Git Workflow
- **Branch Naming**: `feature/description`, `fix/description`
- **Commit Messages**: Clear, descriptive (use conventional commits)
- **No Secrets**: NEVER commit API keys (use `.env` files)
- **Pre-commit Hooks**: Automatic linting and formatting

## Important Files

| File | Purpose |
|------|---------|
| `backend/app/main.py` | FastAPI application entry point |
| `backend/app/services/rag.py` | RAG pipeline core logic (singleton) |
| `backend/app/routes/rag.py` | API routes + request/response models |
| `frontend/src/components/ChatWidget/ChatWidget.tsx` | Main chatbot UI component |
| `frontend/docs/intro.md` | Book introduction + learning paths |
| `frontend/docusaurus.config.ts` | Site configuration + metadata |
| `backend/scripts/seed_vector_db.py` | Vector DB indexing script |
| `docker-compose.yml` | Multi-container orchestration |
| `vercel.json` | Vercel deployment configuration |

## Deployment

### Frontend (Vercel)
1. Connect GitHub repository to Vercel
2. Set root directory to `frontend`
3. Build command: `npm run build`
4. Output directory: `build`
5. Environment variables: None needed (API calls go to backend)

### Backend
**Options:**
1. **Vercel Serverless** (current setup)
2. **Railway** (recommended for 24/7 uptime)
3. **Docker** on any VPS

**Required Environment Variables:**
- `OPENAI_API_KEY`
- `QDRANT_URL`
- `QDRANT_API_KEY`
- `DATABASE_URL`
- `JWT_SECRET`

### Vector Database (Qdrant Cloud)
1. Create free cluster at https://cloud.qdrant.io/
2. Note URL and API key
3. Run `seed_vector_db.py` to index content
4. Free tier: 1GB storage (~10,000 vectors)

## Performance Metrics

- **Query Latency**: 2-4 seconds (embedding + search + generation)
- **API Cost**: ~$0.001-0.003 per query
- **Vector DB**: ~85-90 chunks (500 tokens each, 50 token overlap)
- **Embedding Dimension**: 1536 (text-embedding-3-small)
- **Content Size**: ~38,000 words, 10 pages, 17 diagrams

## Troubleshooting

### Common Issues

**Frontend won't start:**
```bash
rm -rf node_modules package-lock.json
npm install
npm run dev
```

**Backend 500 errors:**
- Check `.env` file exists with all required keys
- Verify Qdrant connection: `curl http://localhost:8000/api/v1/health`
- Check logs: `docker-compose logs backend`

**ChatWidget not loading:**
- Ensure backend is running on correct port
- Check CORS configuration in `backend/app/main.py`
- Verify API endpoint in `ChatWidget.tsx`

**Vector DB empty:**
```bash
cd backend
python scripts/seed_vector_db.py
```

**Vercel deployment fails:**
- Ensure `vercel.json` has correct config
- Check build logs for missing dependencies
- Verify Node.js version (20+)

## Security Best Practices

1. **API Keys**: Always store in `.env` files (never commit to git)
2. **Authentication**: JWT tokens with secure secrets
3. **CORS**: Restrict to known domains in production
4. **Input Validation**: Pydantic models validate all inputs
5. **Rate Limiting**: Implement for production (not yet added)
6. **HTTPS**: Required for production (Vercel provides automatically)

## Contributing

When working on this project:

1. **Read the docs first**: Understand the architecture before making changes
2. **Test locally**: Always test changes with `npm run dev` and `uvicorn`
3. **Follow conventions**: Use existing code style and patterns
4. **Update docs**: Keep this file and README.md current
5. **No secrets**: Use `.gitignore` for sensitive files
6. **Write tests**: Add tests for new features
7. **Clear commits**: Descriptive commit messages

## Resources

- **OpenAI API**: https://platform.openai.com/docs
- **Qdrant Docs**: https://qdrant.tech/documentation/
- **Docusaurus**: https://docusaurus.io/docs
- **FastAPI**: https://fastapi.tiangolo.com/
- **ROS 2**: https://docs.ros.org/en/humble/

## Project Status

**Current Phase**: Foundation (Phase 1) ✅
- [x] Module 1 content complete
- [x] RAG backend operational
- [x] ChatWidget integrated
- [x] Vector DB seeded
- [x] Authentication framework

**Next Steps**:
- [ ] Complete remaining Module 1 chapters
- [ ] Labs 2-3 (URDF humanoid, sensor fusion)
- [ ] Modules 2-4 content
- [ ] Multi-turn conversation history
- [ ] User feedback mechanism
- [ ] Production monitoring

## Contact

For questions about this codebase:
- Review this document first
- Check existing documentation in `DEPLOYMENT.md` and `QDRANT-SETUP-GUIDE.md`
- Refer to inline code comments
- Check commit history for context on specific changes

---

**Last Updated**: 2025-12-15
**Version**: 1.0.0
**License**: MIT (assumed - add LICENSE file if needed)
