# Quickstart Guide: Physical AI Book Development

**Feature**: 001-physical-ai-book  
**Last Updated**: 2025-12-07  
**Target Audience**: Developers setting up local environment for the first time

---

## Overview

This guide walks through setting up the development environment for the Physical AI & Humanoid Robotics educational book project. The stack includes:

- **Frontend**: Docusaurus 3.9.3 (React 18.3+, TypeScript 5.3+)
- **Backend**: FastAPI 0.115+ (Python 3.11+, Uvicorn ASGI server)
- **RAG System**: Qdrant Cloud vector database + OpenAI models
- **Development Tools**: Node.js 18.x, Python 3.11+, Git

**Time to Complete**: ~15 minutes

---

## Prerequisites

Before starting, ensure you have:

- [ ] **Node.js 18.x or later** ([download](https://nodejs.org/))
- [ ] **Python 3.11 or later** ([download](https://www.python.org/downloads/))
- [ ] **Git** ([download](https://git-scm.com/downloads))
- [ ] **Code Editor** (VS Code recommended)
- [ ] **OpenAI API Key** (create at [platform.openai.com](https://platform.openai.com/api-keys))
- [ ] **Qdrant Cloud Account** (free tier at [cloud.qdrant.io](https://cloud.qdrant.io/))

---

## Step 1: Clone Repository

```powershell
# Clone the repository
git clone https://github.com/your-org/physical-ai-book.git
cd physical-ai-book

# Checkout the feature branch
git checkout 001-physical-ai-book

# Verify you're on the correct branch
git branch  # Should show * 001-physical-ai-book
```

---

## Step 2: Frontend Setup (Docusaurus)

### 2.1 Install Dependencies

```powershell
# Navigate to frontend directory
cd docs

# Install npm packages (this may take 2-3 minutes)
npm install

# Verify installation
npm list docusaurus  # Should show @docusaurus/core@3.9.3
```

### 2.2 Start Development Server

```powershell
# Start Docusaurus dev server with hot reload
npm start

# Server will start at http://localhost:3000
# Browser should open automatically
```

**Expected Output:**

```
[INFO] Starting the development server...
[SUCCESS] Docusaurus website is running at: http://localhost:3000/
```

### 2.3 Verify Frontend Works

1. Open browser to `http://localhost:3000`
2. Check that homepage loads (<2s load time)
3. Navigate to "Chapter 1: Foundations" in sidebar
4. Verify Mermaid diagrams render correctly

**Troubleshooting:**

- **Port 3000 already in use?** Run `npm start -- --port 3001`
- **Module not found errors?** Delete `node_modules/` and `package-lock.json`, then re-run `npm install`
- **Build errors?** Check Node.js version: `node --version` (must be ≥18.x)

---

## Step 3: Backend Setup (FastAPI)

### 3.1 Create Python Virtual Environment

```powershell
# Navigate to backend directory (from project root)
cd api

# Create virtual environment
python -m venv venv

# Activate virtual environment
.\venv\Scripts\Activate.ps1

# Verify activation (prompt should show "(venv)")
```

### 3.2 Install Python Dependencies

```powershell
# Upgrade pip
python -m pip install --upgrade pip

# Install FastAPI and dependencies
pip install -r requirements.txt

# Verify installation
pip list | Select-String "fastapi"  # Should show fastapi 0.115.x
pip list | Select-String "qdrant"   # Should show qdrant-client 1.x
pip list | Select-String "openai"   # Should show openai 1.x
```

**What's in `requirements.txt`?**

```
fastapi==0.115.5
uvicorn[standard]==0.32.1
pydantic==2.10.3
qdrant-client==1.12.1
openai==1.57.2
python-dotenv==1.0.1
```

### 3.3 Configure Environment Variables

```powershell
# Create .env file in api/ directory
@"
# OpenAI Configuration
OPENAI_API_KEY=sk-proj-your-api-key-here
OPENAI_EMBEDDING_MODEL=text-embedding-3-small
OPENAI_CHAT_MODEL=gpt-4o-mini

# Qdrant Configuration
QDRANT_URL=https://your-cluster-id.us-east-1-0.aws.cloud.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key-here
QDRANT_COLLECTION_NAME=physical_ai_book

# API Configuration
API_HOST=0.0.0.0
API_PORT=8000
LOG_LEVEL=info
"@ | Out-File -FilePath .env -Encoding utf8
```

**How to get credentials:**

1. **OpenAI API Key**:

   - Go to [platform.openai.com/api-keys](https://platform.openai.com/api-keys)
   - Click "Create new secret key"
   - Copy key (starts with `sk-proj-...`)
   - Paste into `.env` file

2. **Qdrant URL & API Key**:
   - Go to [cloud.qdrant.io](https://cloud.qdrant.io/)
   - Create free cluster (1GB storage)
   - Copy "Cluster URL" (e.g., `https://abc123.us-east-1-0.aws.cloud.qdrant.io`)
   - Generate API key in dashboard → copy key
   - Paste both into `.env` file

### 3.4 Start FastAPI Server

```powershell
# Ensure you're in api/ directory with venv activated
# Start Uvicorn server with auto-reload
uvicorn main:app --reload --host 0.0.0.0 --port 8000

# Server will start at http://localhost:8000
```

**Expected Output:**

```
INFO:     Will watch for changes in these directories: ['F:\\PROJECTS\\ai-book\\physical-ai-robotics-book\\api']
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [12345] using StatReload
INFO:     Started server process [12346]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

### 3.5 Verify Backend Works

1. Open browser to `http://localhost:8000/docs` (FastAPI auto-generated docs)
2. Test health check:

   ```powershell
   curl http://localhost:8000/health
   ```

   Should return:

   ```json
   {
     "status": "healthy",
     "timestamp": "2025-12-07T14:30:00Z",
     "services": {
       "api": "healthy",
       "openai": "healthy",
       "qdrant": "healthy"
     },
     "version": "1.0.0"
   }
   ```

3. Test RAG query (in browser at `/docs`, expand `POST /api/v1/query`, click "Try it out"):
   ```json
   {
     "query": "What is Physical AI?"
   }
   ```
   Should return answer with sources in <3 seconds.

**Troubleshooting:**

- **ModuleNotFoundError?** Re-activate venv: `.\venv\Scripts\Activate.ps1`
- **OpenAI API error?** Check API key in `.env`, verify it's active at [platform.openai.com/usage](https://platform.openai.com/usage)
- **Qdrant connection error?** Verify cluster URL and API key, check cluster is running in Qdrant dashboard
- **Port 8000 in use?** Change port: `uvicorn main:app --reload --port 8001`

---

## Step 4: Verify Full Stack Integration

With both servers running (frontend at `:3000`, backend at `:8000`):

1. Open Docusaurus at `http://localhost:3000`
2. Navigate to any chapter page
3. Open the chatbot widget (bottom-right corner)
4. Ask: _"What is Physical AI?"_
5. Verify:
   - [ ] Response appears in <3 seconds
   - [ ] Answer includes citations (Chapter X, Section Y.Z)
   - [ ] Clicking citation scrolls to relevant content

**Network Flow:**

```
User types question in chatbot
  → React ChatWidget component sends POST to http://localhost:8000/api/v1/query
  → FastAPI receives request
  → Embeds query using OpenAI text-embedding-3-small
  → Queries Qdrant for top 5 similar chunks
  → Sends context + query to gpt-4o-mini
  → Returns answer + sources to frontend
  → ChatWidget renders response with citations
```

---

## Step 5: Run Tests

### 5.1 Frontend Tests

```powershell
# In docs/ directory
npm test

# Run Lighthouse performance audit (requires Chrome)
npm run lighthouse
```

**Expected Results:**

- All unit tests pass
- Lighthouse scores: Performance ≥90, Accessibility ≥90, SEO ≥90

### 5.2 Backend Tests

```powershell
# In api/ directory with venv activated
pytest tests/ -v

# Run with coverage report
pytest tests/ --cov=. --cov-report=html
```

**Expected Results:**

- All tests pass
- Coverage ≥80% for core modules (`main.py`, `rag.py`, `models.py`)

---

## Step 6: Index Book Content in Qdrant

**Note**: This only needs to be done once (or when content changes).

```powershell
# In api/ directory with venv activated
python scripts/index_content.py

# Expected output:
# [INFO] Connecting to Qdrant...
# [INFO] Creating collection 'physical_ai_book'...
# [INFO] Processing Chapter 1: Foundations of Physical AI...
# [INFO] - Chunked into 11 documents
# [INFO] - Embedded and uploaded to Qdrant
# [INFO] Processing Chapter 2: Core Concepts...
# ...
# [SUCCESS] Indexed 110 documents in 45.2 seconds
# [SUCCESS] Total cost: $1.06 (embedding tokens)
```

**What this script does:**

1. Reads all chapter Markdown files from `docs/docs/chapters/`
2. Splits each chapter into 500-token chunks with 50-token overlap
3. Generates embeddings using OpenAI `text-embedding-3-small`
4. Uploads embeddings + metadata to Qdrant collection
5. Verifies all chunks are indexed

**Verification:**

```powershell
# Check collection stats
python scripts/check_index.py

# Expected output:
# Collection: physical_ai_book
# Total documents: 110
# Vector dimensions: 1536
# Storage used: ~730 KB / 1 GB (0.07%)
```

---

## Common Development Commands

### Frontend (Docusaurus)

```powershell
# Start dev server
npm start

# Build production site
npm run build

# Serve production build locally
npm run serve

# Clear cache (if seeing stale content)
npm run clear

# Check for linting errors
npm run lint

# Format code with Prettier
npm run format
```

### Backend (FastAPI)

```powershell
# Start dev server with auto-reload
uvicorn main:app --reload

# Start production server (no reload)
uvicorn main:app --host 0.0.0.0 --port 8000

# Run tests
pytest tests/

# Run tests with coverage
pytest tests/ --cov=. --cov-report=html

# Format code with Black
black .

# Lint with Ruff
ruff check .

# Type check with MyPy
mypy .
```

---

## Project Structure Reference

```
physical-ai-book/
├── docs/                          # Docusaurus frontend
│   ├── docs/
│   │   └── chapters/              # Book content (Markdown)
│   │       ├── chapter-01.md
│   │       ├── chapter-02.md
│   │       └── ...
│   ├── src/
│   │   ├── components/            # React components
│   │   │   └── ChatWidget.tsx     # RAG chatbot UI
│   │   ├── css/                   # Styles
│   │   └── pages/                 # Custom pages
│   ├── static/                    # Images, diagrams
│   ├── docusaurus.config.ts       # Docusaurus configuration
│   ├── package.json
│   └── tsconfig.json
├── api/                           # FastAPI backend
│   ├── main.py                    # FastAPI app entry point
│   ├── models.py                  # Pydantic models
│   ├── rag.py                     # RAG logic (Qdrant + OpenAI)
│   ├── config.py                  # Environment config
│   ├── scripts/
│   │   ├── index_content.py       # Index chapters in Qdrant
│   │   └── check_index.py         # Verify index status
│   ├── tests/                     # Pytest tests
│   ├── requirements.txt
│   └── .env                       # Environment variables (not in Git)
├── specs/
│   └── 001-physical-ai-book/
│       ├── spec.md                # Feature specification
│       ├── plan.md                # Implementation plan
│       ├── data-model.md          # Entity definitions
│       ├── research.md            # Architecture research
│       └── contracts/
│           └── rag-api.openapi.yaml  # OpenAPI spec
└── .specify/
    └── memory/
        └── constitution.md        # Project principles
```

---

## Development Workflow

### 1. Making Content Changes

**Editing chapters:**

```powershell
# 1. Edit Markdown file
code docs/docs/chapters/chapter-01.md

# 2. Docusaurus auto-reloads at http://localhost:3000

# 3. Re-index content in Qdrant (so chatbot sees changes)
cd api
python scripts/index_content.py

# 4. Test chatbot with updated content
# Navigate to http://localhost:3000 → Ask question → Verify answer reflects changes
```

### 2. Adding New Chapters

```powershell
# 1. Create new Markdown file
New-Item docs/docs/chapters/chapter-11.md

# 2. Add frontmatter
@"
---
sidebar_position: 11
title: "Chapter 11: Advanced Topics"
---

# Chapter 11: Advanced Topics

## 11.1 Subchapter Title

Content here...
"@ | Out-File docs/docs/chapters/chapter-11.md -Encoding utf8

# 3. Update sidebar (if not using auto-generation)
# Edit docs/sidebars.ts

# 4. Re-index in Qdrant
cd api
python scripts/index_content.py
```

### 3. Modifying RAG Behavior

**Changing chunking strategy:**

```python
# Edit api/rag.py
CHUNK_SIZE = 500  # Change from 500 → 600 tokens
CHUNK_OVERLAP = 50  # Change overlap as needed

# Re-index with new chunking
python scripts/index_content.py --force-reindex
```

**Changing retrieval count:**

```python
# Edit api/rag.py
TOP_K = 5  # Change from 5 → 7 documents retrieved per query

# No re-indexing needed; change takes effect immediately
```

---

## Performance Optimization

### Frontend (Docusaurus)

**Enable code splitting:**

```javascript
// docs/docusaurus.config.ts
export default {
  webpack: {
    jsLoader: (isServer) => ({
      loader: require.resolve("esbuild-loader"),
      options: {
        loader: "tsx",
        target: isServer ? "node12" : "es2017",
      },
    }),
  },
};
```

**Image optimization:**

```powershell
# Install Sharp for automatic image optimization
cd docs
npm install @docusaurus/faster
```

### Backend (FastAPI)

**Add response caching:**

```python
# api/main.py
from functools import lru_cache

@lru_cache(maxsize=100)
def embed_query(query: str):
    # Cached embeddings for repeated queries
    return openai.embeddings.create(...)
```

---

## Deployment Preview

### Deploy Frontend to GitHub Pages

```powershell
# Build optimized production bundle
cd docs
npm run build

# Deploy to GitHub Pages
$env:GIT_USER="your-github-username"
npm run deploy
```

Site will be live at: `https://your-username.github.io/physical-ai-book/`

### Deploy Backend to Render

1. Create `render.yaml`:

```yaml
services:
  - type: web
    name: physical-ai-api
    env: python
    buildCommand: pip install -r requirements.txt
    startCommand: uvicorn main:app --host 0.0.0.0 --port $PORT
    envVars:
      - key: OPENAI_API_KEY
        sync: false
      - key: QDRANT_URL
        sync: false
      - key: QDRANT_API_KEY
        sync: false
```

2. Push to GitHub
3. Connect repository in Render dashboard
4. Add environment variables in Render settings
5. Deploy (takes ~5 minutes)

API will be live at: `https://physical-ai-api.onrender.com`

---

## Troubleshooting

### "CORS error when calling backend from frontend"

**Solution**: Add CORS middleware to FastAPI:

```python
# api/main.py
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # Docusaurus dev server
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

### "Qdrant collection not found"

**Solution**: Run indexing script:

```powershell
cd api
python scripts/index_content.py
```

### "OpenAI rate limit exceeded"

**Solution**: Check usage at [platform.openai.com/usage](https://platform.openai.com/usage). If exceeded:

- Wait for rate limit reset (typically 1 minute)
- Upgrade to paid tier for higher limits
- Implement request throttling in `api/rag.py`

### "Docusaurus build fails with 'out of memory'"

**Solution**: Increase Node.js memory:

```powershell
$env:NODE_OPTIONS="--max-old-space-size=4096"
npm run build
```

---

## Next Steps

After completing this quickstart:

1. **Read the constitution**: `.specify/memory/constitution.md` for quality standards
2. **Review the spec**: `specs/001-physical-ai-book/spec.md` for feature requirements
3. **Check the plan**: `specs/001-physical-ai-book/plan.md` for architecture decisions
4. **Explore data model**: `specs/001-physical-ai-book/data-model.md` for entity definitions
5. **Review API contract**: `specs/001-physical-ai-book/contracts/rag-api.openapi.yaml`

**Development checklist before submitting PRs:**

- [ ] All tests pass (`npm test` + `pytest`)
- [ ] Lighthouse score ≥90 for all metrics
- [ ] RAG responses <3s (95th percentile)
- [ ] All new content has citations
- [ ] Diagrams render in light/dark modes
- [ ] Code examples are tested and executable
- [ ] Accessibility: WCAG AA compliance (use axe DevTools)

---

## Getting Help

- **Questions about spec?** Review `specs/001-physical-ai-book/spec.md`
- **Architecture questions?** Review `specs/001-physical-ai-book/research.md`
- **API questions?** Check OpenAPI docs at `http://localhost:8000/docs`
- **Docusaurus questions?** See [docusaurus.io/docs](https://docusaurus.io/docs)
- **FastAPI questions?** See [fastapi.tiangolo.com](https://fastapi.tiangolo.com/)

**Constitutional requirements:**

- All changes must align with `.specify/memory/constitution.md`
- Content must be 2025-accurate with proper citations
- Accessibility and performance standards are non-negotiable
- RAG responses must include source traceability
