# Backend RAG System - Quick Start Guide

## Prerequisites

1. **Environment Variables** - Create `.env` file in `backend/` directory:

```env
# OpenAI API
OPENAI_API_KEY=sk-your-key-here

# Qdrant Cloud (or local)
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-api-key-here
QDRANT_COLLECTION=physical_ai_textbook

# Optional (for future features)
DATABASE_URL=postgresql://...
```

2. **Python Dependencies** - Already in `requirements.txt`:
   - fastapi 0.109.0
   - qdrant-client 1.7.3
   - openai 1.10.0
   - tiktoken (for token counting)
   - uvicorn (ASGI server)

## Step 1: Seed Vector Database

This script reads all Module 1 markdown files, chunks them, generates embeddings, and uploads to Qdrant.

```bash
cd backend
python scripts/seed_vector_db.py
```

**Expected Output:**

```
🚀 Starting Vector Database Seeding
============================================================

🔧 Initializing RAG service...
🗄️  Creating collection: physical_ai_textbook

📂 Scanning directory: F:\PROJECTS\ai-book\physical-ai-robotics-book\frontend\docs
📋 Found 9 markdown files
📋 Including intro.md

📄 Processing: intro.md
   ✂️  Created 8 chunks
   📤 Uploaded chunk 1/8
   ...
   📤 Uploaded chunk 8/8

📄 Processing: overview.md
   ✂️  Created 12 chunks
   ...

📄 Processing: 1.1.1-architecture.md
   ✂️  Created 15 chunks
   ...

============================================================
✅ Seeding complete!
📊 Files processed: 10
📊 Total chunks uploaded: 87
📊 Collection points: 87
```

**What it does:**

- Reads all `.md` files from `frontend/docs/module-01-ros2/` and `intro.md`
- Extracts metadata (module, chapter, section) from file paths
- Chunks text into 500-token segments with 50-token overlap
- Generates embeddings using OpenAI `text-embedding-3-small` (1536 dimensions)
- Uploads to Qdrant with metadata for filtering and citations

## Step 2: Start FastAPI Server

```bash
cd backend
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

**Expected Output:**

```
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [12345] using WatchFiles
INFO:     Started server process [67890]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

Server will be available at: `http://localhost:8000`

## Step 3: Test Endpoints

### Health Check

```bash
curl http://localhost:8000/api/v1/health
```

**Expected Response:**

```json
{
  "status": "healthy",
  "qdrant_connected": true,
  "collections": ["physical_ai_textbook"]
}
```

### Query RAG System

```bash
curl -X POST http://localhost:8000/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is ROS 2 and why do we need it?",
    "module": "module-01-ros2",
    "num_results": 5
  }'
```

**Expected Response:**

```json
{
  "answer": "ROS 2 (Robot Operating System 2) is a middleware framework that acts as the nervous system for robots...",
  "sources": [
    {
      "content": "...relevant text chunk...",
      "module": "module-01-ros2",
      "chapter": "1.1",
      "section": "1.1.1",
      "section_title": "Architecture",
      "file_path": "frontend/docs/module-01-ros2/ros2-fundamentals/1.1.1-architecture.md",
      "score": 0.892
    }
  ],
  "citations": ["1.1.1 Architecture", "1.1.2 Rclpy Patterns"],
  "model": "gpt-4o-mini"
}
```

### Get Database Stats

```bash
curl http://localhost:8000/api/v1/stats
```

**Expected Response:**

```json
{
  "collection_name": "physical_ai_textbook",
  "points_count": 87,
  "vectors_count": 87,
  "indexed": "green"
}
```

## Step 4: Test Sample Queries

### Query 1: ROS 2 Basics

```bash
curl -X POST http://localhost:8000/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{"question": "How do I create a publisher in ROS 2?"}'
```

### Query 2: URDF

```bash
curl -X POST http://localhost:8000/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What is URDF used for?"}'
```

### Query 3: Sensors

```bash
curl -X POST http://localhost:8000/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{"question": "How do I add a camera to my robot in URDF?"}'
```

## API Documentation

FastAPI provides automatic interactive documentation:

- **Swagger UI**: http://localhost:8000/docs
- **ReDoc**: http://localhost:8000/redoc

## Troubleshooting

### Error: "OpenAI API key not found"

**Solution**: Set `OPENAI_API_KEY` in `.env` file

### Error: "Could not connect to Qdrant"

**Solution**:

- Check `QDRANT_URL` and `QDRANT_API_KEY` in `.env`
- For local Qdrant: `QDRANT_URL=http://localhost:6333` (no API key needed)
- For Qdrant Cloud: Get URL and API key from dashboard

### Error: "Collection not found"

**Solution**: Run seeding script first: `python scripts/seed_vector_db.py`

### Error: "No relevant chunks found"

**Solution**:

- Check if seeding completed successfully
- Verify question is related to Module 1 content
- Try broader or more specific queries

## Next Steps

1. **Frontend Integration**: Create ChatWidget component in Docusaurus
2. **More Content**: Seed additional modules (2-4) when ready
3. **Enhancements**:
   - Add conversation history
   - Implement feedback mechanism
   - Add query analytics
   - Optimize chunk size and overlap

## Architecture Overview

```
User Query → FastAPI (/api/v1/query)
    ↓
RAGService.query()
    ↓
1. Generate embedding (OpenAI)
2. Search Qdrant (vector similarity)
3. Retrieve top-k chunks with metadata
    ↓
4. Build context from chunks
5. Generate response (GPT-4o-mini)
6. Extract citations
    ↓
Return {answer, sources, citations}
```

## Performance Notes

- **Embedding generation**: ~100-200ms per query
- **Qdrant search**: ~50-100ms for 5 results
- **GPT response**: ~1-3 seconds
- **Total latency**: ~2-4 seconds per query

## Cost Estimates (OpenAI)

- **Embeddings**: $0.00002 per 1K tokens (~$0.002 per seeding run)
- **GPT-4o-mini**: $0.15 per 1M input tokens, $0.60 per 1M output tokens
- **Estimated per query**: $0.001-0.003

For hackathon demo: Total cost ~$1-5 for development and testing.
