# Render Backend Deployment Guide

## Overview

This guide explains how to deploy the Physical AI & Robotics Book backend API to Render.com with support for multiple LLM providers (OpenAI, Gemini, Qwen).

## Prerequisites

1. **Render Account**: Sign up at [render.com](https://render.com)
2. **API Keys**: Get your API keys from the providers you want to use
3. **Qdrant Cloud**: Set up a vector database at [cloud.qdrant.io](https://cloud.qdrant.io)
4. **Neon PostgreSQL**: (Optional) Database for authentication at [neon.tech](https://neon.tech)

## Required Environment Variables

### Core Variables (Always Required)

```bash
# Qdrant Vector Database
QDRANT_URL=https://your-cluster.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION=physical_ai_textbook

# Default LLM Provider (openai, gemini, or qwen)
LLM_PROVIDER=openai
```

### LLM Provider API Keys

**At least ONE of these must be set:**

```bash
# OpenAI (recommended for production)
OPENAI_API_KEY=sk-proj-...

# Google Gemini (free tier: 60 req/min, 1500 req/day)
GEMINI_API_KEY=AIza...

# Alibaba Qwen (optional)
QWEN_API_KEY=sk-...
```

### Optional Variables

```bash
# Database (for authentication - optional for MVP)
DATABASE_URL=postgresql://user:pass@host/db

# JWT Authentication (optional)
JWT_SECRET=your-secret-key-here
JWT_ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=60

# CORS - Allow your frontend domains
ALLOWED_ORIGINS=["http://localhost:3000","https://your-frontend.vercel.app"]

# Environment
ENVIRONMENT=production
DEBUG=false
```

## Render Deployment Steps

### 1. Create New Web Service

1. Go to Render Dashboard
2. Click **"New +"** → **"Web Service"**
3. Connect your GitHub repository
4. Select the repository: `physical-ai-robotics-book`

### 2. Configure Service Settings

**Basic Settings:**
- **Name**: `physical-ai-robotics-book-api` (or your preferred name)
- **Region**: Choose closest to your users
- **Branch**: `main`
- **Root Directory**: `backend`
- **Runtime**: `Python 3`
- **Build Command**: `pip install -r requirements.txt`
- **Start Command**: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`

**Instance Type:**
- Free tier is sufficient for testing
- Upgrade to Starter ($7/month) for production with 24/7 uptime

### 3. Add Environment Variables

In Render's **Environment** tab, add all required variables:

```bash
# Required
QDRANT_URL=https://68697dd6-cdad-4a81-bc60-d6bbeb8240e8.europe-west3-0.gcp.cloud.qdrant.io:6333
QDRANT_API_KEY=your-actual-key
QDRANT_COLLECTION=physical_ai_textbook
LLM_PROVIDER=openai

# At least one of these
OPENAI_API_KEY=sk-proj-...
GEMINI_API_KEY=AIza...

# Optional
DATABASE_URL=postgresql://...
JWT_SECRET=random-secret-key
ALLOWED_ORIGINS=["https://your-frontend.vercel.app"]
```

⚠️ **Important**: Set `LLM_PROVIDER` to match the API key you're using (`openai`, `gemini`, or `qwen`)

### 4. Deploy

1. Click **"Create Web Service"**
2. Wait for deployment to complete (3-5 minutes)
3. Your backend will be available at: `https://your-service-name.onrender.com`

## Seeding the Vector Database

After deployment, you need to seed the vector database with textbook content.

### Option 1: Local Seeding (Recommended)

```bash
# On your local machine
cd backend

# Set environment variables
export QDRANT_URL="your-qdrant-url"
export QDRANT_API_KEY="your-qdrant-key"
export OPENAI_API_KEY="your-openai-key"  # or GEMINI_API_KEY
export LLM_PROVIDER="openai"  # or "gemini"

# Run seeding script
python scripts/seed_vector_db.py
```

### Option 2: SSH into Render (Paid Plans Only)

```bash
# SSH into Render instance
render ssh your-service-name

# Run seeding script
python scripts/seed_vector_db.py
```

### Multi-Provider Setup

If you want to support multiple providers, seed separate collections:

```bash
# Seed OpenAI collection
export LLM_PROVIDER=openai
python scripts/seed_vector_db.py

# Seed Gemini collection
export LLM_PROVIDER=gemini
python scripts/seed_vector_db.py
```

This creates:
- `physical_ai_textbook_openai` (1536 dimensions)
- `physical_ai_textbook_gemini` (768 dimensions)

## Verifying Deployment

### 1. Test Health Endpoint

```bash
curl https://your-service-name.onrender.com/api/v1/health
```

**Expected Response:**
```json
{
  "status": "healthy",
  "qdrant_connected": true,
  "collections": ["physical_ai_textbook_openai"],
  "provider": "OpenAI",
  "collection_name": "physical_ai_textbook_openai"
}
```

### 2. Test Query Endpoint

```bash
curl -X POST "https://your-service-name.onrender.com/api/v1/query/contextual" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is ROS 2?",
    "num_results": 5,
    "provider": "openai"
  }'
```

**Expected Response:**
```json
{
  "answer": "ROS 2 (Robot Operating System 2) is a middleware framework...",
  "sources": [...],
  "citations": ["1.1.1", "1.1.2"],
  "model": "OpenAI"
}
```

## Frontend Configuration

Update your frontend to use the deployed backend:

**ChatWidget.tsx** (around line 79):
```typescript
if (typeof window !== 'undefined' &&
    (window.location.hostname.includes('vercel.app') ||
     window.location.hostname.includes('physical-ai-robotics-book'))) {
  return 'https://your-service-name.onrender.com/api/v1';
}
```

Or set it in **docusaurus.config.ts**:
```typescript
customFields: {
  apiUrl: 'https://your-service-name.onrender.com/api/v1'
}
```

## Common Issues & Solutions

### Issue 1: 500 Internal Server Error

**Cause**: Missing environment variables or empty vector database

**Solution**:
1. Check Render logs: `Dashboard → your-service → Logs`
2. Verify all required env variables are set
3. Ensure vector database is seeded

### Issue 2: "Collection already exists" Error (409 Conflict)

**Cause**: Concurrent initialization attempts or mismatched dimensions

**Solution**: This is now handled gracefully in the latest code. If you still see this:
1. Pull the latest code with improved error handling
2. Redeploy the service

### Issue 3: CORS Errors from Frontend

**Cause**: Frontend domain not in ALLOWED_ORIGINS

**Solution**:
```bash
# Add your frontend URLs
ALLOWED_ORIGINS=["https://your-frontend.vercel.app","https://your-custom-domain.com"]
```

### Issue 4: Slow First Response (Cold Start)

**Cause**: Render free tier spins down after 15 minutes of inactivity

**Solution**:
- Upgrade to Starter plan ($7/month) for 24/7 uptime
- Or implement a keep-alive ping from your frontend

### Issue 5: Provider Not Found Error

**Cause**: `LLM_PROVIDER` doesn't match available API keys

**Solution**:
```bash
# Make sure these match
LLM_PROVIDER=openai
OPENAI_API_KEY=sk-proj-...  # Must be set if using openai
```

## Cost Estimates

### Render Hosting
- **Free Tier**: $0/month (spins down after 15 min inactivity)
- **Starter**: $7/month (recommended - 24/7 uptime)
- **Standard**: $25/month (more resources)

### LLM Provider Costs

**OpenAI** (Pay-as-you-go):
- Embeddings: $0.00002 per 1K tokens
- GPT-4o-mini: $0.00015 per 1K tokens
- **Est. cost per query**: ~$0.001-0.003

**Google Gemini** (Free Tier):
- 60 requests/minute
- 1,500 requests/day
- **Cost**: FREE for typical usage

**Alibaba Qwen** (Free Tier):
- Similar to Gemini
- Good for Asia-Pacific users

### Qdrant Vector Database
- **Free Tier**: 1GB storage (sufficient for this project)
- Paid: Starts at $25/month

**Total Monthly Cost**: $0-7 (Render) + $0-10 (OpenAI API) = **$0-17/month**

## Monitoring & Logs

### View Logs in Render

```bash
# In Render Dashboard
Dashboard → Your Service → Logs
```

### Key Log Messages

```bash
✅ RAG Service initialized with OpenAI
✅ Collection 'physical_ai_textbook_openai' exists with correct dimension 1536
📥 Contextual Query Request: ...
✅ Query completed in 2341ms
```

### Error Indicators

```bash
❌ RAG Query Error: ...
⚠️  Warning: Collection exists with dimension 768  # Dimension mismatch
❌ Error in contextual_query: ...
```

## Updating Environment Variables

1. Go to Render Dashboard → Your Service → Environment
2. Update the variable value
3. Click **"Save Changes"**
4. Service will automatically redeploy (takes 2-3 minutes)

## Switching LLM Providers

### Runtime Switching (No Redeploy)

The frontend can switch providers without backend changes:

```typescript
// ChatWidget sends provider in request
const requestPayload = {
  question: "What is ROS 2?",
  provider: "gemini"  // or "openai" or "qwen"
};
```

### Changing Default Provider

Update `LLM_PROVIDER` environment variable in Render:

```bash
LLM_PROVIDER=gemini  # Change from openai to gemini
```

⚠️ **Remember**: You need the corresponding API key set and the collection seeded for that provider.

## Production Checklist

- [ ] All required environment variables set in Render
- [ ] Vector database seeded with content
- [ ] Health endpoint returns "healthy"
- [ ] Test query endpoint returns valid responses
- [ ] Frontend connected to production backend URL
- [ ] CORS configured with frontend domains
- [ ] Monitoring/logging enabled
- [ ] Secrets rotated from development values
- [ ] HTTPS enforced (Render does this automatically)
- [ ] Rate limiting considered (if using free tier LLMs)

## Support

For issues:
1. Check Render logs first
2. Verify environment variables
3. Test endpoints with curl
4. Check GitHub Issues: [github.com/your-repo/issues](https://github.com)

---

**Last Updated**: 2025-12-17
**Render Version**: Compatible with Render v2
**Backend Version**: 0.2.0
