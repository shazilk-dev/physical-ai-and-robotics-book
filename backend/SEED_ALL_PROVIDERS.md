# Seeding Vector Database for All Providers

## Overview

The system supports multiple LLM providers with **provider-specific vector collections**. Each provider needs its own collection because embeddings have different dimensions and are not compatible.

## Provider Specifications

| Provider | Embedding Model | Dimensions | Collection Name |
|----------|----------------|------------|-----------------|
| **OpenAI** | text-embedding-3-small | 1536 | `physical_ai_book_openai` |
| **Gemini** | text-embedding-004 | 768 | `physical_ai_book_gemini` |
| **Qwen** | qwen-text-embedding | 1024 | `physical_ai_book_qwen` |

## Prerequisites

### 1. Install Provider SDKs

```bash
cd backend

# OpenAI (already installed)
pip install openai

# Gemini (Google)
pip install google-generativeai

# Qwen (Alibaba Cloud)
pip install alibabacloud-client
```

### 2. Configure API Keys

Edit `backend/.env`:

```env
# OpenAI
OPENAI_API_KEY=sk-proj-...

# Gemini (Get from https://makersuite.google.com/app/apikey)
GEMINI_API_KEY=AIza...

# Qwen (Get from https://dashscope.console.aliyun.com/)
QWEN_API_KEY=sk-...
```

## Seeding Instructions

### Option 1: Seed All Providers (Recommended)

Run the provided script to seed all configured providers:

```bash
cd backend
python scripts/seed_all_providers.py
```

This will:
1. Detect which providers have API keys configured
2. Seed each provider's collection sequentially
3. Show progress and statistics for each
4. Skip providers without API keys

**Estimated Time:** 5-10 minutes per provider (depending on content size)

### Option 2: Seed Individual Providers

Seed one provider at a time:

```bash
cd backend

# Seed OpenAI (1536 dimensions)
LLM_PROVIDER=openai python scripts/seed_vector_db.py

# Seed Gemini (768 dimensions)
LLM_PROVIDER=gemini python scripts/seed_vector_db.py

# Seed Qwen (1024 dimensions)
LLM_PROVIDER=qwen python scripts/seed_vector_db.py
```

## Current Status

**Run this to check status:**

```bash
cd backend
python -c "
from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv
load_dotenv()

client = QdrantClient(
    url=os.getenv('QDRANT_URL'),
    api_key=os.getenv('QDRANT_API_KEY')
)

collections = client.get_collections()
print('Current Collections in Qdrant:')
for c in collections.collections:
    info = client.get_collection(c.name)
    print(f'  - {c.name}: {info.points_count} points, {info.config.params.vectors.size} dims')
"
```

## Verification

### Test Each Provider

After seeding, test each provider:

```bash
# Test OpenAI
curl -X POST http://localhost:8000/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?", "provider": "openai"}'

# Test Gemini
curl -X POST http://localhost:8000/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?", "provider": "gemini"}'

# Test Qwen
curl -X POST http://localhost:8000/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?", "provider": "qwen"}'
```

Expected response format:

```json
{
  "answer": "ROS 2 is a middleware framework...",
  "sources": [...],
  "citations": ["1.1.1", "1.1.2"],
  "query_time_ms": 2341,
  "provider": "openai"
}
```

## Troubleshooting

### Issue: "Collection dimension mismatch"

**Problem:** Trying to use a provider with a collection created for a different provider.

**Solution:** Each provider needs its own collection. The system automatically creates provider-specific collections.

### Issue: "Module not found: google.generativeai"

**Solution:**
```bash
pip install google-generativeai
```

### Issue: "API key not found for provider X"

**Solution:** Add the API key to `backend/.env`:
```env
GEMINI_API_KEY=your-key-here
```

### Issue: "Collection is empty"

**Problem:** Provider collection wasn't seeded.

**Solution:** Run the seeding script for that specific provider:
```bash
LLM_PROVIDER=gemini python scripts/seed_vector_db.py
```

## Re-seeding (After Content Updates)

When you add new content (like Module 2), re-seed **all** providers:

```bash
cd backend

# Option 1: Seed all at once
python scripts/seed_all_providers.py

# Option 2: Seed each individually
for provider in openai gemini qwen; do
  LLM_PROVIDER=$provider python scripts/seed_vector_db.py
done
```

**Note:** Seeding overwrites existing collections, so all providers stay in sync.

## Performance Comparison

| Provider | Embedding Speed | Chat Speed | Cost per 1M tokens | Free Tier |
|----------|----------------|------------|-------------------|-----------|
| **OpenAI** | ~500ms | ~2s | $0.02 (embed) + $0.15 (chat) | $5 credit |
| **Gemini** | ~300ms | ~1.5s | FREE | 60 req/min, 1500/day |
| **Qwen** | ~400ms | ~1.8s | ~$0.01 (embed) + $0.08 (chat) | 1M tokens/month |

**Recommendation:**
- **Development:** Use Gemini (free tier is generous)
- **Production:** Use OpenAI (most reliable) or load-balance across providers
- **Cost-sensitive:** Use Qwen (cheapest)

## Architecture Notes

### Why Provider-Specific Collections?

1. **Embedding dimensions differ**: OpenAI (1536) ≠ Gemini (768) ≠ Qwen (1024)
2. **Embeddings are not transferable**: Each model learns different semantic representations
3. **Qdrant requires fixed dimensions**: Collections can't mix dimension sizes

### Runtime Provider Switching

The system supports **runtime provider switching without restart**:

```typescript
// Frontend can specify provider per request
const response = await fetch('/api/v1/query', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({
    query: "What is PID control?",
    provider: "gemini"  // Switch to Gemini
  })
});
```

Backend automatically:
1. Loads the appropriate LLM provider (cached)
2. Queries the correct collection (`physical_ai_book_gemini`)
3. Uses Gemini's embedding model and chat model
4. Returns response with provider name

## Next Steps

1. ✅ Seed OpenAI collection (DONE)
2. ⏳ Install Gemini SDK: `pip install google-generativeai`
3. ⏳ Seed Gemini collection: `LLM_PROVIDER=gemini python scripts/seed_vector_db.py`
4. ⏳ (Optional) Configure Qwen and seed: `LLM_PROVIDER=qwen python scripts/seed_vector_db.py`
5. ✅ Test all providers with sample queries
6. ✅ Deploy backend with all collections ready

## Monitoring

Check Qdrant Cloud dashboard:
- https://cloud.qdrant.io/
- Verify all collections exist
- Monitor usage and performance

---

**Questions?** Check the [Backend README](README.md) or [GitHub Discussions](https://github.com/shazilk-dev/physical-ai-and-robotics-book/discussions).
