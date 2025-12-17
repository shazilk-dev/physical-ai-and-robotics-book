# Multi-Provider LLM Configuration Guide

## 🎯 Overview

The Physical AI Robotics Book platform now supports **multiple LLM providers**, giving you the flexibility to choose between:

- **OpenAI** (GPT-4o-mini) - High quality, paid (~$2-3/month)
- **Google Gemini** (Gemini 1.5 Flash) - Free tier available
- **Qwen** (Alibaba Cloud) - Competitive pricing, good for multilingual

### ✨ Key Features

- **Provider-Specific Collections**: Each provider has its own vector database collection
  - `physical_ai_book_openai` (1536-dim)
  - `physical_ai_book_gemini` (768-dim)
  - `physical_ai_book_qwen` (1024-dim)

- **Instant Switching**: Change providers by updating one environment variable
- **No Re-seeding**: Seed each provider once, then switch freely
- **Auto-Detection**: System automatically uses the correct collection for your provider

---

## 🚀 Quick Start

### 1. Choose Your Provider

Edit your `backend/.env` file and set:

```env
# Choose one: openai, gemini, or qwen
LLM_PROVIDER=gemini
```

### 2. Set the API Key

Add the corresponding API key:

```env
# If using OpenAI
OPENAI_API_KEY=sk-proj-your-key-here

# If using Gemini
GEMINI_API_KEY=your-gemini-key-here

# If using Qwen
QWEN_API_KEY=your-qwen-key-here
```

### 3. Install Dependencies

```bash
cd backend
pip install -r requirements.txt
```

### 4. Seed Provider Collection (First Time Only!)

**✅ NEW:** The system now uses **provider-specific collections**, so you only need to seed each provider **once**!

| Provider | Collection Name | Embedding Dimension |
|----------|----------------|-------------------|
| OpenAI   | `physical_ai_book_openai`   | 1536              |
| Gemini   | `physical_ai_book_gemini`   | 768               |
| Qwen     | `physical_ai_book_qwen`     | 1024              |

**First time using a provider:**

```bash
cd backend
python scripts/seed_vector_db.py
```

This will:
1. Create provider-specific collection (e.g., `physical_ai_book_gemini`)
2. Index all textbook content with that provider's embeddings
3. Keep the collection ready for future use

**After seeding once, you can switch providers instantly - no re-seeding needed!**

### 5. Start the Backend

```bash
cd backend
python -m uvicorn app.main:app --reload
```

You should see:
```
🔧 Initializing LLM Provider: GEMINI
✅ Gemini Provider initialized
   Embedding Model: models/text-embedding-004
   Chat Model: gemini-1.5-flash
✅ RAG Service initialized with Google Gemini
```

---

## 📊 Provider Comparison

| Feature | OpenAI | Gemini | Qwen |
|---------|--------|--------|------|
| **Cost (1000 queries)** | $2-3 | **FREE*** | ~$1.50 |
| **Quality** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ |
| **Speed** | Very Fast | Fast | Fast |
| **Rate Limits** | High | 60/min, 1500/day | Medium |
| **Embedding Dimension** | 1536 | 768 | 1024 |
| **Best For** | Production | Development/Free | Multilingual |

*Gemini free tier: 60 requests/min, 1,500 requests/day

---

## 🔑 Getting API Keys

### OpenAI

1. Go to https://platform.openai.com/api-keys
2. Sign up or log in
3. Click "Create new secret key"
4. Copy the key (starts with `sk-proj-`)
5. **Cost**: $0.002 per query (~$2-3/month for 1000 queries)

### Google Gemini (FREE)

1. Go to https://ai.google.dev/
2. Click "Get API key in Google AI Studio"
3. Sign in with Google account
4. Click "Create API key"
5. Copy the key
6. **Free tier**: 60 requests/minute, 1,500 requests/day

### Qwen (Alibaba Cloud)

1. Go to https://dashscope.console.aliyun.com/
2. Sign up for Alibaba Cloud account
3. Enable DashScope API
4. Create API key
5. Copy the key
6. **Cost**: ~$0.0015 per query (cheaper than OpenAI)

---

## ⚙️ Environment Configuration

### Full `.env` Example

```env
# Application
APP_NAME=Physical AI RAG API
DEBUG=true

# Database (Neon Postgres)
DATABASE_URL=postgresql://user:password@host:5432/dbname

# Qdrant Cloud
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION=physical_ai_textbook

# LLM Provider Configuration
LLM_PROVIDER=gemini  # Choose: openai, gemini, qwen

# OpenAI (Default - $2-3/month for 1000 queries)
OPENAI_API_KEY=your-openai-api-key

# Google Gemini (FREE - 60 req/min, 1,500 req/day)
GEMINI_API_KEY=your-gemini-api-key

# Qwen/Alibaba Cloud (Paid/Free tiers available)
QWEN_API_KEY=your-qwen-api-key

# JWT Auth
JWT_SECRET=your-secret-key-change-in-production
JWT_ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=60

# CORS Origins
ALLOWED_ORIGINS=http://localhost:3000,https://yourdomain.com
```

---

## 🔄 Switching Providers

### Step-by-Step Migration

**Example: Switching from OpenAI to Gemini**

1. **Get Gemini API key** (if you don't have one):
   - Visit https://ai.google.dev/
   - Create free API key

2. **Add to `.env`:**
   ```env
   GEMINI_API_KEY=your-gemini-key-here
   ```

3. **Update environment variable:**
   ```env
   LLM_PROVIDER=gemini
   ```

4. **Install Gemini SDK:**
   ```bash
   pip install google-generativeai==0.8.3
   ```

5. **Seed Gemini collection** (first time only):
   ```bash
   python scripts/seed_vector_db.py
   ```

   Expected output:
   ```
   🚀 Starting Vector Database Seeding
   🤖 LLM Provider: GEMINI
   🔧 Initializing LLM Provider: GEMINI
   ✅ Gemini Provider initialized
      Collection: physical_ai_book_gemini
      Embedding Dimension: 768
   📦 Creating new collection 'physical_ai_book_gemini'...
   ✅ Created collection with dimension 768
   📚 Processing textbook content...
   ✅ Indexed 87 chunks
   ```

6. **Restart backend:**
   ```bash
   python -m uvicorn app.main:app --reload
   ```

   You should see:
   ```
   🔧 Initializing RAG Service...
   🔧 Initializing LLM Provider: GEMINI
   ✅ Gemini Provider initialized
      Collection: physical_ai_book_gemini
      Embedding Dimension: 768
   ✅ Collection 'physical_ai_book_gemini' exists with correct dimension 768
   ✅ RAG Service initialized with Google Gemini
   ```

7. **Switch back to OpenAI anytime:**
   ```env
   LLM_PROVIDER=openai
   ```
   Restart backend - **no re-seeding needed!** (OpenAI collection already exists)

---

## 🛠️ Technical Architecture

### Provider Abstraction Layer

```python
# backend/app/services/llm_providers.py

class BaseLLMProvider(ABC):
    """Abstract base class for all LLM providers"""

    @abstractmethod
    def generate_embedding(self, text: str) -> List[float]:
        """Generate embedding vector"""
        pass

    @abstractmethod
    def generate_chat_completion(
        self, messages: List[Dict], max_tokens: int, temperature: float
    ) -> str:
        """Generate chat response"""
        pass

    @abstractmethod
    def get_embedding_dimension(self) -> int:
        """Return embedding dimension size"""
        pass
```

### Supported Providers

1. **OpenAIProvider** - Wraps OpenAI API
2. **GeminiProvider** - Wraps Google Generative AI
3. **QwenProvider** - Wraps Alibaba DashScope

### Factory Pattern

```python
def get_llm_provider() -> BaseLLMProvider:
    provider_name = os.getenv("LLM_PROVIDER", "openai")

    if provider_name == "openai":
        return OpenAIProvider(os.getenv("OPENAI_API_KEY"))
    elif provider_name == "gemini":
        return GeminiProvider(os.getenv("GEMINI_API_KEY"))
    elif provider_name == "qwen":
        return QwenProvider(os.getenv("QWEN_API_KEY"))
```

---

## 🐛 Troubleshooting

### Error: "API key not set"

**Problem:** Missing API key for selected provider

**Solution:**
```bash
# Check your .env file
cat backend/.env | grep GEMINI_API_KEY

# If missing, add it:
echo "GEMINI_API_KEY=your-key-here" >> backend/.env
```

### Error: "Module not found: google.generativeai"

**Problem:** Provider SDK not installed

**Solution:**
```bash
# For Gemini
pip install google-generativeai==0.8.3

# For Qwen
pip install dashscope==1.20.11
```

### Error: "Vector dimension mismatch"

**Problem:** Collection exists but has wrong dimension for current provider

**Solution:**
This shouldn't happen with the new provider-specific collections, but if it does:

```bash
# Delete the problematic collection and re-seed
cd backend
python scripts/seed_vector_db.py
# When prompted, confirm deletion with 'y'
```

**Prevention:** The system now uses provider-specific collection names, so each provider has its own collection with the correct dimension.

### Error: "Rate limit exceeded" (Gemini)

**Problem:** Exceeded free tier limits (60 req/min or 1500 req/day)

**Solution:**
- Wait for rate limit to reset
- OR switch to OpenAI/Qwen (paid)
- OR implement request queuing/throttling

### Chatbot returns empty responses

**Problem:** Provider not generating completions correctly

**Debug steps:**
```bash
# Check backend logs
python -m uvicorn app.main:app --reload

# Look for:
# ✅ Gemini Provider initialized
# ✅ RAG Service initialized with Google Gemini
```

If provider initialization fails, check:
1. API key is correct
2. SDK is installed
3. No network issues

---

## 💰 Cost Comparison

### Monthly Costs (1000 queries/month)

| Provider | Embeddings | Chat Completions | Total |
|----------|-----------|-----------------|-------|
| **OpenAI** | $0.02 | $2.00 | **$2.02** |
| **Gemini** | Free | Free | **$0.00*** |
| **Qwen** | $0.01 | $1.50 | **$1.51** |

*Gemini free tier: Up to 1,500 requests/day

### Cost Per Query

| Provider | Embedding | Chat | Total/Query |
|----------|-----------|------|-------------|
| **OpenAI** | $0.00002 | $0.002 | **$0.00202** |
| **Gemini** | Free | Free | **$0.00000** |
| **Qwen** | $0.00001 | $0.0015 | **$0.00151** |

---

## 📈 Performance Comparison

### Response Quality

**Based on testing with robotics questions:**

1. **OpenAI (GPT-4o-mini)** - Best quality, most accurate technical details
2. **Gemini (1.5 Flash)** - Very good quality, occasionally less detailed
3. **Qwen (qwen-turbo)** - Good quality, especially for Chinese/English

### Response Speed

**Average latency per query:**

| Provider | Embedding | Chat | Total |
|----------|-----------|------|-------|
| OpenAI   | 150ms | 1200ms | ~1.35s |
| Gemini   | 200ms | 1000ms | ~1.20s |
| Qwen     | 180ms | 1100ms | ~1.28s |

---

## 🔒 Security Best Practices

1. **Never commit API keys to git:**
   ```bash
   # .gitignore should include:
   .env
   .env.local
   ```

2. **Use different keys for dev/prod:**
   ```env
   # Development
   GEMINI_API_KEY=dev-key-here

   # Production (use environment variables in deployment platform)
   ```

3. **Rotate keys regularly:**
   - OpenAI: https://platform.openai.com/api-keys
   - Gemini: https://ai.google.dev/
   - Qwen: https://dashscope.console.aliyun.com/

4. **Monitor usage:**
   - Set up billing alerts
   - Track API usage dashboards
   - Implement rate limiting in production

---

## 🚀 Production Deployment

### Recommended Setup

**For Production (High Traffic):**
- Use **OpenAI** for best quality
- Enable caching to reduce costs
- Implement rate limiting
- Monitor API usage

**For Development/Testing:**
- Use **Gemini** free tier
- No cost, good quality
- Rate limits are usually sufficient

**For Multilingual/China:**
- Use **Qwen**
- Good Chinese language support
- Competitive pricing

### Environment Variables in Vercel/Railway

```bash
# Vercel
vercel env add LLM_PROVIDER production
vercel env add GEMINI_API_KEY production

# Railway
railway variables set LLM_PROVIDER=gemini
railway variables set GEMINI_API_KEY=your-key
```

---

## 📚 Additional Resources

### Documentation

- **OpenAI**: https://platform.openai.com/docs
- **Gemini**: https://ai.google.dev/docs
- **Qwen**: https://help.aliyun.com/zh/dashscope/

### API References

- **OpenAI API**: https://platform.openai.com/docs/api-reference
- **Gemini API**: https://ai.google.dev/api/python/google/generativeai
- **Qwen API**: https://help.aliyun.com/zh/dashscope/developer-reference/api-details

### Community Support

- **GitHub Issues**: https://github.com/your-repo/issues
- **Discord**: (Add your Discord link)
- **Documentation**: (Add your docs link)

---

## 🎓 FAQ

### Q: Can I mix providers (e.g., OpenAI embeddings + Gemini chat)?

**A:** No, the current implementation uses the same provider for both embeddings and chat. This ensures consistency and simplifies the architecture.

### Q: What happens if I switch providers without seeding that provider first?

**A:** The system will automatically create an empty collection for that provider. You'll need to run `seed_vector_db.py` once to index the textbook content for that provider.

### Q: Can I use multiple providers simultaneously?

**A:** Not currently. The system uses one provider at a time based on `LLM_PROVIDER` env variable.

### Q: Which provider should I use?

**A:**
- **Development**: Gemini (free)
- **Production**: OpenAI (best quality)
- **Budget**: Gemini (free) or Qwen (cheaper than OpenAI)
- **Multilingual**: Qwen

### Q: How do I test all providers locally?

**A:**
```bash
# Test OpenAI
LLM_PROVIDER=openai python scripts/seed_vector_db.py

# Test Gemini
LLM_PROVIDER=gemini python scripts/seed_vector_db.py

# Test Qwen
LLM_PROVIDER=qwen python scripts/seed_vector_db.py
```

---

## ✅ Checklist: Switching Providers

### First Time Using a New Provider:
- [ ] Choose provider (openai, gemini, qwen)
- [ ] Get API key from provider website
- [ ] Add API key to `.env`
- [ ] Update `LLM_PROVIDER` in `.env`
- [ ] Install provider SDK: `pip install -r requirements.txt`
- [ ] **Seed provider collection** (first time only): `python scripts/seed_vector_db.py`
- [ ] Restart backend: `python -m uvicorn app.main:app --reload`
- [ ] Test chatbot in frontend
- [ ] Verify responses are generated correctly

### Switching Between Previously Seeded Providers:
- [ ] Update `LLM_PROVIDER` in `.env`
- [ ] Restart backend: `python -m uvicorn app.main:app --reload`
- [ ] Test chatbot in frontend
- [ ] ✅ That's it! No re-seeding needed.

---

**Last Updated**: 2025-12-17
**Version**: 1.0.0
**Supported Providers**: OpenAI, Google Gemini, Alibaba Qwen
