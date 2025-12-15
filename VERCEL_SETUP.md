# Vercel Environment Variable Setup

## Quick Setup Guide

### Step 1: Add Environment Variable in Vercel

1. Go to: https://vercel.com/dashboard
2. Select your project: **physical-ai-robotics-book**
3. Click **Settings** → **Environment Variables**
4. Add new variable:
   - **Name:** `NEXT_PUBLIC_API_URL`
   - **Value:** `https://physical-ai-and-robotics-book.onrender.com/api/v1`
   - **Environments:** Check **Production** and **Preview**
5. Click **Save**

### Step 2: Redeploy Frontend

1. Go to **Deployments** tab
2. Click on the latest deployment
3. Click **...** menu → **Redeploy**
4. Check **"Clear Build Cache and Redeploy"**
5. Click **Redeploy**

### Step 3: Test Integration

After redeployment completes:
1. Visit: https://physical-ai-robotics-book.vercel.app
2. Click the chat widget (bottom-right corner)
3. Ask: "What is ROS 2?"
4. Should receive AI-powered response with citations

---

## Files Changed

### Frontend
- ✅ `.env.production` - Backend URL for production builds
- ✅ `.env.local` - Backend URL for local development  
- ✅ `.env.example` - Template for other developers

### Backend  
- ✅ `app/main.py` - CORS configuration
- ✅ `app/config/settings.py` - Added ENVIRONMENT variable
- ✅ `requirements.txt` - Added email-validator

---

## Environment Variables Summary

### Vercel (Frontend)
```
NEXT_PUBLIC_API_URL=https://physical-ai-and-robotics-book.onrender.com/api/v1
```

### Render (Backend)
Add these if not already set:
```
ENVIRONMENT=production
OPENAI_API_KEY=sk-...
QDRANT_URL=https://...
QDRANT_API_KEY=...
DATABASE_URL=postgresql://...
JWT_SECRET=... (generate with: openssl rand -hex 32)
```

---

## Testing Commands

### Test Backend Health
```bash
curl https://physical-ai-and-robotics-book.onrender.com/health
```

### Test RAG API
```bash
curl -X POST https://physical-ai-and-robotics-book.onrender.com/api/v1/query \
  -H "Content-Type: application/json" \
  -d "{\"question\": \"What is ROS 2?\", \"num_results\": 3}"
```

---

## Commit and Push

```bash
git add frontend/.env.production frontend/.env.local frontend/.env.example
git add backend/app/main.py backend/app/config/settings.py backend/requirements.txt
git commit -m "Configure frontend-backend integration with Render"
git push origin main
```

**Both services will auto-deploy!**
