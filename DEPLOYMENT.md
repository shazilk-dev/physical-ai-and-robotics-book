# 🚀 Deployment Guide - Physical AI Textbook

## ⚠️ VERCEL DEPLOYMENT FIX (December 11, 2025)

### Problem: Works Locally But Not on Vercel

- ✅ Sidebar collapse works locally
- ✅ ChatBot visible locally
- ❌ Sidebar doesn't collapse on Vercel
- ❌ ChatBot not visible on Vercel

### Root Cause: **Vercel is caching old build files**

### ✅ Solution Applied:

1. Updated `vercel.json` with `npm run clear && npm run build`
2. Added `Cache-Control` headers to force re-validation
3. Fixed CSS z-index for ChatBot (999999)
4. Added JavaScript fix in Root.tsx

### 🚨 IMMEDIATE ACTION REQUIRED:

**Step 1: Commit Changes**

```bash
git add .
git commit -m "Fix: Vercel cache busting and UI fixes"
git push origin main
```

**Step 2: Force Redeploy in Vercel**

1. Go to https://vercel.com/dashboard
2. Select your project
3. Go to latest deployment → Click **"⋯"** → **"Redeploy"**
4. **UNCHECK** "Use existing Build Cache" ✅ THIS IS CRITICAL
5. Click **"Redeploy"**

**Step 3: Clear Browser Cache**

- Hard refresh: **Ctrl+Shift+R** (Windows) or **Cmd+Shift+R** (Mac)
- Or open in **Incognito mode**

**Expected Result:**

- ✅ Sidebar categories collapse/expand
- ✅ ChatBot button visible (bottom-right)
- ✅ Everything works like localhost

---

## Overview

This project has **2 deployments**:

1. **Frontend** (Docusaurus) → Vercel
2. **Backend** (FastAPI RAG API) → Railway

**What gets deployed:**

- ✅ `frontend/` → Static site on Vercel
- ✅ `backend/` → API server on Railway
- ✅ `labs/` → Included in frontend static files
- ❌ `specs/`, `_archive/` → Not deployed (docs only)

---

## 🎯 Quick Deploy (15 minutes)

### Prerequisites

- GitHub repository pushed
- Vercel account (free)
- Railway account (free)
- OpenAI API key
- Qdrant Cloud cluster (free tier)

---

## Step 1: Deploy Backend to Railway (5 min)

### 1.1 Sign Up & Connect GitHub

```
1. Go to https://railway.app/
2. Sign up with GitHub
3. Click "New Project"
4. Select "Deploy from GitHub repo"
5. Choose: physical-ai-robotics-book
```

### 1.2 Configure Service

**Root Directory:**

```
backend
```

**Build Command:** (auto-detected)

```
pip install -r requirements.txt
```

**Start Command:** (auto-detected)

```
uvicorn app.main:app --host 0.0.0.0 --port $PORT
```

### 1.3 Set Environment Variables

In Railway dashboard → Variables:

```bash
OPENAI_API_KEY=sk-proj-xxxxx
QDRANT_URL=https://xxxxx.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-key
QDRANT_COLLECTION=physical_ai_textbook
```

### 1.4 Deploy & Get URL

```
1. Click "Deploy"
2. Wait 2-3 minutes
3. Copy your URL: https://your-backend.up.railway.app
4. Test: https://your-backend.up.railway.app/health
```

**Expected Response:**

```json
{
  "status": "healthy",
  "qdrant_connected": true,
  "collections": ["physical_ai_textbook"]
}
```

### 1.5 Seed Vector Database

**Option A: Local seed → Remote Qdrant**

```bash
cd backend
# Update .env with production QDRANT_URL and QDRANT_API_KEY
python scripts/seed_vector_db.py
```

**Option B: Railway CLI**

```bash
# Install Railway CLI
npm i -g @railway/cli

# Login
railway login

# Link project
railway link

# Run seed script
railway run python scripts/seed_vector_db.py
```

---

## Step 2: Deploy Frontend to Vercel (3 min)

### 2.1 Import Project

```
1. Go to https://vercel.com/
2. Sign up with GitHub
3. Click "Add New Project"
4. Import: physical-ai-robotics-book
```

### 2.2 Configure Build

**Framework Preset:**

```
Docusaurus
```

**Root Directory:**

```
frontend
```

**Build Command:**

```
npm run build
```

**Output Directory:**

```
build
```

**Install Command:**

```
npm install
```

### 2.3 Set Environment Variable

In Vercel → Settings → Environment Variables:

```bash
# Replace with your Railway backend URL
NEXT_PUBLIC_API_URL=https://your-backend.up.railway.app/api/v1
```

### 2.4 Deploy

```
1. Click "Deploy"
2. Wait 2-3 minutes
3. Your site is live: https://your-project.vercel.app
```

---

## Step 3: Test Deployment

### 3.1 Test Frontend

```
1. Open: https://your-project.vercel.app
2. Navigate to Module 1
3. Check diagrams render correctly
4. Check code examples have syntax highlighting
```

### 3.2 Test ChatWidget

```
1. Click 💬 floating button (bottom-right)
2. Ask: "What is ROS 2?"
3. Verify:
   - Response appears (2-4 seconds)
   - Citations show (e.g., "1.1.1 Architecture")
   - Clicking citation navigates to section
```

### 3.3 Test Backend API

```bash
# Health check
curl https://your-backend.up.railway.app/api/v1/health

# Stats
curl https://your-backend.up.railway.app/api/v1/stats

# Query
curl -X POST https://your-backend.up.railway.app/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?", "num_results": 5}'
```

---

## 🔧 Troubleshooting

### Frontend Issues

**Problem: Build fails on Vercel**

```
Solution:
1. Check Node.js version matches local (18+)
2. Verify all dependencies in package.json
3. Check build logs for specific error
4. Ensure docusaurus.config.ts has correct baseUrl
```

**Problem: ChatWidget not appearing**

```
Solution:
1. Check browser console for errors
2. Verify Root.tsx is in src/theme/
3. Clear Vercel cache and redeploy
4. Check ChatWidget.tsx imported correctly
```

**Problem: API calls fail (CORS error)**

```
Solution:
1. Check backend ALLOWED_ORIGINS in settings.py
2. Add Vercel URL to ALLOWED_ORIGINS list:
   ALLOWED_ORIGINS = [
       "http://localhost:3000",
       "https://your-project.vercel.app"
   ]
3. Redeploy backend
```

### Backend Issues

**Problem: Backend won't start**

```
Solution:
1. Check Railway logs for Python errors
2. Verify requirements.txt has all dependencies
3. Check start command uses $PORT variable
4. Ensure Python version 3.10+ in railway.toml
```

**Problem: Qdrant connection fails**

```
Solution:
1. Verify QDRANT_URL and QDRANT_API_KEY are set
2. Check Qdrant Cloud dashboard (cluster active?)
3. Test connection locally first
4. Ensure collection is seeded
```

**Problem: OpenAI API errors**

```
Solution:
1. Verify OPENAI_API_KEY is valid
2. Check API usage quota (not exceeded?)
3. Test key with curl:
   curl https://api.openai.com/v1/models \
     -H "Authorization: Bearer $OPENAI_API_KEY"
```

---

## 📊 Cost Estimates

### Vercel (Frontend)

- **Free Tier:** 100 GB bandwidth, unlimited builds
- **Cost:** $0/month for hobby projects
- **Limits:** 1 commercial project, 3 team members

### Railway (Backend)

- **Free Tier:** $5 credit/month, 500 hours
- **Cost:** ~$0-5/month (likely free for demos)
- **After free:** $0.000231/GB-hour RAM + $0.000463/vCPU-hour

### OpenAI API

- **Embeddings:** $0.00002/1K tokens (~$0.002 per seeding)
- **GPT-4o-mini:** $0.15/1M input + $0.60/1M output tokens
- **Estimate:** ~$1-5/month for development + demos

### Qdrant Cloud

- **Free Tier:** 1GB cluster, 1M vectors
- **Cost:** $0/month
- **Sufficient for:** Module 1 (~90 chunks = ~0.5MB)

**Total Monthly Cost:** ~$0-10 for hackathon/demo 💰

---

## 🔄 Continuous Deployment

Both platforms auto-deploy on git push:

```bash
# Make changes locally
git add .
git commit -m "Update Module 1 content"
git push origin 001-physical-ai-book

# Automatic deployments:
# - Vercel rebuilds frontend (2-3 min)
# - Railway redeploys backend (1-2 min)
```

---

## 🌍 Custom Domain (Optional)

### Vercel Custom Domain

```
1. Go to Vercel → Settings → Domains
2. Add domain: physical-ai-textbook.com
3. Update DNS records (Vercel provides values)
4. Wait 24-48 hours for DNS propagation
```

### Railway Custom Domain

```
1. Go to Railway → Settings → Networking
2. Add custom domain: api.physical-ai-textbook.com
3. Update DNS CNAME record
4. Update ALLOWED_ORIGINS in backend
5. Update NEXT_PUBLIC_API_URL in Vercel
```

---

## ✅ Deployment Checklist

Before going live:

### Pre-Deploy

- [ ] Seed Qdrant database (85+ chunks)
- [ ] Test backend locally (`uvicorn app.main:app --reload`)
- [ ] Test frontend locally (`npm run start`)
- [ ] Test ChatWidget queries work
- [ ] Check all environment variables set

### Backend Deploy

- [ ] Railway project created
- [ ] Environment variables configured
- [ ] Backend deployed successfully
- [ ] Health check returns 200
- [ ] Qdrant connection verified
- [ ] Test query endpoint with curl

### Frontend Deploy

- [ ] Vercel project imported
- [ ] Build settings configured
- [ ] NEXT_PUBLIC_API_URL set
- [ ] Site deployed successfully
- [ ] All pages load correctly
- [ ] ChatWidget appears and works
- [ ] Citations navigate correctly

### Post-Deploy

- [ ] Test end-to-end flow (ask question → get answer → click citation)
- [ ] Check mobile responsiveness
- [ ] Verify HTTPS (not HTTP)
- [ ] Test from different browsers
- [ ] Record demo video
- [ ] Update README with live URLs

---

## 📝 Environment Variables Summary

### Backend (Railway)

```bash
OPENAI_API_KEY=sk-proj-xxxxx
QDRANT_URL=https://xxxxx.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-key
QDRANT_COLLECTION=physical_ai_textbook
```

### Frontend (Vercel)

```bash
NEXT_PUBLIC_API_URL=https://your-backend.up.railway.app/api/v1
```

---

## 🎓 Alternative Platforms

### Backend Alternatives to Railway

**Render.com:**

- Free tier: 750 hours/month
- Auto-deploy from GitHub
- Similar to Railway

**Fly.io:**

- Free tier: 3 shared-CPU VMs
- Global edge deployment
- Slightly more complex setup

**Heroku:**

- Free tier removed (now $7/month minimum)
- Not recommended for hackathons

### Frontend Alternatives to Vercel

**Netlify:**

- Similar to Vercel
- 100 GB bandwidth free
- Good Docusaurus support

**GitHub Pages:**

- Free for public repos
- Static sites only (perfect for Docusaurus)
- Slower builds than Vercel

**Cloudflare Pages:**

- Unlimited bandwidth (free)
- Fast global CDN
- Good for static sites

---

## 🚀 Ready to Deploy?

**Your deployment URLs will be:**

- Frontend: `https://physical-ai-textbook.vercel.app`
- Backend: `https://physical-ai-textbook-api.up.railway.app`

**Time to deploy:** ~15 minutes total

**Need help?** Check Railway/Vercel docs or ask in their Discord communities.

Good luck with your hackathon demo! 🎉
