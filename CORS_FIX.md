# CORS & Backend Configuration Fix

## Problems Found

### 1. CORS Credentials Conflict ❌
**Error:**
```
Access-Control-Allow-Origin: * with credentials mode 'include' is not allowed
```

**Root Cause:** 
- Backend was using wildcard `*` for CORS origins
- Frontend sends `credentials: 'include'` for cookies/auth
- Browsers block wildcard + credentials for security

### 2. Backend 500 Error ❌
**Error:**
```
500 Internal Server Error on /api/v1/query
```

**Root Cause:**
- Backend environment variables not set in Render
- Qdrant/OpenAI not configured

---

## ✅ Solutions Implemented

### Fix 1: CORS Configuration (Backend)

**Updated:** `backend/app/main.py`

**Before:**
```python
if settings.ENVIRONMENT == "production":
    allow_origins=settings.ALLOWED_ORIGINS
else:
    allow_origins=["*"]  # ❌ Conflicts with credentials=True
```

**After:**
```python
# ALWAYS use specific origins (no wildcard with credentials)
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.ALLOWED_ORIGINS,  # ✅ Explicit origins only
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allow_headers=["*"],
    expose_headers=["*"],
)
```

### Fix 2: Added Vercel URL to Allowed Origins

**Updated:** `backend/app/config/settings.py`

```python
ALLOWED_ORIGINS: List[str] = [
    "http://localhost:3000",
    "http://localhost:3001",
    "https://physical-ai-robotics-book.vercel.app",
    "https://physical-ai-and-robotics-book-front.vercel.app",  # ✅ Your actual URL
    "https://shazilk-dev.github.io"
]
```

### Fix 3: Updated Frontend Detection

**Updated:**
- `ChatWidget.tsx`
- `AuthButtons.tsx`

Now matches both `vercel.app` and `physical-ai-robotics-book` in hostname.

---

## 🚀 Deployment Steps

### Step 1: Commit Backend Changes

```bash
git add backend/app/main.py
git add backend/app/config/settings.py
git commit -m "Fix: CORS configuration for credentials mode"
git push origin main
```

**Render will auto-deploy** (~2-3 minutes)

### Step 2: Configure Render Environment Variables

**CRITICAL:** Add these in Render Dashboard

1. Go to: https://dashboard.render.com
2. Select: **physical-ai-and-robotics-book** service
3. Click: **Environment** tab
4. Add/Verify these variables:

| Variable | Value | Required |
|----------|-------|----------|
| `ENVIRONMENT` | `production` | ✅ YES |
| `OPENAI_API_KEY` | `sk-proj-...` | ✅ YES |
| `QDRANT_URL` | `https://...qdrant.io` | ✅ YES |
| `QDRANT_API_KEY` | `...` | ✅ YES |
| `DATABASE_URL` | `postgresql://...` | ✅ YES |
| `JWT_SECRET` | (32+ chars random) | ✅ YES |

5. Click **Save Changes**
6. Service will auto-redeploy

### Step 3: Commit Frontend Changes

```bash
git add frontend/src/components/ChatWidget/ChatWidget.tsx
git add frontend/src/components/Auth/AuthButtons.tsx
git commit -m "Fix: Update production domain detection for Vercel"
git push origin main
```

**Vercel will auto-deploy** (~2 minutes)

---

## ✅ Verification Steps

### Test 1: Backend Health

```bash
curl https://physical-ai-and-robotics-book.onrender.com/health
```

**Expected:**
```json
{
  "status": "healthy",
  "database": "connected",
  "vector_store": "not_configured"
}
```

### Test 2: CORS Headers

```bash
curl -X OPTIONS https://physical-ai-and-robotics-book.onrender.com/api/v1/query \
  -H "Origin: https://physical-ai-and-robotics-book-front.vercel.app" \
  -H "Access-Control-Request-Method: POST" \
  -v
```

**Expected Response Headers:**
```
Access-Control-Allow-Origin: https://physical-ai-and-robotics-book-front.vercel.app
Access-Control-Allow-Credentials: true
Access-Control-Allow-Methods: GET, POST, PUT, DELETE, OPTIONS
```

**NOT:** `Access-Control-Allow-Origin: *`

### Test 3: Frontend Integration

1. Visit: https://physical-ai-and-robotics-book-front.vercel.app
2. Open Browser Console (F12)
3. Check for errors:
   - ✅ No CORS errors
   - ✅ No "Failed to fetch" errors
4. Click chat widget
5. Ask: "What is ROS 2?"
6. Should receive AI response

---

## 🐛 Troubleshooting

### Issue: Still Getting CORS Error

**Check:**
```bash
# Verify backend has your URL in allowed origins
curl https://physical-ai-and-robotics-book.onrender.com/health
```

**Fix:**
1. Check Render logs for startup errors
2. Verify `ALLOWED_ORIGINS` includes your exact Vercel URL
3. Ensure no typos in URL

### Issue: Backend Still Returns 500

**Check Render Logs:**
```
Dashboard → Service → Logs
```

**Common causes:**
- ❌ Missing `OPENAI_API_KEY`
- ❌ Missing `QDRANT_URL` or `QDRANT_API_KEY`
- ❌ Invalid Qdrant collection name
- ❌ Qdrant not seeded with data

**Fix:**
1. Verify all environment variables are set
2. Check logs for specific error message
3. Test Qdrant connection:
   ```bash
   curl -H "api-key: YOUR_KEY" https://your-cluster.qdrant.io/collections
   ```

### Issue: "Collection not found"

**Solution:**
```bash
# Seed Qdrant with textbook content
cd backend
python scripts/seed_vector_db.py
```

---

## 📋 Environment Variables Checklist

Before going live, verify in Render Dashboard:

- [ ] `ENVIRONMENT` = `production`
- [ ] `OPENAI_API_KEY` set (starts with `sk-proj-` or `sk-`)
- [ ] `QDRANT_URL` set (HTTPS URL)
- [ ] `QDRANT_API_KEY` set
- [ ] `DATABASE_URL` set (PostgreSQL connection string)
- [ ] `JWT_SECRET` set (32+ random characters)
- [ ] Service redeployed after adding variables

---

## 🔐 Security Notes

### Why Wildcard CORS is Blocked with Credentials

**Browser Security Rule:**
```
If request has credentials (cookies, auth headers):
  → CORS origin MUST be specific
  → Cannot use "*" wildcard
  → Must match exact origin
```

**Our Fix:**
```python
# ❌ WRONG - Security violation
allow_origins=["*"]
allow_credentials=True  # Conflicts!

# ✅ CORRECT - Specific origins
allow_origins=[
  "https://physical-ai-and-robotics-book-front.vercel.app",
  "http://localhost:3000"
]
allow_credentials=True  # Now allowed!
```

---

## 📊 Expected Results

### Before Fix:
- ❌ CORS error in console
- ❌ Requests blocked by browser
- ❌ Chat widget doesn't work
- ❌ 500 errors from backend

### After Fix:
- ✅ No CORS errors
- ✅ Requests succeed
- ✅ Chat widget works
- ✅ AI responses appear
- ✅ Citations are clickable
- ✅ Auth buttons work (if implemented)

---

## 🎯 Final Checklist

After completing all steps:

- [ ] Backend deployed to Render
- [ ] All environment variables set in Render
- [ ] Backend health check returns 200 OK
- [ ] CORS headers include your Vercel URL (not *)
- [ ] Frontend deployed to Vercel
- [ ] No CORS errors in browser console
- [ ] Chat widget opens without errors
- [ ] Can send messages and receive responses
- [ ] Network tab shows successful API calls

---

**Once all steps are complete, your full-stack application will work perfectly!** 🎉
