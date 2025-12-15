# Deployment Verification Guide

## ✅ Step-by-Step Deployment and Testing

This guide will help you verify that your frontend and backend are properly connected and working in production.

---

## Phase 1: Push Backend Changes

### Step 1.1: Push to GitHub

```bash
git push origin main
```

**Expected output:**
```
Enumerating objects: X, done.
To https://github.com/your-repo/physical-ai-robotics-book.git
   xxxxxxx..yyyyyyy  main -> main
```

### Step 1.2: Monitor Render Deployment

1. Go to: https://dashboard.render.com
2. Click on your service: **physical-ai-and-robotics-book**
3. Watch the "Events" tab for deployment progress

**What to look for:**
- ✅ "Deploy live" status (usually takes 2-3 minutes)
- ✅ In logs, you should see:
```
==================================================
🌐 CORS Configuration:
   Allowed Origins: ['http://localhost:3000', 'http://localhost:3001', 'http://127.0.0.1:3000', 'http://127.0.0.1:3001', 'https://physical-ai-robotics-book.vercel.app', 'https://physical-ai-and-robotics-book-front.vercel.app', 'https://physical-ai-and-robotics-book.vercel.app', 'https://shazilk-dev.github.io']
   Environment: production
==================================================
```

---

## Phase 2: Verify Backend Configuration

### Step 2.1: Check Health Endpoint

```bash
curl https://physical-ai-and-robotics-book.onrender.com/health
```

**Expected response:**
```json
{
  "status": "healthy",
  "database": "connected",
  "vector_store": "not_configured"
}
```

### Step 2.2: Test CORS Configuration

```bash
curl -i https://physical-ai-and-robotics-book.onrender.com/cors-test
```

**Expected response headers:**
```
HTTP/2 200
content-type: application/json
...
```

**Expected response body:**
```json
{
  "message": "CORS is working!",
  "allowed_origins": [
    "http://localhost:3000",
    "http://localhost:3001",
    "http://127.0.0.1:3000",
    "http://127.0.0.1:3001",
    "https://physical-ai-robotics-book.vercel.app",
    "https://physical-ai-and-robotics-book-front.vercel.app",
    "https://physical-ai-and-robotics-book.vercel.app",
    "https://shazilk-dev.github.io"
  ],
  "environment": "production"
}
```

### Step 2.3: Test CORS Preflight (Most Important!)

This simulates what the browser does before making the actual request:

```bash
curl -X OPTIONS https://physical-ai-and-robotics-book.onrender.com/api/v1/query \
  -H "Origin: https://physical-ai-and-robotics-book-front.vercel.app" \
  -H "Access-Control-Request-Method: POST" \
  -H "Access-Control-Request-Headers: Content-Type" \
  -v 2>&1 | grep -i "access-control"
```

**Expected output (CRITICAL):**
```
< access-control-allow-origin: https://physical-ai-and-robotics-book-front.vercel.app
< access-control-allow-credentials: true
< access-control-allow-methods: GET, POST, PUT, DELETE, OPTIONS
< access-control-allow-headers: *
```

**❌ If you see this, it's broken:**
```
< access-control-allow-origin: *
```
or
```
(no access-control headers)
```

---

## Phase 3: Configure Render Environment Variables

**CRITICAL:** If the CORS test in Step 2.3 failed or you see backend 500 errors, you need to set environment variables in Render.

### Step 3.1: Go to Render Dashboard

1. Navigate to: https://dashboard.render.com
2. Click your service: **physical-ai-and-robotics-book**
3. Click the **Environment** tab on the left

### Step 3.2: Add/Verify These Variables

| Variable Name | Value | Required |
|---------------|-------|----------|
| `ENVIRONMENT` | `production` | ✅ YES |
| `OPENAI_API_KEY` | `sk-proj-...` (your key) | ✅ YES |
| `QDRANT_URL` | `https://xxxxx.qdrant.io` | ✅ YES |
| `QDRANT_API_KEY` | `xxxxx` (your key) | ✅ YES |
| `DATABASE_URL` | `postgresql://...` (Neon) | ✅ YES |
| `JWT_SECRET` | (32+ random characters) | ✅ YES |

**How to get these values:**
- **OPENAI_API_KEY**: https://platform.openai.com/api-keys
- **QDRANT_URL**: https://cloud.qdrant.io/
- **DATABASE_URL**: Your Neon PostgreSQL connection string
- **JWT_SECRET**: Generate with: `openssl rand -hex 32`

### Step 3.3: Save and Redeploy

1. Click **Save Changes** at the bottom
2. Render will automatically redeploy (takes ~2-3 minutes)
3. Wait for "Deploy live" status
4. **Re-run Step 2.3** to verify CORS headers are now correct

---

## Phase 4: Verify Frontend Integration

### Step 4.1: Check Vercel Deployment Status

1. Go to: https://vercel.com/dashboard
2. Ensure latest deployment is "Ready"
3. Note: Frontend was already deployed with the correct domain detection code

### Step 4.2: Test in Browser Console

1. Visit: https://physical-ai-and-robotics-book-front.vercel.app
2. Open Browser DevTools (F12)
3. Go to **Console** tab
4. Look for errors

**✅ GOOD - No errors:**
```
(no CORS errors)
(no "Failed to fetch" errors)
```

**❌ BAD - Still broken:**
```
Access to fetch at 'https://physical-ai-and-robotics-book.onrender.com/api/auth/session'
from origin 'https://physical-ai-and-robotics-book-front.vercel.app'
has been blocked by CORS policy
```

If you see the error, go back to **Phase 3** and verify environment variables are set.

### Step 4.3: Test Network Requests

1. Still in DevTools, click **Network** tab
2. Filter by "Fetch/XHR"
3. Look for requests to `physical-ai-and-robotics-book.onrender.com`
4. Click on a request
5. Check **Headers** tab

**✅ GOOD - Response Headers should include:**
```
access-control-allow-credentials: true
access-control-allow-origin: https://physical-ai-and-robotics-book-front.vercel.app
```

**❌ BAD - If you see:**
```
access-control-allow-origin: *
```
Backend is using old code. Wait for Render to finish deploying.

---

## Phase 5: End-to-End Testing

### Test 1: Authentication Check

1. On your website, look at the top-right corner
2. You should see "Sign In" button (no errors in console)
3. **Success criteria**: Button loads without errors

### Test 2: Chat Widget

1. Click the floating chat button (bottom-right)
2. Type: "What is ROS 2?"
3. Click Send

**✅ GOOD:**
- Loading indicator appears
- AI response appears within 2-4 seconds
- Citations are clickable (e.g., "1.1.1", "1.1.2")
- No errors in console

**❌ BAD:**
- Red error message appears
- Console shows CORS error
- Console shows 500 error

### Test 3: Citation Links

1. After getting a response with citations
2. Click on a citation button (e.g., "1.1.1")
3. Should navigate to the corresponding documentation page

---

## 🐛 Troubleshooting

### Issue: Still Getting CORS Errors

**Symptom:**
```
Access-Control-Allow-Origin: * with credentials mode is 'include'
```

**Solution:**
1. Check Render logs for the CORS debug output (Step 1.2)
2. Verify `ENVIRONMENT` is set to `production` in Render (Step 3.2)
3. Wait 5 minutes for Render to fully deploy
4. Clear browser cache (Ctrl+Shift+Delete)
5. Hard refresh (Ctrl+Shift+R)

### Issue: No Access-Control-Allow-Origin Header

**Symptom:**
```
No 'Access-Control-Allow-Origin' header is present on the requested resource
```

**Possible causes:**
1. ❌ Backend crashed (check Render logs)
2. ❌ Route doesn't exist (verify /health works)
3. ❌ Old code still running (wait for deployment)
4. ❌ CORS middleware not loaded (check logs)

**Solution:**
```bash
# Check if backend is running
curl https://physical-ai-and-robotics-book.onrender.com/health

# Check CORS config endpoint
curl https://physical-ai-and-robotics-book.onrender.com/cors-test

# If both fail, check Render logs for errors
```

### Issue: Backend Returns 500 Error

**Symptom:**
```
POST /api/v1/query → 500 Internal Server Error
```

**Possible causes:**
1. ❌ Missing `OPENAI_API_KEY`
2. ❌ Missing `QDRANT_URL` or `QDRANT_API_KEY`
3. ❌ Qdrant collection not seeded
4. ❌ Invalid API keys

**Solution:**
1. Go to Render Dashboard → Environment
2. Verify all variables from Step 3.2 are set
3. Check Render logs for specific error messages:
```
Dashboard → Service → Logs tab
```
4. Look for errors like:
   - "Invalid API key"
   - "Collection not found"
   - "Connection refused"

### Issue: Chat Widget Shows "Failed to Fetch"

**Symptom:**
```
Sorry, I couldn't process your question. Failed to fetch.
```

**Possible causes:**
1. ❌ Backend is down
2. ❌ CORS blocking requests
3. ❌ Network issue

**Solution:**
1. Test backend health: `curl https://physical-ai-and-robotics-book.onrender.com/health`
2. Check browser console for specific error
3. Verify Render service is "Running" (not "Failed")

---

## ✅ Success Checklist

After completing all steps, verify:

- [ ] Backend deployed successfully on Render
- [ ] `ENVIRONMENT=production` set in Render
- [ ] All 6 environment variables set in Render
- [ ] `/health` endpoint returns 200 OK
- [ ] `/cors-test` shows your Vercel URL in allowed_origins
- [ ] CORS preflight test shows correct headers (not wildcard)
- [ ] Frontend loads without console errors
- [ ] "Sign In" button appears (no auth errors)
- [ ] Chat widget opens
- [ ] Can send message and receive AI response
- [ ] Citations are clickable and navigate correctly
- [ ] No CORS errors in browser console
- [ ] Network tab shows successful API calls with CORS headers

---

## 📊 Expected Results

### Before Fix:
- ❌ CORS error: "wildcard '*' cannot be used with credentials"
- ❌ CORS error: "No Access-Control-Allow-Origin header"
- ❌ Chat widget doesn't work
- ❌ 500 errors from backend

### After Fix:
- ✅ No CORS errors
- ✅ Preflight requests succeed
- ✅ Chat widget works perfectly
- ✅ AI responses appear
- ✅ Citations clickable
- ✅ Full frontend-backend integration

---

## 🎯 Quick Verification Commands

Run these in order:

```bash
# 1. Push changes
git push origin main

# 2. Wait 3 minutes, then test health
curl https://physical-ai-and-robotics-book.onrender.com/health

# 3. Test CORS config
curl https://physical-ai-and-robotics-book.onrender.com/cors-test

# 4. Test CORS preflight (MOST IMPORTANT)
curl -X OPTIONS https://physical-ai-and-robotics-book.onrender.com/api/v1/query \
  -H "Origin: https://physical-ai-and-robotics-book-front.vercel.app" \
  -H "Access-Control-Request-Method: POST" \
  -v 2>&1 | grep -i "access-control-allow-origin"

# Expected: access-control-allow-origin: https://physical-ai-and-robotics-book-front.vercel.app
# NOT: access-control-allow-origin: *
```

If all tests pass, visit your website and test the chat widget!

---

**Last Updated**: 2025-12-15
**Status**: Ready for deployment verification
