# Quick Setup Guide

## 1. Configure Environment

Copy the example and fill in your values:

```powershell
cp .env.example .env
```

**Required variables:**

- `DATABASE_URL`: Your Neon Postgres connection string
- `JWT_SECRET`: Generate with `openssl rand -base64 32`

## 2. Install Dependencies

```powershell
pip install -r requirements.txt
```

## 3. Start Server

```powershell
# Using the startup script
.\start.ps1

# Or manually
uvicorn app.main:app --reload --port 8000
```

## 4. Test Endpoints

Open http://localhost:8000/docs to see the API documentation.

### Available Endpoints:

**Authentication:**

- `POST /api/auth/sign-up/email` - Create account
- `POST /api/auth/sign-in/email` - Sign in
- `GET /api/auth/session` - Get current session
- `POST /api/auth/sign-out` - Sign out
- `POST /api/auth/user/update-profile` - Update user profile

**RAG Chatbot:**

- `POST /api/rag/chat` - Send message to chatbot

## 5. Frontend Configuration

The frontend is already configured to use `http://localhost:8000` when running locally.

Make sure the backend is running before testing authentication.

## Troubleshooting

**Database connection errors:**

- Verify your DATABASE_URL in .env
- Check Neon dashboard for connection string
- Ensure database schema is created (users, accounts, sessions tables)

**CORS errors:**

- Frontend origin should be in ALLOWED_ORIGINS setting
- Default includes localhost:3000 and localhost:3001

**Import errors:**

- Make sure you're in the backend directory
- Activate virtual environment: `.\venv\Scripts\Activate.ps1`
- Reinstall dependencies: `pip install -r requirements.txt`
