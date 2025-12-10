"""
FastAPI Backend Entry Point
Physical AI & Humanoid Robotics Textbook
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
from app.config.settings import settings
from app.config.database import init_db_pool, close_db_pool

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Startup and shutdown events"""
    # Startup
    try:
        await init_db_pool()
    except Exception as e:
        print(f"⚠️  Startup warning: {e}")
    yield
    # Shutdown
    try:
        await close_db_pool()
        print("✅ Database pool closed")
    except:
        pass

app = FastAPI(
    title="Physical AI Textbook API",
    description="RAG-powered chatbot backend with authentication",
    version="0.2.0",
    lifespan=lifespan
)

# CORS Configuration - Allow all origins in development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins for development
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
async def root():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "service": "Physical AI API",
        "version": "0.2.0",
        "features": ["auth", "rag"]
    }

@app.get("/health")
async def health():
    """Detailed health check"""
    return {
        "status": "healthy",
        "database": "connected",
        "vector_store": "not_configured"
    }

# Import routes
from app.routes import auth, rag
app.include_router(auth.router)
app.include_router(rag.router)
