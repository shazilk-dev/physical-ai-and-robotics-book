"""
FastAPI Backend Entry Point
Physical AI & Humanoid Robotics Textbook
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.config.settings import settings

app = FastAPI(
    title="Physical AI Textbook RAG API",
    description="RAG-powered chatbot backend for the Physical AI textbook",
    version="0.1.0",
)

# CORS Configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
async def root():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "service": "Physical AI RAG API",
        "version": "0.1.0"
    }

@app.get("/health")
async def health():
    """Detailed health check"""
    return {
        "status": "healthy",
        "database": "not_configured",
        "vector_store": "not_configured"
    }

# Import routes
from app.routes import rag
app.include_router(rag.router)
