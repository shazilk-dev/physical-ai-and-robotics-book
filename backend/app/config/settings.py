"""
Application Settings
Environment Configuration
"""

from pydantic_settings import BaseSettings
from typing import List

class Settings(BaseSettings):
    # Application
    APP_NAME: str = "Physical AI RAG API"
    APP_VERSION: str = "0.1.0"
    DEBUG: bool = True
    ENVIRONMENT: str = "development"  # "development" or "production"

    # CORS - Allow all variations of frontend URLs
    ALLOWED_ORIGINS: List[str] = [
        # Local development
        "http://localhost:3000",
        "http://localhost:3001",
        "http://127.0.0.1:3000",
        "http://127.0.0.1:3001",
        # Vercel production URLs (all variations)
        "https://physical-ai-robotics-book.vercel.app",
        "https://physical-ai-and-robotics-book-front.vercel.app",
        "https://physical-ai-and-robotics-book.vercel.app",
        # GitHub Pages
        "https://shazilk-dev.github.io",
    ]
    
    # Database (Neon Postgres)
    DATABASE_URL: str = ""
    
    # Vector Store (Qdrant Cloud)
    QDRANT_URL: str = ""
    QDRANT_API_KEY: str = ""
    QDRANT_COLLECTION: str = "physical_ai_textbook"
    
    # OpenAI
    OPENAI_API_KEY: str = ""
    OPENAI_EMBEDDING_MODEL: str = "text-embedding-3-small"
    OPENAI_CHAT_MODEL: str = "gpt-4o-mini"
    
    # Auth
    JWT_SECRET: str = ""
    JWT_ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 60
    
    class Config:
        env_file = ".env"
        case_sensitive = True

settings = Settings()
