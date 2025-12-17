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

    # Database (Neon Postgres) - Optional for MVP
    DATABASE_URL: str = "sqlite:///./test.db"  # Default fallback

    # Vector Store (Qdrant Cloud) - Required
    QDRANT_URL: str
    QDRANT_API_KEY: str
    QDRANT_COLLECTION: str = "physical_ai_textbook"

    # OpenAI - Required
    OPENAI_API_KEY: str
    OPENAI_EMBEDDING_MODEL: str = "text-embedding-3-small"
    OPENAI_CHAT_MODEL: str = "gpt-4o-mini"

    # Gemini - Optional
    GEMINI_API_KEY: str = ""

    # Qwen - Optional
    QWEN_API_KEY: str = ""

    # LLM Provider - Default to OpenAI
    LLM_PROVIDER: str = "openai"

    # Auth - Optional for MVP
    JWT_SECRET: str = "dev-secret-key-change-in-production"
    JWT_ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 60

    class Config:
        env_file = ".env"
        case_sensitive = True
        # Don't fail if .env file is missing (for Docker/Render)
        env_file_encoding = 'utf-8'

# Use try-except to provide better error messages
try:
    settings = Settings()
except Exception as e:
    import sys
    print(f"❌ Error loading settings: {e}")
    print("\n⚠️  Required environment variables:")
    print("  - QDRANT_URL")
    print("  - QDRANT_API_KEY")
    print("  - OPENAI_API_KEY")
    print("\nOptional environment variables:")
    print("  - DATABASE_URL")
    print("  - GEMINI_API_KEY")
    print("  - QWEN_API_KEY")
    print("  - JWT_SECRET")
    sys.exit(1)
