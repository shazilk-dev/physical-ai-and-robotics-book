"""
Database Connection Pool
AsyncPG connection management
"""

import asyncpg
from app.config.settings import settings

_pool = None

async def init_db_pool():
    """Initialize database connection pool"""
    global _pool
    if _pool is None and settings.DATABASE_URL:
        try:
            _pool = await asyncpg.create_pool(
                settings.DATABASE_URL,
                min_size=2,
                max_size=10,
                command_timeout=60
            )
            print("[OK] Database pool connected")
        except Exception as e:
            print(f"[WARNING] Database connection failed: {e}")
            print("   Auth endpoints will not work without database")
    return _pool

def get_db_pool() -> asyncpg.Pool:
    """Get database connection pool"""
    global _pool
    if _pool is None:
        raise RuntimeError("Database pool not initialized. Call init_db_pool() first.")
    return _pool

async def close_db_pool():
    """Close database connection pool"""
    global _pool
    if _pool is not None:
        await _pool.close()
        _pool = None
