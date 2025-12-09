"""
Database Setup Script
Creates all required tables for authentication
"""

import asyncio
import asyncpg
from dotenv import load_dotenv
import os

load_dotenv()

async def create_tables():
    """Create all database tables"""
    
    database_url = os.getenv('DATABASE_URL')
    if not database_url:
        print("❌ DATABASE_URL not found in .env file")
        return
    
    print("🔗 Connecting to database...")
    
    try:
        conn = await asyncpg.connect(database_url)
        print("✅ Connected to database")
        
        # Read SQL file
        with open('create_tables.sql', 'r') as f:
            sql = f.read()
        
        # Execute SQL
        print("\n📝 Creating tables...")
        await conn.execute(sql)
        print("✅ All tables created successfully!")
        
        # Verify tables exist
        print("\n🔍 Verifying tables...")
        tables = await conn.fetch("""
            SELECT tablename FROM pg_tables 
            WHERE schemaname = 'public' 
            AND tablename IN ('users', 'accounts', 'sessions', 'verification')
            ORDER BY tablename
        """)
        
        for table in tables:
            print(f"  ✓ {table['tablename']}")
        
        await conn.close()
        print("\n✅ Database setup complete!")
        print("\n🚀 You can now start the backend server:")
        print("   python -m uvicorn app.main:app --port 8000")
        
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    asyncio.run(create_tables())
