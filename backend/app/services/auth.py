"""
Authentication Service
Handles user registration, login, and session management
"""

import secrets
import hashlib
import hmac
from datetime import datetime, timedelta
from typing import Optional, Dict
import asyncpg
from jose import jwt, JWTError
from app.config.settings import settings

class AuthService:
    def __init__(self, db_pool: asyncpg.Pool):
        self.db = db_pool
    
    def hash_password(self, password: str) -> str:
        """Hash password using PBKDF2"""
        salt = secrets.token_hex(32)
        pwdhash = hashlib.pbkdf2_hmac('sha256', 
                                       password.encode('utf-8'), 
                                       salt.encode('utf-8'), 
                                       100000)
        return f"{salt}${pwdhash.hex()}"
    
    def verify_password(self, stored_password: str, provided_password: str) -> bool:
        """Verify password against hash"""
        salt, pwdhash = stored_password.split('$')
        check_hash = hashlib.pbkdf2_hmac('sha256',
                                          provided_password.encode('utf-8'),
                                          salt.encode('utf-8'),
                                          100000)
        return hmac.compare_digest(pwdhash, check_hash.hex())
    
    def create_access_token(self, user_id: str, email: str) -> str:
        """Create JWT access token"""
        expire = datetime.utcnow() + timedelta(minutes=settings.ACCESS_TOKEN_EXPIRE_MINUTES)
        to_encode = {
            "sub": user_id,
            "email": email,
            "exp": expire
        }
        return jwt.encode(to_encode, settings.JWT_SECRET, algorithm=settings.JWT_ALGORITHM)
    
    def verify_token(self, token: str) -> Optional[Dict]:
        """Verify JWT token"""
        try:
            payload = jwt.decode(token, settings.JWT_SECRET, algorithms=[settings.JWT_ALGORITHM])
            return payload
        except JWTError:
            return None
    
    async def create_user(self, email: str, password: str, name: str) -> Optional[Dict]:
        """Create new user"""
        try:
            # Check if user exists
            existing = await self.db.fetchrow(
                "SELECT id FROM users WHERE email = $1", email
            )
            if existing:
                return None
            
            # Hash password
            password_hash = self.hash_password(password)
            user_id = secrets.token_urlsafe(16)
            
            # Insert user
            user = await self.db.fetchrow("""
                INSERT INTO users (id, email, name, email_verified, created_at, updated_at)
                VALUES ($1, $2, $3, $4, $5, $6)
                RETURNING id, email, name, email_verified, created_at, updated_at,
                          software_background, hardware_background, python_experience,
                          ros2_experience, robotics_experience
            """, user_id, email, name, False, datetime.utcnow(), datetime.utcnow())
            
            # Store password in accounts table
            await self.db.execute("""
                INSERT INTO accounts (id, user_id, account_id, provider_id, password)
                VALUES ($1, $2, $3, $4, $5)
            """, secrets.token_urlsafe(16), user_id, email, "credential", password_hash)
            
            return dict(user)
        except Exception as e:
            print(f"Error creating user: {e}")
            return None
    
    async def verify_user(self, email: str, password: str) -> Optional[Dict]:
        """Verify user credentials"""
        try:
            # Get user and password
            result = await self.db.fetchrow("""
                SELECT u.id, u.email, u.name, u.email_verified, u.created_at, u.updated_at,
                       u.software_background, u.hardware_background, u.python_experience,
                       u.ros2_experience, u.robotics_experience, a.password
                FROM users u
                JOIN accounts a ON u.id = a.user_id
                WHERE u.email = $1 AND a.provider_id = 'credential'
            """, email)
            
            if not result:
                return None
            
            # Verify password
            if not self.verify_password(result['password'], password):
                return None
            
            # Remove password from result
            user_dict = dict(result)
            del user_dict['password']
            
            return user_dict
        except Exception as e:
            print(f"Error verifying user: {e}")
            return None
    
    async def get_user_by_id(self, user_id: str) -> Optional[Dict]:
        """Get user by ID"""
        try:
            user = await self.db.fetchrow("""
                SELECT id, email, name, email_verified, created_at, updated_at,
                       software_background, hardware_background, python_experience,
                       ros2_experience, robotics_experience
                FROM users WHERE id = $1
            """, user_id)
            return dict(user) if user else None
        except Exception as e:
            print(f"Error getting user: {e}")
            return None
    
    async def update_user_profile(self, user_id: str, profile_data: Dict) -> Optional[Dict]:
        """Update user profile"""
        try:
            # Build dynamic update query
            updates = []
            values = []
            param_count = 1
            
            for field in ['software_background', 'hardware_background', 'python_experience',
                          'ros2_experience', 'robotics_experience']:
                camel_field = ''.join([w.capitalize() if i > 0 else w 
                                       for i, w in enumerate(field.split('_'))])
                if camel_field in profile_data and profile_data[camel_field] is not None:
                    updates.append(f"{field} = ${param_count}")
                    values.append(profile_data[camel_field])
                    param_count += 1
            
            if not updates:
                return await self.get_user_by_id(user_id)
            
            values.append(datetime.utcnow())
            updates.append(f"updated_at = ${param_count}")
            param_count += 1
            values.append(user_id)
            
            query = f"""
                UPDATE users
                SET {', '.join(updates)}
                WHERE id = ${param_count}
                RETURNING id, email, name, email_verified, created_at, updated_at,
                          software_background, hardware_background, python_experience,
                          ros2_experience, robotics_experience
            """
            
            user = await self.db.fetchrow(query, *values)
            return dict(user) if user else None
        except Exception as e:
            print(f"Error updating profile: {e}")
            return None
    
    async def create_session(self, user_id: str, token: str) -> str:
        """Create session record"""
        try:
            session_id = secrets.token_urlsafe(32)
            expires_at = datetime.utcnow() + timedelta(days=7)
            
            await self.db.execute("""
                INSERT INTO sessions (id, user_id, expires_at, token, created_at, updated_at)
                VALUES ($1, $2, $3, $4, $5, $6)
            """, session_id, user_id, expires_at, token, datetime.utcnow(), datetime.utcnow())
            
            return session_id
        except Exception as e:
            print(f"Error creating session: {e}")
            return None
    
    async def get_session(self, token: str) -> Optional[Dict]:
        """Get session by token"""
        try:
            session = await self.db.fetchrow("""
                SELECT s.id, s.user_id, s.expires_at, s.token,
                       u.id as user_id, u.email, u.name, u.email_verified,
                       u.created_at, u.updated_at, u.software_background,
                       u.hardware_background, u.python_experience,
                       u.ros2_experience, u.robotics_experience
                FROM sessions s
                JOIN users u ON s.user_id = u.id
                WHERE s.token = $1 AND s.expires_at > $2
            """, token, datetime.utcnow())
            
            return dict(session) if session else None
        except Exception as e:
            print(f"Error getting session: {e}")
            return None
