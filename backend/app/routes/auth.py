"""
Authentication Routes
Better Auth compatible REST API endpoints
"""

from fastapi import APIRouter, HTTPException, Header, Response, Request
from typing import Optional
from app.models.user import (
    SignUpRequest, SignInRequest, UpdateProfileRequest,
    UserResponse, SessionResponse, AuthResponse
)
from app.services.auth import AuthService
from app.config.database import get_db_pool

router = APIRouter(prefix="/api/auth", tags=["auth"])

# Database pool will be injected
db_pool = None

def get_auth_service():
    """Get auth service instance"""
    global db_pool
    if db_pool is None:
        db_pool = get_db_pool()
    return AuthService(db_pool)

@router.post("/sign-up/email")
async def sign_up(data: SignUpRequest, response: Response):
    """Sign up with email and password"""
    auth_service = get_auth_service()

    # Validate input
    if not data.email or not data.password or not data.name:
        raise HTTPException(status_code=400, detail="Email, password, and name are required")

    if len(data.password) < 8:
        raise HTTPException(status_code=400, detail="Password must be at least 8 characters")

    # Create user
    user = await auth_service.create_user(data.email, data.password, data.name)
    if not user:
        raise HTTPException(status_code=400, detail="An account with this email already exists")

    # Create token
    token = auth_service.create_access_token(user['id'], user['email'])

    # Create session
    await auth_service.create_session(user['id'], token)

    # Set cookie
    response.set_cookie(
        key="auth-token",
        value=token,
        httponly=True,
        max_age=7 * 24 * 60 * 60,  # 7 days
        samesite="lax"
    )

    # Convert snake_case to camelCase for response
    user_response = {
        "id": user['id'],
        "email": user['email'],
        "name": user['name'],
        "emailVerified": user['email_verified'],
        "createdAt": user['created_at'],
        "updatedAt": user['updated_at'],
        "softwareBackground": user.get('software_background'),
        "hardwareBackground": user.get('hardware_background'),
        "pythonExperience": user.get('python_experience'),
        "ros2Experience": user.get('ros2_experience'),
        "roboticsExperience": user.get('robotics_experience'),
    }

    return {
        "token": token,
        "user": user_response
    }

@router.post("/sign-in/email")
async def sign_in(data: SignInRequest, response: Response):
    """Sign in with email and password"""
    auth_service = get_auth_service()

    # Validate input
    if not data.email or not data.password:
        raise HTTPException(status_code=400, detail="Email and password are required")

    # Verify credentials
    user = await auth_service.verify_user(data.email, data.password)
    if not user:
        raise HTTPException(status_code=401, detail="Invalid email or password. Please check your credentials and try again.")

    # Create token
    token = auth_service.create_access_token(user['id'], user['email'])

    # Create session
    await auth_service.create_session(user['id'], token)

    # Set cookie
    response.set_cookie(
        key="auth-token",
        value=token,
        httponly=True,
        max_age=7 * 24 * 60 * 60,  # 7 days
        samesite="lax"
    )

    # Convert snake_case to camelCase
    user_response = {
        "id": user['id'],
        "email": user['email'],
        "name": user['name'],
        "emailVerified": user['email_verified'],
        "createdAt": user['created_at'],
        "updatedAt": user['updated_at'],
        "softwareBackground": user.get('software_background'),
        "hardwareBackground": user.get('hardware_background'),
        "pythonExperience": user.get('python_experience'),
        "ros2Experience": user.get('ros2_experience'),
        "roboticsExperience": user.get('robotics_experience'),
    }

    return {
        "token": token,
        "user": user_response
    }

@router.get("/session")
async def get_session(request: Request):
    """Get current session"""
    auth_service = get_auth_service()
    
    # Get token from cookie or header
    token = request.cookies.get("auth-token")
    if not token:
        auth_header = request.headers.get("Authorization")
        if auth_header and auth_header.startswith("Bearer "):
            token = auth_header[7:]
    
    if not token:
        return {"session": None, "user": None}
    
    # Verify token
    payload = auth_service.verify_token(token)
    if not payload:
        return {"session": None, "user": None}
    
    # Get user
    user = await auth_service.get_user_by_id(payload['sub'])
    if not user:
        return {"session": None, "user": None}
    
    # Convert snake_case to camelCase
    user_response = {
        "id": user['id'],
        "email": user['email'],
        "name": user['name'],
        "emailVerified": user['email_verified'],
        "createdAt": user['created_at'],
        "updatedAt": user['updated_at'],
        "softwareBackground": user.get('software_background'),
        "hardwareBackground": user.get('hardware_background'),
        "pythonExperience": user.get('python_experience'),
        "ros2Experience": user.get('ros2_experience'),
        "roboticsExperience": user.get('robotics_experience'),
    }
    
    return {
        "session": {
            "token": token,
            "expiresAt": payload.get('exp')
        },
        "user": user_response
    }

@router.post("/sign-out")
async def sign_out(response: Response):
    """Sign out user"""
    response.delete_cookie("auth-token")
    return {"success": True}

@router.post("/user/update-profile")
async def update_profile(data: UpdateProfileRequest, request: Request):
    """Update user profile"""
    auth_service = get_auth_service()
    
    # Get token
    token = request.cookies.get("auth-token")
    if not token:
        auth_header = request.headers.get("Authorization")
        if auth_header and auth_header.startswith("Bearer "):
            token = auth_header[7:]
    
    if not token:
        raise HTTPException(status_code=401, detail="Not authenticated")
    
    # Verify token
    payload = auth_service.verify_token(token)
    if not payload:
        raise HTTPException(status_code=401, detail="Invalid token")
    
    # Update profile
    profile_dict = data.dict(exclude_none=True)
    user = await auth_service.update_user_profile(payload['sub'], profile_dict)
    
    if not user:
        raise HTTPException(status_code=500, detail="Failed to update profile")
    
    # Convert snake_case to camelCase
    user_response = {
        "id": user['id'],
        "email": user['email'],
        "name": user['name'],
        "emailVerified": user['email_verified'],
        "createdAt": user['created_at'],
        "updatedAt": user['updated_at'],
        "softwareBackground": user.get('software_background'),
        "hardwareBackground": user.get('hardware_background'),
        "pythonExperience": user.get('python_experience'),
        "ros2Experience": user.get('ros2_experience'),
        "roboticsExperience": user.get('robotics_experience'),
    }
    
    return {"user": user_response}
