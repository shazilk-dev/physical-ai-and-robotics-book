"""
User Models
Pydantic models for authentication
"""

from pydantic import BaseModel, EmailStr, Field
from typing import Optional, Literal
from datetime import datetime

# Request Models
class SignUpRequest(BaseModel):
    email: EmailStr
    password: str = Field(..., min_length=8)
    name: str
    
class SignInRequest(BaseModel):
    email: EmailStr
    password: str

class UpdateProfileRequest(BaseModel):
    softwareBackground: Optional[Literal["none", "beginner", "intermediate", "advanced"]] = None
    hardwareBackground: Optional[Literal["none", "beginner", "intermediate", "advanced"]] = None
    pythonExperience: Optional[Literal["none", "beginner", "intermediate", "advanced"]] = None
    ros2Experience: Optional[Literal["none", "beginner", "intermediate", "advanced"]] = None
    roboticsExperience: Optional[Literal["none", "beginner", "intermediate", "advanced"]] = None

# Response Models
class UserResponse(BaseModel):
    id: str
    email: str
    name: str
    emailVerified: bool
    createdAt: datetime
    updatedAt: datetime
    softwareBackground: Optional[str] = None
    hardwareBackground: Optional[str] = None
    pythonExperience: Optional[str] = None
    ros2Experience: Optional[str] = None
    roboticsExperience: Optional[str] = None

class SessionResponse(BaseModel):
    session: dict
    user: UserResponse

class AuthResponse(BaseModel):
    token: str
    user: UserResponse
