"""
FastAPI Routes for RAG Query Endpoint
"""
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional, List, Dict, Any
from ..services.rag import get_rag_service

router = APIRouter(prefix="/api/v1", tags=["rag"])


class QueryRequest(BaseModel):
    """Request model for RAG query"""
    question: str
    module: Optional[str] = None
    num_results: Optional[int] = 5


class SourceChunk(BaseModel):
    """Source chunk model"""
    content: str
    module: str
    chapter: str
    section: str
    file_path: str
    score: float


class QueryResponse(BaseModel):
    """Response model for RAG query"""
    answer: str
    sources: List[Dict[str, Any]]
    citations: List[str]
    model: Optional[str] = None


@router.post("/query", response_model=QueryResponse)
async def query_rag(request: QueryRequest):
    """
    Query the RAG system with a question
    
    **Example Request:**
    ```json
    {
        "question": "What is ROS 2 and why do we need it?",
        "module": "module-01-ros2",
        "num_results": 5
    }
    ```
    
    **Example Response:**
    ```json
    {
        "answer": "ROS 2 (Robot Operating System 2) is a middleware framework...",
        "sources": [...],
        "citations": ["1.1.1 Architecture", "1.1.2 rclpy Patterns"],
        "model": "gpt-4o-mini"
    }
    ```
    """
    try:
        rag_service = get_rag_service()
        
        result = rag_service.query(
            question=request.question,
            module=request.module,
            num_results=request.num_results
        )
        
        return QueryResponse(**result)
    
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error processing query: {str(e)}"
        )


@router.get("/health")
async def health_check():
    """Health check endpoint"""
    try:
        rag_service = get_rag_service()
        # Test Qdrant connection
        collections = rag_service.qdrant_client.get_collections()
        return {
            "status": "healthy",
            "qdrant_connected": True,
            "collections": [c.name for c in collections.collections]
        }
    except Exception as e:
        return {
            "status": "unhealthy",
            "error": str(e)
        }


@router.get("/stats")
async def get_stats():
    """Get statistics about the vector database"""
    try:
        rag_service = get_rag_service()
        collection_info = rag_service.qdrant_client.get_collection(
            rag_service.collection_name
        )
        
        return {
            "collection_name": rag_service.collection_name,
            "points_count": collection_info.points_count,
            "vectors_count": collection_info.vectors_count,
            "indexed": collection_info.status
        }
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error getting stats: {str(e)}"
        )
