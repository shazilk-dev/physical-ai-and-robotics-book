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


class ContextualQueryRequest(BaseModel):
    """Request model for contextual RAG query with text selection"""
    question: str
    selected_text: Optional[str] = None  # Text user selected from the document
    action: Optional[str] = None  # explain, simplify, example, quiz
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
        import traceback
        error_detail = traceback.format_exc()
        print(f"❌ RAG Query Error: {error_detail}")
        raise HTTPException(
            status_code=500,
            detail=f"Error processing query: {str(e)}"
        )


@router.post("/query/contextual", response_model=QueryResponse)
async def query_rag_contextual(request: ContextualQueryRequest):
    """
    Enhanced query endpoint with text selection context

    **Example Request:**
    ```json
    {
        "question": "Explain this concept: 'pub/sub pattern'",
        "selected_text": "pub/sub pattern",
        "action": "explain",
        "num_results": 5
    }
    ```

    **Actions:**
    - **explain**: Clear, comprehensive explanation
    - **simplify**: Simplified explanation with analogies
    - **example**: Practical code examples
    - **quiz**: Generate quiz questions to test understanding

    **Example Response:**
    ```json
    {
        "answer": "The pub/sub pattern in ROS 2 works like...",
        "sources": [...],
        "citations": ["1.1.1", "1.1.2"],
        "model": "gpt-4o-mini"
    }
    ```
    """
    try:
        rag_service = get_rag_service()

        # Use contextual query method if selection context provided
        if request.selected_text and request.action:
            result = rag_service.contextual_query(
                question=request.question,
                selected_text=request.selected_text,
                action=request.action,
                module=request.module,
                num_results=request.num_results
            )
        else:
            # Fall back to regular query
            result = rag_service.query(
                question=request.question,
                module=request.module,
                num_results=request.num_results
            )

        return QueryResponse(**result)

    except Exception as e:
        import traceback
        error_detail = traceback.format_exc()
        print(f"❌ Contextual RAG Query Error: {error_detail}")
        raise HTTPException(
            status_code=500,
            detail=f"Error processing contextual query: {str(e)}"
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
