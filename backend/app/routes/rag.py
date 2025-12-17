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
    provider: Optional[str] = None  # LLM provider: openai, gemini, qwen (overrides env variable)


class ChatSettings(BaseModel):
    """User's response style preferences"""
    responseMode: Optional[str] = 'detailed'  # quick, detailed, tutorial, socratic
    explanationDepth: Optional[int] = 3  # 1-5 scale
    includeCodeExamples: Optional[bool] = True
    includeVisuals: Optional[bool] = True
    languageStyle: Optional[str] = 'casual'  # casual, formal, technical


class ContextualQueryRequest(BaseModel):
    """Request model for contextual RAG query with text selection and settings"""
    question: str
    selected_text: Optional[str] = None  # Text user selected from the document
    action: Optional[str] = None  # explain, simplify, example, quiz
    module: Optional[str] = None
    num_results: Optional[int] = 5
    settings: Optional[ChatSettings] = None  # User's response style preferences
    provider: Optional[str] = None  # LLM provider: openai, gemini, qwen (overrides env variable)

    class Config:
        json_schema_extra = {
            "example": {
                "question": "Explain this concept: 'pub/sub pattern'",
                "selected_text": "pub/sub pattern",
                "action": "explain",
                "num_results": 5,
                "settings": {
                    "responseMode": "detailed",
                    "explanationDepth": 3,
                    "includeCodeExamples": True,
                    "includeVisuals": True,
                    "languageStyle": "casual"
                }
            }
        }

    # Validation
    @classmethod
    def validate_action(cls, v):
        if v and v not in ['explain', 'simplify', 'example', 'quiz']:
            raise ValueError(f"Invalid action: {v}. Must be one of: explain, simplify, example, quiz")
        return v

    @classmethod
    def validate_selected_text(cls, v):
        if v and (len(v) < 5 or len(v) > 1000):
            raise ValueError("Selected text must be between 5 and 1000 characters")
        return v


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

        # Check if provider override is specified
        if request.provider:
            print(f"🔄 Using provider override: {request.provider}")
            result = rag_service.query_with_provider(
                question=request.question,
                provider_name=request.provider,
                module=request.module,
                num_results=request.num_results
            )
        else:
            # Use default provider from env variable
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
    import time
    start_time = time.time()

    try:
        # Validate action if provided
        if request.action:
            request.validate_action(request.action)

        # Validate selected text if provided
        if request.selected_text:
            request.validate_selected_text(request.selected_text)

        print(f"\n{'='*60}")
        print(f"📥 Contextual Query Request:")
        print(f"   Question: {request.question[:80]}...")
        if request.selected_text:
            print(f"   Selected: {request.selected_text[:50]}...")
        if request.action:
            print(f"   Action: {request.action}")
        if request.settings:
            print(f"   Response Mode: {request.settings.responseMode}")
            print(f"   Depth: {request.settings.explanationDepth}")
        print(f"{'='*60}\n")

        rag_service = get_rag_service()

        # Convert settings to dict if provided
        settings_dict = None
        if request.settings:
            settings_dict = request.settings.model_dump()

        # Check if provider override is specified
        if request.provider:
            print(f"   Using provider override: {request.provider}")
            # Use provider-specific query methods
            if request.selected_text and request.action:
                result = rag_service.contextual_query_with_provider(
                    question=request.question,
                    selected_text=request.selected_text,
                    action=request.action,
                    provider_name=request.provider,
                    module=request.module,
                    num_results=request.num_results,
                    settings=settings_dict
                )
            else:
                result = rag_service.query_with_provider(
                    question=request.question,
                    provider_name=request.provider,
                    module=request.module,
                    num_results=request.num_results
                )
        else:
            # Use default provider from env variable
            if request.selected_text and request.action:
                result = rag_service.contextual_query(
                    question=request.question,
                    selected_text=request.selected_text,
                    action=request.action,
                    module=request.module,
                    num_results=request.num_results,
                    settings=settings_dict
                )
            else:
                result = rag_service.query(
                    question=request.question,
                    module=request.module,
                    num_results=request.num_results
                )

        # Log success
        elapsed_time = (time.time() - start_time) * 1000
        print(f"✅ Query completed in {elapsed_time:.0f}ms")
        print(f"   Citations: {result.get('citations', [])}")
        print(f"   Sources: {len(result.get('sources', []))}")

        return QueryResponse(**result)

    except ValueError as ve:
        # Validation error
        print(f"⚠️  Validation Error: {str(ve)}")
        raise HTTPException(
            status_code=400,
            detail=str(ve)
        )
    except Exception as e:
        import traceback
        error_detail = traceback.format_exc()
        print(f"❌ Contextual RAG Query Error:")
        print(error_detail)

        # Don't expose internal errors to users
        raise HTTPException(
            status_code=500,
            detail="An error occurred while processing your request. Please try again."
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
