"""
RAG Service for Physical AI Book
Handles vector search and response generation
"""
import os
from pathlib import Path
from typing import List, Dict, Any
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, Filter, FieldCondition, MatchValue
from dotenv import load_dotenv

# Load environment variables
load_dotenv(Path(__file__).parent.parent.parent / ".env")

class RAGService:
    def __init__(self):
        """Initialize RAG service with OpenAI and Qdrant clients"""
        # Validate environment variables
        openai_key = os.getenv("OPENAI_API_KEY")
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_key = os.getenv("QDRANT_API_KEY")

        if not openai_key:
            raise ValueError("OPENAI_API_KEY environment variable is not set")
        if not qdrant_url:
            raise ValueError("QDRANT_URL environment variable is not set")
        if not qdrant_key:
            raise ValueError("QDRANT_API_KEY environment variable is not set")

        print(f"🔧 Initializing RAG Service...")
        print(f"   OpenAI Key: {'✅ Set' if openai_key else '❌ Missing'}")
        print(f"   Qdrant URL: {qdrant_url[:30]}..." if qdrant_url else "❌ Missing")
        print(f"   Qdrant Key: {'✅ Set' if qdrant_key else '❌ Missing'}")

        self.openai_client = OpenAI(api_key=openai_key)
        self.qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_key)
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_book")
        self.embedding_model = "text-embedding-3-small"
        self.chat_model = "gpt-4o-mini"

        print(f"   Collection: {self.collection_name}")
        print(f"✅ RAG Service initialized")
        
    def create_collection(self):
        """Create Qdrant collection if it doesn't exist"""
        try:
            self.qdrant_client.get_collection(self.collection_name)
            print(f"Collection '{self.collection_name}' already exists")
        except:
            self.qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=1536,  # text-embedding-3-small dimension
                    distance=Distance.COSINE
                )
            )
            print(f"Created collection '{self.collection_name}'")
    
    def generate_embedding(self, text: str) -> List[float]:
        """Generate embedding for text using OpenAI"""
        response = self.openai_client.embeddings.create(
            model=self.embedding_model,
            input=text
        )
        return response.data[0].embedding
    
    def search_similar_chunks(
        self,
        query: str,
        limit: int = 5,
        module: str = None
    ) -> List[Dict[str, Any]]:
        """Search for similar content chunks"""
        query_vector = self.generate_embedding(query)

        # Optional filter by module
        query_filter = None
        if module:
            query_filter = Filter(
                must=[FieldCondition(key="module", match=MatchValue(value=module))]
            )

        # Use search() method for qdrant-client 1.7.3 compatibility
        search_result = self.qdrant_client.search(
            collection_name=self.collection_name,
            query_vector=query_vector,
            limit=limit,
            query_filter=query_filter
        )

        results = []
        for hit in search_result:
            results.append({
                "content": hit.payload.get("content"),
                "module": hit.payload.get("module"),
                "chapter": hit.payload.get("chapter"),
                "section": hit.payload.get("section"),
                "file_path": hit.payload.get("file_path"),
                "score": hit.score
            })

        return results
    
    def generate_response(
        self, 
        query: str, 
        context_chunks: List[Dict[str, Any]]
    ) -> Dict[str, Any]:
        """Generate answer using GPT-4 with retrieved context"""
        
        # Build context from retrieved chunks
        context = "\n\n".join([
            f"[Source: {chunk['section']}]\n{chunk['content']}"
            for chunk in context_chunks
        ])
        
        # System prompt for Physical AI expert
        system_prompt = """You are an expert instructor for Physical AI and Humanoid Robotics. 
You help students understand ROS 2, URDF, sensors, and robot programming.

Guidelines:
- Answer based on the provided context from the textbook
- Be clear and educational, assuming the student is learning
- Include code examples when relevant
- Cite specific sections when referencing material
- If the answer isn't in the context, say so and provide general guidance
- Use analogies and visual descriptions when helpful
"""
        
        # User prompt with context
        user_prompt = f"""Context from the Physical AI & Humanoid Robotics textbook:

{context}

Student Question: {query}

Please provide a clear, educational answer based on the context above. Include specific section references where appropriate."""
        
        # Generate response
        response = self.openai_client.chat.completions.create(
            model=self.chat_model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.7,
            max_tokens=1000
        )
        
        answer = response.choices[0].message.content
        
        # Extract citations (sections referenced)
        citations = list(set([
            chunk['section'] for chunk in context_chunks
        ]))
        
        return {
            "answer": answer,
            "sources": context_chunks,
            "citations": citations,
            "model": self.chat_model
        }
    
    def query(
        self,
        question: str,
        module: str = None,
        num_results: int = 5
    ) -> Dict[str, Any]:
        """Main query method - search and generate answer"""

        try:
            # Step 1: Search for relevant chunks
            relevant_chunks = self.search_similar_chunks(
                query=question,
                limit=num_results,
                module=module
            )

            if not relevant_chunks:
                return {
                    "answer": "I couldn't find relevant information in the textbook for this question. The vector database might be empty. Please ensure it has been seeded with content.",
                    "sources": [],
                    "citations": []
                }

            # Step 2: Generate answer with context
            response = self.generate_response(question, relevant_chunks)

            return response

        except Exception as e:
            print(f"❌ Error in query method: {e}")
            raise


# Singleton instance
_rag_service = None

def get_rag_service() -> RAGService:
    """Get or create RAG service singleton"""
    global _rag_service
    if _rag_service is None:
        _rag_service = RAGService()
    return _rag_service
