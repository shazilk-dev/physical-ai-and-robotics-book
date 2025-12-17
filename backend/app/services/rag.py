"""
RAG Service for Physical AI Book
Handles vector search and response generation with personalized response styling
Supports multiple LLM providers: OpenAI, Google Gemini, Alibaba Qwen
"""
import os
from pathlib import Path
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, Filter, FieldCondition, MatchValue
from dotenv import load_dotenv
from .llm_providers import get_llm_provider, BaseLLMProvider

# Load environment variables
load_dotenv(Path(__file__).parent.parent.parent / ".env")


class ResponseStyler:
    """
    Generates customized system prompts based on user settings.
    Adapts AI responses to match user's preferred learning style.
    """

    def __init__(self, settings: Optional[Dict[str, Any]] = None):
        """
        Initialize response styler with user settings

        Args:
            settings: Dict with keys:
                - responseMode: 'quick' | 'detailed' | 'tutorial' | 'socratic'
                - explanationDepth: 1-5 (beginner to expert)
                - includeCodeExamples: bool
                - includeVisuals: bool
                - languageStyle: 'casual' | 'formal' | 'technical'
        """
        self.settings = settings or {}
        self.mode = self.settings.get('responseMode', 'detailed')
        self.depth = self.settings.get('explanationDepth', 3)
        self.include_code = self.settings.get('includeCodeExamples', True)
        self.include_visuals = self.settings.get('includeVisuals', True)
        self.style = self.settings.get('languageStyle', 'casual')

    def get_system_prompt(self, action: Optional[str] = None) -> str:
        """
        Generate complete system prompt based on settings and action

        Args:
            action: Optional action type ('explain', 'simplify', 'example', 'quiz')

        Returns:
            Complete system prompt string
        """
        base_prompt = """You are an expert robotics tutor helping students
learn Physical AI and ROS 2."""

        # Mode-specific instructions
        mode_instructions = {
            'quick': """
                Keep responses CONCISE (2-3 short paragraphs max).
                Use bullet points for lists.
                Get straight to the point.
                Provide quick, actionable answers.
                Focus on the most essential information only.
            """,

            'detailed': """
                Provide COMPREHENSIVE explanations.
                Include background context and reasoning.
                Use examples and analogies to clarify.
                Cover edge cases and common pitfalls.
                Explain the "why" behind concepts, not just the "what".
            """,

            'tutorial': """
                Use STEP-BY-STEP format:
                1. Number each step clearly
                2. Explain WHY each step matters
                3. Show expected outcomes
                4. Provide "check your understanding" questions
                5. Build progressively from simple to complex
            """,

            'socratic': """
                Use SOCRATIC METHOD:
                - Ask guiding questions instead of giving direct answers
                - Help the student discover the answer themselves
                - Build on their existing knowledge
                - Provide hints and prompts, not solutions
                - Encourage critical thinking through questions
            """
        }

        # Depth-specific instructions
        depth_instructions = {
            1: "Assume NO prior knowledge. Use everyday language and simple analogies. Avoid all technical jargon.",
            2: "Assume BASIC programming knowledge. Explain technical terms when used. Use simple examples.",
            3: "Assume INTERMEDIATE understanding. Balance theory and practice. Use standard technical terminology.",
            4: "Assume ADVANCED knowledge. Use technical language freely. Focus on nuanced details and implications.",
            5: "EXPERT level. Dive into implementation details, edge cases, and advanced concepts. Assume deep technical knowledge."
        }

        # Language style instructions
        style_instructions = {
            'casual': "Use friendly, conversational tone. Use 'you' and contractions. Occasional emojis are fine.",
            'formal': "Use professional, academic tone. Avoid slang or casual language. Be precise and structured.",
            'technical': "Focus on precise technical terminology. Cite documentation when relevant. Use industry standard terms."
        }

        # Code examples instruction
        code_instruction = ""
        if self.include_code:
            code_instruction = "Include code examples when relevant. Show working, complete snippets with comments."
        else:
            code_instruction = "Minimize code examples. Only include when absolutely necessary."

        # Visuals instruction
        visuals_instruction = ""
        if self.include_visuals:
            visuals_instruction = "When helpful, suggest or describe visual diagrams (flowcharts, architecture diagrams, etc.)."

        # Action-specific overlay (if provided)
        action_overlay = ""
        if action:
            action_overlays = {
                "explain": "\n\nCurrent Request: Provide a CLEAR EXPLANATION of the selected concept.",
                "simplify": "\n\nCurrent Request: SIMPLIFY the concept using everyday language and analogies.",
                "example": "\n\nCurrent Request: Provide a PRACTICAL CODE EXAMPLE demonstrating the concept.",
                "quiz": "\n\nCurrent Request: Create QUIZ QUESTIONS to test understanding (don't provide answers yet)."
            }
            action_overlay = action_overlays.get(action, "")

        # Assemble full prompt
        full_prompt = f"""
{base_prompt}

**Response Style:** {mode_instructions[self.mode]}

**Explanation Depth:** {depth_instructions[self.depth]}

**Language Style:** {style_instructions[self.style]}

**Code Examples:** {code_instruction}

**Visual Aids:** {visuals_instruction}
{action_overlay}

Remember: Adapt your response to match these settings while staying accurate and educational.
"""

        return full_prompt.strip()

    def adjust_max_tokens(self) -> int:
        """Get appropriate max_tokens based on response mode"""
        token_limits = {
            'quick': 500,
            'detailed': 2000,
            'tutorial': 2500,
            'socratic': 1000
        }
        return token_limits.get(self.mode, 1500)

class RAGService:
    def __init__(self):
        """Initialize RAG service with LLM provider and Qdrant client"""
        # Validate Qdrant environment variables
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_key = os.getenv("QDRANT_API_KEY")

        if not qdrant_url:
            raise ValueError("QDRANT_URL environment variable is not set")
        if not qdrant_key:
            raise ValueError("QDRANT_API_KEY environment variable is not set")

        print(f"🔧 Initializing RAG Service...")
        print(f"   Qdrant URL: {qdrant_url[:30]}...")
        print(f"   Qdrant Key: {'✅ Set' if qdrant_key else '❌ Missing'}")

        # Initialize LLM provider (handles its own API key validation)
        self.llm_provider = get_llm_provider()

        # Initialize Qdrant client
        self.qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_key)

        # Get embedding dimension from provider
        self.embedding_dimension = self.llm_provider.get_embedding_dimension()

        # Generate provider-specific collection name
        provider_suffix = os.getenv("LLM_PROVIDER", "openai").lower()
        base_collection = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_book")
        self.collection_name = f"{base_collection}_{provider_suffix}"

        print(f"   Collection: {self.collection_name}")
        print(f"   Embedding Dimension: {self.embedding_dimension}")

        # Ensure collection exists with correct dimension
        self._ensure_collection_exists()

        print(f"✅ RAG Service initialized with {self.llm_provider.get_provider_name()}")

    def _ensure_collection_exists(self):
        """
        Ensure collection exists with correct dimension for current provider.
        Creates collection if it doesn't exist.
        Validates dimension if it does exist.
        """
        try:
            # Try to get existing collection
            collection_info = self.qdrant_client.get_collection(self.collection_name)

            # Validate dimension matches current provider
            existing_dimension = collection_info.config.params.vectors.size

            if existing_dimension != self.embedding_dimension:
                print(f"⚠️  Warning: Collection '{self.collection_name}' exists with dimension {existing_dimension}")
                print(f"   Current provider needs dimension {self.embedding_dimension}")
                print(f"   You may need to re-seed this collection or use a different provider")
            else:
                print(f"✅ Collection '{self.collection_name}' exists with correct dimension {self.embedding_dimension}")

        except Exception:
            # Collection doesn't exist, create it
            print(f"📦 Creating new collection '{self.collection_name}'...")
            self.qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=self.embedding_dimension,
                    distance=Distance.COSINE
                )
            )
            print(f"✅ Created collection with dimension {self.embedding_dimension}")
            print(f"⚠️  Collection is empty - run 'python scripts/seed_vector_db.py' to index content")

    def create_collection(self):
        """
        Create Qdrant collection if it doesn't exist.
        (Deprecated: Use _ensure_collection_exists instead, called automatically in __init__)
        """
        self._ensure_collection_exists()
    
    def generate_embedding(self, text: str) -> List[float]:
        """Generate embedding for text using configured LLM provider"""
        return self.llm_provider.generate_embedding(text)
    
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
        
        # Generate response using LLM provider
        answer = self.llm_provider.generate_chat_completion(
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            max_tokens=1000,
            temperature=0.7
        )
        
        # Extract citations (sections referenced)
        citations = list(set([
            chunk['section'] for chunk in context_chunks
        ]))
        
        return {
            "answer": answer,
            "sources": context_chunks,
            "citations": citations,
            "model": self.llm_provider.get_provider_name()
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

    def contextual_query(
        self,
        question: str,
        selected_text: str,
        action: str,
        module: str = None,
        num_results: int = 5,
        settings: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """
        Enhanced query with text selection context and personalized settings

        Args:
            question: User's question (may be auto-generated)
            selected_text: The text user selected from the document
            action: The action type (explain, simplify, example, quiz)
            module: Optional module filter
            num_results: Number of results to retrieve
            settings: User's response style preferences

        Returns:
            Query response dict with answer, sources, and citations
        """
        try:
            print(f"🎯 Contextual Query:")
            print(f"   Selected: {selected_text[:50]}...")
            print(f"   Action: {action}")
            if settings:
                print(f"   Mode: {settings.get('responseMode', 'detailed')}")
                print(f"   Depth: {settings.get('explanationDepth', 3)}")

            # Step 1: Search for relevant chunks (use selected text for better search)
            relevant_chunks = self.search_similar_chunks(
                query=selected_text,  # Use selected text for semantic search
                limit=num_results,
                module=module
            )

            if not relevant_chunks:
                return {
                    "answer": f"I couldn't find information about '{selected_text}' in the textbook. Could you try selecting a different concept?",
                    "sources": [],
                    "citations": [],
                    "action": action
                }

            # Step 2: Generate action-specific response with personalized settings
            response = self.generate_contextual_response(
                question=question,
                selected_text=selected_text,
                action=action,
                context_chunks=relevant_chunks,
                settings=settings
            )

            response["action"] = action
            return response

        except Exception as e:
            print(f"❌ Error in contextual_query: {e}")
            raise

    def generate_contextual_response(
        self,
        question: str,
        selected_text: str,
        action: str,
        context_chunks: List[Dict[str, Any]],
        settings: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """
        Generate response adapted to the selected action and user settings

        Args:
            question: User's question
            selected_text: The text user selected
            action: The action type
            context_chunks: Retrieved context from vector DB
            settings: User's response style preferences

        Returns:
            Query response with personalized answer
        """

        # Build context from retrieved chunks
        context = "\n\n".join([
            f"[Source: {chunk['section']}]\n{chunk['content']}"
            for chunk in context_chunks
        ])

        # Create ResponseStyler with user settings
        styler = ResponseStyler(settings)

        # Get personalized system prompt (includes action-specific instructions)
        system_prompt = styler.get_system_prompt(action=action)

        # Get appropriate max_tokens based on response mode
        max_tokens = styler.adjust_max_tokens()

        # Build user prompt with all context
        user_prompt = f"""The student is reading about Physical AI and Robotics.

They selected this text from the document:
"{selected_text}"

Relevant content from the textbook:
{context}

Student's question: {question}

Respond according to your instructions and the student's preferences."""

        # Generate response using LLM provider
        answer = self.llm_provider.generate_chat_completion(
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            max_tokens=max_tokens,  # Adapt based on response mode
            temperature=0.7
        )

        # Extract citations
        citations = list(set([
            chunk['section'] for chunk in context_chunks
        ]))

        return {
            "answer": answer,
            "sources": context_chunks,
            "citations": citations,
            "model": self.llm_provider.get_provider_name(),
            "settings_used": settings  # Return what settings were applied
        }


# Singleton instance
_rag_service = None

def get_rag_service() -> RAGService:
    """Get or create RAG service singleton"""
    global _rag_service
    if _rag_service is None:
        _rag_service = RAGService()
    return _rag_service
