# Data Model: RAG Integration for Module-Based Content

**Feature Branch**: `001-physical-ai-book`  
**Created**: 2025-12-07  
**Purpose**: Define data structures, chunking strategy, and retrieval logic for the RAG-powered chatbot serving module-based educational content

---

## Overview

The RAG (Retrieval-Augmented Generation) system indexes the 4-module Physical AI textbook content into Qdrant vector database, enabling students to query specific concepts with automatic citation to module/chapter/section sources. This document defines:

1. **Content Chunking Strategy**: How to split 50 pages into retrievable units
2. **Metadata Schema**: Module/chapter/section identifiers for filtering
3. **Embedding Pipeline**: OpenAI text-embedding-3-small configuration
4. **Retrieval Strategy**: Hybrid search (vector similarity + metadata filters)
5. **Database Schema**: Qdrant collection structure with Postgres backing

---

## Content Structure (Source Hierarchy)

```
frontend/docs/
├── module-01-ros2/              # Module 1: ROS 2 Nervous System (12 pages)
│   ├── 00-overview.md           # Module overview page
│   ├── 01-ros2-fundamentals/    # Chapter 1.1 (5 pages)
│   │   ├── 1.1.1-architecture.md          # Section (~800 words)
│   │   ├── 1.1.2-rclpy-patterns.md        # Section (~800 words)
│   │   ├── 1.1.3-dds-qos.md               # Section (~800 words)
│   │   └── 1.1.4-debugging-rqt.md         # Section (~800 words)
│   ├── 02-urdf-robot-description/  # Chapter 1.2 (4 pages)
│   └── 03-sensors-proprioception/  # Chapter 1.3 (3 pages)
├── module-02-architecture/      # Module 2: Core Architecture (14 pages)
├── module-03-isaac-simulation/  # Module 3: Isaac & Simulation (13 pages)
└── module-04-vla-whisper/       # Module 4: VLA & Whisper (11 pages)

labs/
├── lab01-ros2-basics/           # Lab 1: Your First ROS 2 Node
│   └── README.md                # Lab instructions (~1500 words)
└── lab02-urdf-humanoid/         # Lab 2: URDF for Humanoid
    └── README.md
```

---

## Chunking Strategy

### Chunk Types and Sizes

| Content Type         | Source                          | Target Chunk Size | Overlap    | Chunks per Page        |
| -------------------- | ------------------------------- | ----------------- | ---------- | ---------------------- |
| **Theory Sections**  | `*.md` files in module chapters | 500 tokens        | 50 tokens  | ~2-3 chunks            |
| **Code Examples**    | Fenced code blocks in `.md`     | 200-300 tokens    | 0 tokens   | 1 chunk per example    |
| **Lab Instructions** | `labs/*/README.md`              | 600 tokens        | 100 tokens | ~3-4 chunks per lab    |
| **Diagrams/Tables**  | Mermaid code or Markdown tables | Full content      | 0 tokens   | 1 chunk per diagram    |
| **Checkpoints**      | ✅ sections in `.md`            | 150 tokens        | 0 tokens   | 1 chunk per checkpoint |

**Rationale**:

- **500 tokens/chunk** balances context (enough for coherent answer) vs retrieval precision (focused results)
- **50-token overlap** ensures continuity when splitting mid-paragraph
- **Code examples** kept whole to preserve syntax
- **Lab instructions** slightly larger (600 tokens) to capture complete step sequences

### Token Calculation (OpenAI Tokenizer)

```python
import tiktoken

encoding = tiktoken.encoding_for_model("text-embedding-3-small")
tokens = encoding.encode(text)
num_tokens = len(tokens)
```

**Average Token Ratios**:

- **English prose**: ~1.3 tokens/word
- **Python code**: ~0.9 tokens/word (keywords tokenized efficiently)
- **YAML/JSON**: ~1.1 tokens/character (structured data)

---

## Metadata Schema

### Core Metadata Fields (All Chunks)

```python
{
    "chunk_id": "chunk_001_1.1.1_0",        # Unique identifier: chunk_<doc_id>_<section_id>_<index>
    "content_type": "theory",               # Enum: theory | code_example | lab_instructions | diagram | checkpoint
    "module_id": "1",                       # Module number (1-4)
    "module_title": "ROS 2 Nervous System", # Human-readable module name
    "chapter_id": "1.1",                    # Chapter number (e.g., "1.1", "2.3")
    "chapter_title": "ROS 2 Fundamentals",  # Human-readable chapter name
    "section_id": "1.1.1",                  # Section number (e.g., "1.1.1", "2.3.4")
    "section_title": "ROS 2 Architecture & Developer Workflow",  # Human-readable section name
    "page_number": 3,                       # Approximate page in 50-page book
    "source_file": "frontend/docs/module-01-ros2/01-ros2-fundamentals/1.1.1-architecture.md",  # Original file path
    "has_lab": true,                        # Boolean: Does this section reference a lab?
    "lab_id": "lab01-ros2-basics",          # Lab identifier (if has_lab=true)
    "keywords": ["ros2", "publisher", "subscriber", "qos", "dds"],  # Extracted keywords (5-10 per chunk)
    "created_at": "2025-12-07T00:00:00Z",   # Timestamp for version tracking
    "version": "3.0"                        # Content version (matches project-structure.md version)
}
```

### Lab-Specific Metadata (Lab Chunks)

```python
{
    "chunk_id": "chunk_lab01_0",
    "content_type": "lab_instructions",
    "module_id": "1",
    "module_title": "ROS 2 Nervous System",
    "lab_id": "lab01-ros2-basics",
    "lab_title": "Your First ROS 2 Node",
    "estimated_time": "30 min",             # Lab completion time
    "difficulty": "beginner",               # Enum: beginner | intermediate | advanced
    "learning_objectives": [                # List of objectives (3-5 per lab)
        "Create a ROS 2 Python package",
        "Implement a publisher node",
        "Test with ros2 topic echo"
    ],
    "prerequisites": ["ROS 2 Humble installed", "Section 1.1.1-1.1.2 completed"],
    "source_file": "labs/lab01-ros2-basics/README.md",
    "created_at": "2025-12-07T00:00:00Z",
    "version": "3.0"
}
```

### Code Example Metadata (Code Chunks)

```python
{
    "chunk_id": "chunk_001_1.1.2_code_0",
    "content_type": "code_example",
    "module_id": "1",
    "chapter_id": "1.1",
    "section_id": "1.1.2",
    "section_title": "rclpy Publisher Patterns",
    "language": "python",                   # Programming language (python | cpp | bash | yaml)
    "code_type": "complete_example",        # Enum: complete_example | snippet | configuration
    "runnable": true,                       # Boolean: Can this code run standalone?
    "keywords": ["rclpy", "Node", "create_publisher", "timer_callback"],
    "source_file": "frontend/docs/module-01-ros2/01-ros2-fundamentals/1.1.2-rclpy-patterns.md",
    "created_at": "2025-12-07T00:00:00Z",
    "version": "3.0"
}
```

---

## Embedding Pipeline

### OpenAI text-embedding-3-small Configuration

**Model Specifications**:

- **Dimensions**: 1536 (default, can be reduced to 512/256 with `dimensions` parameter)
- **Max Input Tokens**: 8191 tokens per request
- **Cost**: $0.02 per 1M tokens
- **Performance**: ~3000 documents/second on batch API

**Embedding Generation (Python)**:

```python
from openai import OpenAI
import numpy as np

client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

def embed_chunk(text: str, metadata: dict) -> dict:
    """Generate embedding for a single chunk with metadata."""
    response = client.embeddings.create(
        model="text-embedding-3-small",
        input=text,
        encoding_format="float"  # Returns list of floats (1536 dimensions)
    )

    embedding = response.data[0].embedding

    return {
        "id": metadata["chunk_id"],
        "vector": embedding,
        "payload": metadata  # All metadata fields from schema above
    }

def embed_batch(chunks: list[dict], batch_size: int = 100) -> list[dict]:
    """Batch embed multiple chunks for efficiency."""
    embeddings = []

    for i in range(0, len(chunks), batch_size):
        batch = chunks[i:i+batch_size]
        texts = [chunk["text"] for chunk in batch]

        response = client.embeddings.create(
            model="text-embedding-3-small",
            input=texts,
            encoding_format="float"
        )

        for j, embedding_data in enumerate(response.data):
            embeddings.append({
                "id": batch[j]["chunk_id"],
                "vector": embedding_data.embedding,
                "payload": batch[j]["metadata"]
            })

    return embeddings
```

**Estimated Embedding Costs** (50-page book):

- 50 pages × 500 words/page = 25,000 words
- 25,000 words × 1.3 tokens/word = 32,500 tokens (prose)
- Add code examples: +5,000 tokens
- Add lab instructions: +8,000 tokens
- **Total: ~45,500 tokens**
- **Cost**: 45,500 tokens × $0.02/1M = **$0.0009** (less than 1 cent, one-time)

---

## Qdrant Collection Schema

### Collection Configuration

```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PayloadSchemaType

client = QdrantClient(url="https://qdrant-cloud-endpoint", api_key="...")

# Create collection for Physical AI book content
client.create_collection(
    collection_name="physical_ai_book_v3",
    vectors_config=VectorParams(
        size=1536,  # text-embedding-3-small dimensions
        distance=Distance.COSINE  # Cosine similarity for semantic search
    ),
    # Enable payload indexing for fast filtering
    payload_schema={
        "module_id": PayloadSchemaType.KEYWORD,       # Exact match filter
        "chapter_id": PayloadSchemaType.KEYWORD,      # Exact match filter
        "section_id": PayloadSchemaType.KEYWORD,      # Exact match filter
        "content_type": PayloadSchemaType.KEYWORD,    # Filter by chunk type
        "lab_id": PayloadSchemaType.KEYWORD,          # Filter by lab
        "difficulty": PayloadSchemaType.KEYWORD,      # Filter labs by difficulty
        "keywords": PayloadSchemaType.KEYWORD,        # Multi-value keyword search
        "page_number": PayloadSchemaType.INTEGER,     # Range filter (e.g., pages 1-10)
        "has_lab": PayloadSchemaType.BOOL             # Boolean filter
    }
)
```

**Collection Settings**:

- **Distance Metric**: Cosine similarity (0.0 = orthogonal, 1.0 = identical)
- **Indexing**: HNSW (Hierarchical Navigable Small World) for fast approximate nearest neighbor search
- **Quantization**: Optional scalar quantization (1536 → 384 bytes) for 75% storage reduction with <1% accuracy loss

---

## Retrieval Strategy

### Hybrid Search: Vector + Metadata Filtering

**Query Pipeline**:

1. **User Query** → Embed with `text-embedding-3-small`
2. **Metadata Extraction** → Detect filters from query (e.g., "Module 2", "Lab 3", "QoS settings")
3. **Qdrant Search** → Vector similarity + payload filters
4. **Re-ranking** → Optional cross-encoder for precision (if response time allows)
5. **Context Assembly** → Top-k chunks + metadata for LLM prompt
6. **Citation Generation** → Format sources as "Module 1.1.2: rclpy Publisher Patterns"

### Example Query Scenarios

#### Scenario 1: General Concept Query

**User Query**: "How do I tune QoS settings for IMU data in ROS 2?"

**Processing**:

```python
# 1. Embed query
query_vector = embed_chunk("How do I tune QoS settings for IMU data in ROS 2?", {})["vector"]

# 2. Detect metadata filters (keyword extraction)
keywords_detected = ["qos", "imu", "ros2"]

# 3. Qdrant search with filters
results = client.search(
    collection_name="physical_ai_book_v3",
    query_vector=query_vector,
    limit=5,
    query_filter={
        "should": [
            {"key": "keywords", "match": {"any": ["qos", "imu"]}},
            {"key": "module_id", "match": {"value": "1"}}  # QoS is in Module 1
        ]
    },
    score_threshold=0.7  # Only return results with >70% similarity
)

# 4. Format context for LLM
context = "\n\n".join([
    f"[Source: Module {r.payload['module_id']}, Section {r.payload['section_id']}]\n{r.payload['text']}"
    for r in results
])

# 5. Generate answer with GPT-4o-mini
response = client.chat.completions.create(
    model="gpt-4o-mini",
    messages=[
        {"role": "system", "content": "You are a helpful assistant for the Physical AI & Humanoid Robotics textbook. Answer questions using the provided context and cite sources."},
        {"role": "user", "content": f"Context:\n{context}\n\nQuestion: {user_query}"}
    ]
)
```

**Expected Response**:

> "For IMU data in ROS 2, use **BEST_EFFORT** reliability with **VOLATILE** durability and **KEEP_LAST 1** history. This configuration prioritizes low latency over guaranteed delivery, which is ideal for high-frequency sensor data (200Hz+).
>
> **Source**: Module 1.1.3 (DDS & QoS Tuning), Table: QoS Profiles for Common Use Cases"

---

#### Scenario 2: Lab-Specific Query

**User Query**: "What are the prerequisites for Lab 2?"

**Processing**:

```python
query_vector = embed_chunk("What are the prerequisites for Lab 2?", {})["vector"]

results = client.search(
    collection_name="physical_ai_book_v3",
    query_vector=query_vector,
    limit=3,
    query_filter={
        "must": [
            {"key": "content_type", "match": {"value": "lab_instructions"}},
            {"key": "lab_id", "match": {"value": "lab02-urdf-humanoid"}}
        ]
    }
)
```

**Expected Response**:

> "Lab 2 (URDF for Humanoid) requires:
>
> 1. Completion of Lab 1 (Your First ROS 2 Node)
> 2. Understanding of Section 1.2.1-1.2.3 (URDF & Xacro Basics)
> 3. ROS 2 Humble installed with `joint_state_publisher_gui`
>
> **Estimated Time**: 45 minutes | **Difficulty**: Intermediate
>
> **Source**: Lab 2 README.md"

---

#### Scenario 3: Cross-Module Query

**User Query**: "How does Nav2 integrate with Isaac Sim?"

**Processing**:

```python
query_vector = embed_chunk("How does Nav2 integrate with Isaac Sim?", {})["vector"]

# No module filter → search across all modules
results = client.search(
    collection_name="physical_ai_book_v3",
    query_vector=query_vector,
    limit=8,
    query_filter={
        "should": [
            {"key": "keywords", "match": {"any": ["nav2", "isaac", "simulation"]}},
            {"key": "module_id", "match": {"any": ["3"]}}  # Module 3 likely has integration details
        ]
    }
)
```

**Expected Response**:

> "Nav2 integrates with Isaac Sim through the **Isaac ROS Nav2 bridge**, which provides:
>
> 1. **TF Tree Synchronization**: Isaac Sim publishes robot transforms to ROS 2 `/tf` and `/tf_static`
> 2. **Sensor Data Forwarding**: Depth cameras, LiDAR, and IMU data published as ROS 2 topics
> 3. **Command Velocity Subscription**: Nav2 publishes `/cmd_vel`, Isaac Sim subscribes and actuates robot
>
> **Configuration Example**: See Module 3.1.3 (Isaac ROS Integration) for `isaac_ros_navigation.launch.py`
>
> **Sources**:
>
> - Module 3.1.3 (Isaac ROS Integration)
> - Module 3.3.2 (Nav2 Configuration for Humanoids)"

---

## Postgres Schema (Metadata Storage)

Qdrant stores vectors, but Postgres stores structured metadata for analytics and auditing.

### Tables

#### `documents` (Source Files)

```sql
CREATE TABLE documents (
    id SERIAL PRIMARY KEY,
    module_id INTEGER NOT NULL,
    module_title VARCHAR(255) NOT NULL,
    chapter_id VARCHAR(10),
    chapter_title VARCHAR(255),
    section_id VARCHAR(10),
    section_title VARCHAR(255),
    source_file TEXT NOT NULL UNIQUE,
    content_type VARCHAR(50) NOT NULL,  -- theory | lab_instructions | code_example
    word_count INTEGER,
    page_number INTEGER,
    has_lab BOOLEAN DEFAULT FALSE,
    lab_id VARCHAR(100),
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW(),
    version VARCHAR(10) DEFAULT '3.0'
);

CREATE INDEX idx_documents_module ON documents(module_id);
CREATE INDEX idx_documents_chapter ON documents(chapter_id);
CREATE INDEX idx_documents_section ON documents(section_id);
CREATE INDEX idx_documents_lab ON documents(lab_id);
```

#### `chunks` (Individual Embeddings)

```sql
CREATE TABLE chunks (
    id SERIAL PRIMARY KEY,
    chunk_id VARCHAR(100) NOT NULL UNIQUE,  -- Maps to Qdrant point ID
    document_id INTEGER REFERENCES documents(id) ON DELETE CASCADE,
    chunk_index INTEGER NOT NULL,  -- Position in document (0, 1, 2, ...)
    content_type VARCHAR(50) NOT NULL,
    text_content TEXT NOT NULL,
    token_count INTEGER NOT NULL,
    keywords TEXT[],  -- PostgreSQL array of keywords
    embedding_model VARCHAR(100) DEFAULT 'text-embedding-3-small',
    created_at TIMESTAMP DEFAULT NOW()
);

CREATE INDEX idx_chunks_document ON chunks(document_id);
CREATE INDEX idx_chunks_chunk_id ON chunks(chunk_id);
CREATE INDEX idx_chunks_keywords ON chunks USING GIN(keywords);  -- Full-text search on keywords
```

#### `query_logs` (RAG Analytics)

```sql
CREATE TABLE query_logs (
    id SERIAL PRIMARY KEY,
    user_query TEXT NOT NULL,
    retrieved_chunk_ids TEXT[],  -- Array of chunk IDs returned
    response_text TEXT,
    model_used VARCHAR(100) DEFAULT 'gpt-4o-mini',
    latency_ms INTEGER,  -- Query processing time
    feedback_score INTEGER CHECK (feedback_score BETWEEN 1 AND 5),  -- Optional user rating
    created_at TIMESTAMP DEFAULT NOW()
);

CREATE INDEX idx_query_logs_created_at ON query_logs(created_at);
```

**Usage**: Track popular queries, identify gaps in content, measure retrieval accuracy

---

## Indexing Pipeline (seed-vector-db.py)

### High-Level Workflow

```python
# 1. Discover all markdown files
markdown_files = discover_markdown_files("frontend/docs/", "labs/")

# 2. Parse each file into chunks
for file_path in markdown_files:
    metadata = extract_metadata(file_path)  # Module, chapter, section from path
    chunks = chunk_markdown(file_path, chunk_size=500, overlap=50)

    # 3. Generate embeddings
    embeddings = embed_batch(chunks)

    # 4. Insert into Qdrant
    client.upsert(
        collection_name="physical_ai_book_v3",
        points=[
            {"id": emb["id"], "vector": emb["vector"], "payload": emb["payload"]}
            for emb in embeddings
        ]
    )

    # 5. Insert metadata into Postgres
    insert_into_postgres(metadata, chunks)

# 6. Validate indexing
print(f"Indexed {len(markdown_files)} documents")
print(f"Generated {sum([len(c) for c in all_chunks])} chunks")
print(f"Qdrant collection size: {client.count('physical_ai_book_v3')}")
```

### Chunking Implementation

```python
import re
import tiktoken

def chunk_markdown(file_path: str, chunk_size: int = 500, overlap: int = 50) -> list[dict]:
    """Split markdown file into chunks with metadata."""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    encoding = tiktoken.encoding_for_model("text-embedding-3-small")

    # Split by headings (##, ###, ####)
    sections = re.split(r'\n(#{2,4})\s+(.+)\n', content)

    chunks = []
    current_chunk = ""
    current_tokens = 0

    for i in range(0, len(sections), 3):
        section_text = sections[i] if i < len(sections) else ""
        section_tokens = len(encoding.encode(section_text))

        if current_tokens + section_tokens > chunk_size and current_chunk:
            # Save current chunk
            chunks.append({
                "text": current_chunk.strip(),
                "token_count": current_tokens
            })

            # Start new chunk with overlap
            overlap_text = current_chunk[-overlap:] if len(current_chunk) > overlap else ""
            current_chunk = overlap_text + section_text
            current_tokens = len(encoding.encode(current_chunk))
        else:
            current_chunk += section_text
            current_tokens += section_tokens

    # Add final chunk
    if current_chunk.strip():
        chunks.append({
            "text": current_chunk.strip(),
            "token_count": current_tokens
        })

    return chunks
```

---

## Retrieval API Endpoints (FastAPI)

### `/api/query` (Main RAG Endpoint)

```python
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

app = FastAPI()

class QueryRequest(BaseModel):
    query: str
    module_filter: int | None = None  # Optional: restrict to specific module
    top_k: int = 5
    score_threshold: float = 0.7

class QueryResponse(BaseModel):
    answer: str
    sources: list[dict]  # [{module, chapter, section, title, score}, ...]
    latency_ms: int

@app.post("/api/query", response_model=QueryResponse)
async def query_rag(request: QueryRequest):
    start_time = time.time()

    # 1. Embed query
    query_vector = embed_chunk(request.query, {})["vector"]

    # 2. Search Qdrant with filters
    filters = {}
    if request.module_filter:
        filters["must"] = [{"key": "module_id", "match": {"value": str(request.module_filter)}}]

    results = client.search(
        collection_name="physical_ai_book_v3",
        query_vector=query_vector,
        limit=request.top_k,
        query_filter=filters,
        score_threshold=request.score_threshold
    )

    # 3. Format context
    context = "\n\n".join([
        f"[Module {r.payload['module_id']}.{r.payload['section_id']}: {r.payload['section_title']}]\n{r.payload['text']}"
        for r in results
    ])

    # 4. Generate answer with GPT-4o-mini
    response = openai_client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {"role": "system", "content": "You are a helpful assistant for the Physical AI & Humanoid Robotics textbook. Answer questions using the provided context and cite sources."},
            {"role": "user", "content": f"Context:\n{context}\n\nQuestion: {request.query}"}
        ]
    )

    # 5. Format sources
    sources = [
        {
            "module": r.payload["module_id"],
            "chapter": r.payload["chapter_id"],
            "section": r.payload["section_id"],
            "title": r.payload["section_title"],
            "score": r.score
        }
        for r in results
    ]

    latency = int((time.time() - start_time) * 1000)

    return QueryResponse(
        answer=response.choices[0].message.content,
        sources=sources,
        latency_ms=latency
    )
```

---

## Validation & Testing

### Indexing Tests

```python
def test_indexing():
    # Test 1: Verify all 50 pages indexed
    doc_count = client.count("physical_ai_book_v3")
    assert doc_count > 100, f"Expected >100 chunks, got {doc_count}"

    # Test 2: Verify metadata completeness
    sample = client.scroll("physical_ai_book_v3", limit=10)[0]
    for point in sample:
        assert "module_id" in point.payload
        assert "section_id" in point.payload
        assert "content_type" in point.payload

    # Test 3: Verify lab chunks indexed
    lab_results = client.search(
        collection_name="physical_ai_book_v3",
        query_vector=[0.0] * 1536,  # Dummy vector
        limit=1,
        query_filter={"must": [{"key": "content_type", "match": {"value": "lab_instructions"}}]}
    )
    assert len(lab_results) > 0, "No lab chunks found"
```

### Retrieval Tests

```python
def test_retrieval():
    # Test 1: Simple concept query
    response = query_rag(QueryRequest(query="What is a ROS 2 publisher?", top_k=3))
    assert "publisher" in response.answer.lower()
    assert len(response.sources) > 0
    assert response.sources[0]["module"] == "1"

    # Test 2: Lab-specific query
    response = query_rag(QueryRequest(query="Lab 1 prerequisites", top_k=2))
    assert any("lab01" in s["section"] for s in response.sources)

    # Test 3: Cross-module query
    response = query_rag(QueryRequest(query="Isaac Sim and Nav2 integration", top_k=5))
    module_ids = {s["module"] for s in response.sources}
    assert "3" in module_ids, "Module 3 (Isaac) should be in sources"
```

---

## Performance Benchmarks

### Expected Metrics (Production)

| Metric                         | Target | Notes                                      |
| ------------------------------ | ------ | ------------------------------------------ |
| **Indexing Time**              | <2 min | 50 pages, 100-150 chunks, batch embedding  |
| **Query Latency (p95)**        | <800ms | Qdrant search (50ms) + GPT-4o-mini (700ms) |
| **Retrieval Accuracy (Top-5)** | >85%   | Manual evaluation on 50 test queries       |
| **Storage (Qdrant)**           | ~5MB   | 1536 dims × 150 chunks × 4 bytes/float     |
| **Monthly Cost**               | <$6    | RAG queries + embeddings (see research.md) |

---

## Future Enhancements

1. **Semantic Caching**: Cache common queries (e.g., "What is ROS 2?") to reduce OpenAI API costs
2. **Cross-Encoder Re-ranking**: Use `cross-encoder/ms-marco-MiniLM-L-12-v2` to re-rank top-20 results for better precision
3. **Multi-Modal RAG**: Index Mermaid diagrams as images, retrieve with CLIP embeddings for visual queries
4. **Feedback Loop**: Use thumbs up/down ratings in `query_logs` to fine-tune retrieval thresholds
5. **Version Control**: Store multiple versions of chunks (v3.0, v3.1) to support book updates without re-indexing

---

## Summary

This data model supports:

- ✅ **Precise Chunking**: 500-token chunks with 50-token overlap, preserving context
- ✅ **Rich Metadata**: Module/chapter/section/lab identifiers for targeted filtering
- ✅ **Efficient Embedding**: OpenAI text-embedding-3-small ($0.02/1M tokens, <1 cent for full book)
- ✅ **Hybrid Retrieval**: Vector similarity + keyword/module filters for accurate results
- ✅ **Citation Generation**: Automatic source attribution (Module 1.1.2, Lab 3, etc.)
- ✅ **Analytics**: Postgres query logs for measuring RAG performance and content gaps

**Next Steps**: Implement `scripts/rag/seed-vector-db.py` following this specification.
