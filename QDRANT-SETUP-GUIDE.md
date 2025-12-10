# Qdrant Vector Database Setup Guide

## Physical AI & Humanoid Robotics Textbook

This guide will walk you through setting up Qdrant Cloud for your RAG (Retrieval Augmented Generation) chatbot.

---

## 📋 Prerequisites

- OpenAI API Key (for embeddings)
- Qdrant Cloud account (free tier available)
- Python 3.10+
- Your textbook content in markdown files

---

## 🎯 Step 1: Choose Your Qdrant Setup

### Option A: Qdrant Cloud (Recommended for Production)

**Use Case Selected: Global Search**

- ✅ Search across entire textbook collection
- ✅ Optional filters by module/chapter/section
- ✅ Single collection for all content

**Why this approach?**

- Students search across all topics
- Filter by module when needed (e.g., "ROS 2 only")
- Simple to maintain and scale
- Cost-effective for educational content

### Option B: Qdrant Local (Development Only)

For testing without cloud, but NOT recommended for production.

---

## 🚀 Step 2: Create Qdrant Cloud Cluster

### 2.1 Sign Up for Qdrant Cloud

1. Go to: https://cloud.qdrant.io/
2. Click **"Sign Up"** or **"Start Free"**
3. Choose authentication method:
   - Google
   - GitHub
   - Email

### 2.2 Create Your First Cluster

1. After login, click **"Create Cluster"**
2. Configure your cluster:

   ```
   Name: physical-ai-textbook
   Region: Choose closest to your users
     - US: us-east-1 (Virginia) or us-west-2 (Oregon)
     - EU: europe-west3 (Frankfurt)
     - Asia: asia-southeast1 (Singapore)

   Plan: Free Tier (1GB RAM, perfect for textbook)
   ```

3. Click **"Create"**
4. Wait 2-3 minutes for provisioning

### 2.3 Get Your Credentials

1. Once cluster is ready, click on it
2. You'll see:

   - **Cluster URL**: `https://xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx.region.gcp.cloud.qdrant.io:6333`
   - **API Key**: Click "Generate API Key"

3. **IMPORTANT**: Copy both immediately!

---

## 🔧 Step 3: Configure Your Application

### 3.1 Update `.env` File

Your backend `.env` already has the structure. Update it:

```bash
# OpenAI (for embeddings and chat)
OPENAI_API_KEY=sk-proj-your-actual-key-here

# Qdrant Vector Store
QDRANT_URL=https://your-cluster-id.region.gcp.cloud.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-api-key-here
QDRANT_COLLECTION_NAME=physical_ai_textbook

# Qdrant Collection Settings
QDRANT_VECTOR_SIZE=1536  # OpenAI text-embedding-3-small
QDRANT_DISTANCE_METRIC=COSINE
```

### 3.2 Install Required Packages

```bash
cd backend
pip install qdrant-client openai python-dotenv
```

Or add to `requirements.txt`:

```
qdrant-client>=1.7.0
openai>=1.12.0
python-dotenv>=1.0.0
```

---

## 📊 Step 4: Design Your Collection Schema

### 4.1 Collection Structure (Already Implemented)

```python
Collection: physical_ai_textbook
├── Vectors: 1536 dimensions (text-embedding-3-small)
├── Distance: Cosine Similarity
└── Payload Schema:
    ├── content: str          # The actual text chunk
    ├── module: str           # e.g., "module-01-ros2"
    ├── chapter: str          # e.g., "1.1 ROS 2 Fundamentals"
    ├── section: str          # e.g., "1.1.1 Architecture"
    ├── file_path: str        # e.g., "docs/module-01-ros2/..."
    ├── chunk_index: int      # Position in document
    └── metadata: dict        # Additional info (optional)
```

### 4.2 Why This Structure?

- **Global Search**: Single collection for all content
- **Filtered Search**: Use `module` field to filter by topic
- **Source Attribution**: Track where answer came from
- **Chunk Management**: Track position for context

---

## 🔨 Step 5: Initialize Collection

### 5.1 Create Collection Script

The collection is auto-created in your `rag.py`, but let's verify:

```bash
cd backend
python -c "from app.services.rag import RAGService; RAGService().create_collection()"
```

Expected output:

```
Created collection 'physical_ai_textbook'
```

### 5.2 Verify Collection

Check via Qdrant Cloud Dashboard:

1. Go to your cluster
2. Click **"Collections"** tab
3. You should see `physical_ai_textbook`
4. Click to view:
   - Vector dimension: 1536
   - Distance: Cosine
   - Points: 0 (empty until we add data)

---

## 📝 Step 6: Prepare and Upload Content

### 6.1 Create Data Ingestion Script

Create `backend/scripts/seed_vector_db.py`:

```python
"""
Seed Qdrant with textbook content
Reads markdown files and uploads to vector database
"""
import os
import sys
from pathlib import Path
from typing import List, Dict
import hashlib
import re

# Add parent directory to path
sys.path.append(str(Path(__file__).parent.parent))

from app.services.rag import RAGService
from qdrant_client.models import PointStruct

class TextbookIngestion:
    def __init__(self):
        self.rag_service = RAGService()
        self.docs_dir = Path(__file__).parent.parent.parent / "frontend" / "docs"
        self.chunk_size = 800  # characters per chunk
        self.chunk_overlap = 200  # overlap between chunks

    def chunk_text(self, text: str) -> List[str]:
        """Split text into overlapping chunks"""
        chunks = []
        start = 0
        text_length = len(text)

        while start < text_length:
            end = start + self.chunk_size

            # Try to break at sentence boundary
            if end < text_length:
                # Look for sentence ending
                last_period = text[start:end].rfind('.')
                last_newline = text[start:end].rfind('\n')
                break_point = max(last_period, last_newline)

                if break_point > self.chunk_size * 0.5:  # At least 50% of chunk
                    end = start + break_point + 1

            chunk = text[start:end].strip()
            if chunk:
                chunks.append(chunk)

            start = end - self.chunk_overlap

        return chunks

    def extract_metadata(self, file_path: Path) -> Dict[str, str]:
        """Extract metadata from file path"""
        parts = file_path.parts

        # Determine module
        module = "intro"
        for part in parts:
            if part.startswith("module-"):
                module = part
                break
            elif part == "labs":
                module = "labs"
                break

        # Get chapter from filename
        chapter = file_path.stem.replace("-", " ").title()

        return {
            "module": module,
            "chapter": chapter,
            "file_path": str(file_path.relative_to(self.docs_dir.parent))
        }

    def process_markdown_file(self, file_path: Path) -> List[Dict]:
        """Process single markdown file into chunks"""
        print(f"📄 Processing: {file_path.name}")

        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Remove frontmatter if exists
            content = re.sub(r'^---\n.*?\n---\n', '', content, flags=re.DOTALL)

            # Extract metadata
            metadata = self.extract_metadata(file_path)

            # Split into chunks
            chunks = self.chunk_text(content)

            # Create documents with metadata
            documents = []
            for idx, chunk in enumerate(chunks):
                doc = {
                    "content": chunk,
                    "module": metadata["module"],
                    "chapter": metadata["chapter"],
                    "section": f"{metadata['chapter']} - Part {idx + 1}",
                    "file_path": metadata["file_path"],
                    "chunk_index": idx
                }
                documents.append(doc)

            print(f"  ✅ Created {len(documents)} chunks")
            return documents

        except Exception as e:
            print(f"  ❌ Error: {e}")
            return []

    def upload_to_qdrant(self, documents: List[Dict]):
        """Upload documents to Qdrant"""
        print(f"\n🚀 Uploading {len(documents)} chunks to Qdrant...")

        points = []
        for idx, doc in enumerate(documents):
            # Generate embedding
            embedding = self.rag_service.generate_embedding(doc["content"])

            # Create unique ID
            doc_id = hashlib.md5(
                f"{doc['file_path']}-{doc['chunk_index']}".encode()
            ).hexdigest()

            # Create point
            point = PointStruct(
                id=doc_id,
                vector=embedding,
                payload={
                    "content": doc["content"],
                    "module": doc["module"],
                    "chapter": doc["chapter"],
                    "section": doc["section"],
                    "file_path": doc["file_path"],
                    "chunk_index": doc["chunk_index"]
                }
            )
            points.append(point)

            # Upload in batches of 100
            if len(points) >= 100:
                self.rag_service.qdrant_client.upsert(
                    collection_name=self.rag_service.collection_name,
                    points=points
                )
                print(f"  ✅ Uploaded batch of {len(points)} points")
                points = []

        # Upload remaining
        if points:
            self.rag_service.qdrant_client.upsert(
                collection_name=self.rag_service.collection_name,
                points=points
            )
            print(f"  ✅ Uploaded final batch of {len(points)} points")

        print("✨ Upload complete!")

    def ingest_all(self):
        """Main ingestion process"""
        print("🔍 Scanning for markdown files...")

        # Find all markdown files
        md_files = list(self.docs_dir.rglob("*.md"))
        print(f"📚 Found {len(md_files)} markdown files\n")

        # Process all files
        all_documents = []
        for md_file in md_files:
            docs = self.process_markdown_file(md_file)
            all_documents.extend(docs)

        print(f"\n📊 Total chunks created: {len(all_documents)}")

        # Upload to Qdrant
        if all_documents:
            self.upload_to_qdrant(all_documents)
        else:
            print("❌ No documents to upload")

def main():
    """Run ingestion"""
    print("="*60)
    print("  Qdrant Vector Database Ingestion")
    print("  Physical AI & Humanoid Robotics Textbook")
    print("="*60 + "\n")

    ingestion = TextbookIngestion()

    # Create collection if not exists
    ingestion.rag_service.create_collection()

    # Ingest content
    ingestion.ingest_all()

    print("\n" + "="*60)
    print("✨ Ingestion Complete!")
    print("="*60)

if __name__ == "__main__":
    main()
```

### 6.2 Run Ingestion

```bash
cd backend
python scripts/seed_vector_db.py
```

Expected output:

```
============================================================
  Qdrant Vector Database Ingestion
  Physical AI & Humanoid Robotics Textbook
============================================================

🔍 Scanning for markdown files...
📚 Found 15 markdown files

📄 Processing: intro.md
  ✅ Created 5 chunks
📄 Processing: 1.1.1-architecture.md
  ✅ Created 12 chunks
...

📊 Total chunks created: 247

🚀 Uploading 247 chunks to Qdrant...
  ✅ Uploaded batch of 100 points
  ✅ Uploaded batch of 100 points
  ✅ Uploaded final batch of 47 points
✨ Upload complete!

============================================================
✨ Ingestion Complete!
============================================================
```

---

## 🔍 Step 7: Test Your Setup

### 7.1 Create Test Script

Create `backend/scripts/test_qdrant.py`:

```python
"""
Test Qdrant vector search
"""
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

from app.services.rag import RAGService

def test_search():
    """Test vector search"""
    rag = RAGService()

    # Test queries
    queries = [
        "What is ROS 2 architecture?",
        "How do I create a URDF file?",
        "Explain quality of service in ROS 2"
    ]

    for query in queries:
        print(f"\n{'='*60}")
        print(f"Query: {query}")
        print('='*60)

        # Search
        results = rag.search_similar_chunks(query, limit=3)

        for idx, result in enumerate(results, 1):
            print(f"\nResult {idx} (Score: {result['score']:.4f})")
            print(f"Module: {result['module']}")
            print(f"Section: {result['section']}")
            print(f"Content: {result['content'][:200]}...")

def test_full_query():
    """Test full RAG pipeline"""
    rag = RAGService()

    question = "How does the ROS 2 publisher-subscriber pattern work?"
    print(f"\n{'='*60}")
    print(f"Question: {question}")
    print('='*60)

    response = rag.query(question, num_results=5)

    print(f"\nAnswer:\n{response['answer']}\n")
    print(f"Sources: {len(response['sources'])} chunks")
    print(f"Citations: {', '.join(response['citations'])}")

if __name__ == "__main__":
    print("Testing Qdrant Search...")
    test_search()

    print("\n" + "="*60)
    print("Testing Full RAG Pipeline...")
    test_full_query()
```

### 7.2 Run Tests

```bash
python scripts/test_qdrant.py
```

---

## 📈 Step 8: Monitor and Optimize

### 8.1 Check Collection Stats

Via Qdrant Dashboard:

1. Go to your cluster
2. Click **"Collections"** → `physical_ai_textbook`
3. View:
   - Total points (should match your chunks)
   - Memory usage
   - Query performance

### 8.2 Optimize Search Parameters

In your `rag.py`, you can tune:

```python
# Number of results
num_results = 5  # Increase for more context

# Filter by module
module_filter = "module-01-ros2"  # Focus on specific module

# Score threshold
min_score = 0.7  # Only return high-quality matches
```

### 8.3 Add Payload Indexing (Optional)

For faster filtered searches:

```python
from qdrant_client.models import PayloadSchemaType

# Create index on module field
rag_service.qdrant_client.create_payload_index(
    collection_name="physical_ai_textbook",
    field_name="module",
    field_schema=PayloadSchemaType.KEYWORD
)
```

---

## 🎯 Step 9: Advanced Features

### 9.1 Hybrid Search (Coming Soon)

Combine vector search with keyword matching:

```python
from qdrant_client.models import Filter, FieldCondition, MatchValue

# Search with keyword filter
results = qdrant_client.search(
    collection_name="physical_ai_textbook",
    query_vector=embedding,
    query_filter=Filter(
        must=[
            FieldCondition(
                key="content",
                match=MatchValue(value="ROS 2")
            )
        ]
    )
)
```

### 9.2 Multi-Vector Search

For different query types:

```python
# Create collection with multiple vectors
qdrant_client.create_collection(
    collection_name="physical_ai_textbook_v2",
    vectors_config={
        "semantic": VectorParams(size=1536, distance=Distance.COSINE),
        "code": VectorParams(size=768, distance=Distance.COSINE),
    }
)
```

### 9.3 Recommendation System

Find similar content:

```python
# Get similar sections
similar = qdrant_client.recommend(
    collection_name="physical_ai_textbook",
    positive=[point_id],  # Section user is reading
    limit=5
)
```

---

## 🔒 Step 10: Security Best Practices

### 10.1 Environment Variables

✅ **DO:**

- Use `.env` file for local development
- Use environment variables in production
- Add `.env` to `.gitignore`

❌ **DON'T:**

- Commit API keys to git
- Share credentials in code
- Use hardcoded keys

### 10.2 API Key Rotation

1. Generate new API key in Qdrant dashboard
2. Update `.env` file
3. Restart backend
4. Delete old key

### 10.3 Access Control

- Use separate clusters for dev/staging/production
- Limit API key permissions
- Monitor usage in dashboard

---

## 📚 Step 11: Cost Management

### 11.1 Free Tier Limits

Qdrant Cloud Free Tier:

- 1GB RAM
- ~1M vectors (depending on payload)
- Unlimited queries
- Perfect for textbook content

### 11.2 Optimization Tips

1. **Reduce Payload Size**: Store only necessary fields
2. **Efficient Chunking**: Balance size vs. quality
3. **Batch Operations**: Upload in batches
4. **Monitoring**: Check usage regularly

### 11.3 Upgrade Path

If you exceed free tier:

- **Basic**: $25/month (4GB RAM)
- **Pro**: $95/month (16GB RAM)
- **Enterprise**: Custom pricing

---

## 🚨 Troubleshooting

### Issue 1: Connection Timeout

```python
Error: Connection timeout to Qdrant
```

**Solution:**

- Check QDRANT_URL is correct
- Verify API key is valid
- Check network/firewall settings

### Issue 2: Collection Not Found

```python
Error: Collection 'physical_ai_textbook' not found
```

**Solution:**

```python
# Create collection
rag_service.create_collection()
```

### Issue 3: Embedding Dimension Mismatch

```python
Error: Vector dimension mismatch
```

**Solution:**

- Verify embedding model matches collection config
- `text-embedding-3-small` = 1536 dimensions
- `text-embedding-3-large` = 3072 dimensions

### Issue 4: Slow Queries

**Solution:**

1. Create payload indexes on filter fields
2. Reduce `num_results` parameter
3. Optimize chunk size
4. Consider upgrading cluster

---

## ✅ Checklist: You're Ready When...

- [ ] Qdrant Cloud cluster is running
- [ ] Collection `physical_ai_textbook` is created
- [ ] `.env` file is configured
- [ ] Content is ingested (seed script ran successfully)
- [ ] Test queries return relevant results
- [ ] Backend API `/api/chat` works
- [ ] Frontend chatbot displays answers with sources
- [ ] Production deployment configured (Vercel + Railway)

---

## 🎓 Next Steps

1. **Improve Chunking Strategy**: Experiment with chunk sizes
2. **Add Metadata**: Include difficulty level, prerequisites
3. **Implement Caching**: Cache frequent queries
4. **Add Analytics**: Track popular questions
5. **User Feedback**: Let students rate answer quality
6. **Expand Content**: Add more modules and labs

---

## 📞 Support Resources

- **Qdrant Docs**: https://qdrant.tech/documentation/
- **Qdrant Discord**: https://discord.gg/qdrant
- **OpenAI Docs**: https://platform.openai.com/docs/
- **Your GitHub Issues**: Report bugs in your repo

---

## 🎉 Congratulations!

You now have a fully functional RAG system with:

- ✅ Vector search powered by Qdrant
- ✅ Semantic embeddings from OpenAI
- ✅ Intelligent chat responses
- ✅ Source attribution
- ✅ Module-based filtering
- ✅ Scalable cloud infrastructure

**Your students can now ask questions and get instant, accurate answers from your textbook!** 🚀
