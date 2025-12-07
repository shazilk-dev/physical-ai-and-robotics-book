"""
Seed Vector Database
Processes markdown files and creates embeddings in Qdrant
"""

import os
import hashlib
import re
from pathlib import Path
from typing import List, Dict
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from openai import OpenAI
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize clients
openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

COLLECTION_NAME = "physical_ai_textbook"
EMBEDDING_MODEL = "text-embedding-3-small"
EMBEDDING_DIMENSION = 1536

def create_collection():
    """Create Qdrant collection if it doesn't exist"""
    try:
        qdrant_client.get_collection(COLLECTION_NAME)
        print(f"✅ Collection '{COLLECTION_NAME}' already exists")
    except:
        qdrant_client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(
                size=EMBEDDING_DIMENSION,
                distance=Distance.COSINE
            )
        )
        print(f"✅ Created collection '{COLLECTION_NAME}'")

def chunk_markdown(content: str, chunk_size: int = 800, overlap: int = 100) -> List[str]:
    """Split markdown content into chunks"""
    # Remove front matter
    content = re.sub(r'^---\n.*?\n---\n', '', content, flags=re.DOTALL)
    
    # Split by paragraphs
    paragraphs = content.split('\n\n')
    
    chunks = []
    current_chunk = ""
    
    for para in paragraphs:
        if len(current_chunk) + len(para) < chunk_size:
            current_chunk += para + "\n\n"
        else:
            if current_chunk:
                chunks.append(current_chunk.strip())
            current_chunk = para + "\n\n"
    
    if current_chunk:
        chunks.append(current_chunk.strip())
    
    return chunks

def create_embedding(text: str) -> List[float]:
    """Generate embedding using OpenAI"""
    response = openai_client.embeddings.create(
        input=text,
        model=EMBEDDING_MODEL
    )
    return response.data[0].embedding

def process_markdown_file(file_path: Path, base_path: Path) -> List[Dict]:
    """Process a single markdown file"""
    print(f"📄 Processing: {file_path.relative_to(base_path)}")
    
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    chunks = chunk_markdown(content)
    documents = []
    
    for idx, chunk in enumerate(chunks):
        # Generate unique document ID
        doc_id = f"{file_path.stem}_{idx}"
        content_hash = hashlib.sha256(chunk.encode()).hexdigest()
        
        # Create embedding
        embedding = create_embedding(chunk)
        
        documents.append({
            "id": doc_id,
            "vector": embedding,
            "payload": {
                "text": chunk,
                "source_file": str(file_path.relative_to(base_path)),
                "chunk_index": idx,
                "content_hash": content_hash
            }
        })
    
    print(f"  ✅ Created {len(chunks)} chunks")
    return documents

def seed_database():
    """Main function to seed the vector database"""
    print("🚀 Starting vector database seeding...")
    
    # Create collection
    create_collection()
    
    # Find all markdown files in docs/
    docs_path = Path(__file__).parent.parent.parent / "docs" / "docs"
    markdown_files = list(docs_path.rglob("*.md"))
    
    print(f"\n📚 Found {len(markdown_files)} markdown files")
    
    all_documents = []
    
    # Process each file
    for file_path in markdown_files:
        documents = process_markdown_file(file_path, docs_path)
        all_documents.extend(documents)
    
    # Upload to Qdrant
    print(f"\n⬆️  Uploading {len(all_documents)} chunks to Qdrant...")
    
    points = [
        PointStruct(
            id=idx,
            vector=doc["vector"],
            payload=doc["payload"]
        )
        for idx, doc in enumerate(all_documents)
    ]
    
    qdrant_client.upsert(
        collection_name=COLLECTION_NAME,
        points=points
    )
    
    print(f"✅ Successfully seeded {len(all_documents)} document chunks!")
    print(f"📊 Collection: {COLLECTION_NAME}")
    print(f"🔍 Model: {EMBEDDING_MODEL}")

if __name__ == "__main__":
    seed_database()
