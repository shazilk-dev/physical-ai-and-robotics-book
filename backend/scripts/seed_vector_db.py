"""
Content Chunking and Vector Database Seeding Script

This script:
1. Reads markdown files from frontend/docs/module-01-ros2/
2. Parses metadata (module, chapter, section)
3. Chunks text into 500-token segments with 50-token overlap
4. Generates embeddings using OpenAI
5. Uploads to Qdrant vector database
"""

import os
import sys
import re
from pathlib import Path
from typing import List, Dict, Any
import tiktoken

# Add parent directory to path to import RAGService
sys.path.append(str(Path(__file__).parent.parent))

from app.services.rag import get_rag_service


def count_tokens(text: str, model: str = "gpt-4o-mini") -> int:
    """Count tokens in text using tiktoken"""
    encoding = tiktoken.encoding_for_model(model)
    return len(encoding.encode(text))


def chunk_text(text: str, max_tokens: int = 500, overlap_tokens: int = 50) -> List[str]:
    """
    Split text into chunks of max_tokens with overlap_tokens overlap
    
    Args:
        text: Text to chunk
        max_tokens: Maximum tokens per chunk
        overlap_tokens: Number of tokens to overlap between chunks
        
    Returns:
        List of text chunks
    """
    encoding = tiktoken.encoding_for_model("gpt-4o-mini")
    tokens = encoding.encode(text)
    
    chunks = []
    start = 0
    
    while start < len(tokens):
        # Get chunk of tokens
        end = min(start + max_tokens, len(tokens))
        chunk_tokens = tokens[start:end]
        
        # Decode back to text
        chunk_text = encoding.decode(chunk_tokens)
        chunks.append(chunk_text)
        
        # Move start forward, accounting for overlap
        start = end - overlap_tokens
        
        # Break if we've covered all tokens
        if end == len(tokens):
            break
    
    return chunks


def extract_metadata_from_path(file_path: Path, base_dir: Path) -> Dict[str, str]:
    """
    Extract metadata from file path
    
    Example:
    frontend/docs/module-01-ros2/ros2-fundamentals/1.1.1-architecture.md
    -> module: "module-01-ros2"
    -> chapter: "1.1"
    -> section: "1.1.1"
    -> title: "Architecture"
    """
    relative_path = file_path.relative_to(base_dir)
    parts = relative_path.parts
    
    # Extract module (e.g., "module-01-ros2")
    module = parts[0] if len(parts) > 0 else "unknown"
    
    # Extract chapter folder (e.g., "ros2-fundamentals" or "02-urdf-robot-description")
    chapter_folder = parts[1] if len(parts) > 1 else "unknown"
    
    # Extract section number and title from filename (e.g., "1.1.1-architecture.md")
    filename = file_path.stem  # Remove .md extension
    
    # Match pattern like "1.1.1-architecture" or "1.2.3-sensors"
    match = re.match(r'(\d+\.\d+\.\d+)-(.+)', filename)
    if match:
        section_num = match.group(1)
        section_title = match.group(2).replace('-', ' ').title()
        
        # Extract chapter number (e.g., "1.1" from "1.1.1")
        chapter_num = '.'.join(section_num.split('.')[:2])
    else:
        # Handle special cases like "overview.md" or "intro.md"
        section_num = "0.0.0"
        chapter_num = "0.0"
        section_title = filename.replace('-', ' ').title()
    
    return {
        "module": module,
        "chapter": chapter_num,
        "chapter_folder": chapter_folder,
        "section": section_num,
        "section_title": section_title,
        "file_path": str(file_path)
    }


def clean_markdown(text: str) -> str:
    """
    Clean markdown for better chunking
    - Remove excessive whitespace
    - Preserve code blocks
    - Keep section headers
    """
    # Remove multiple consecutive blank lines
    text = re.sub(r'\n{3,}', '\n\n', text)
    
    # Remove markdown comments
    text = re.sub(r'<!--.*?-->', '', text, flags=re.DOTALL)
    
    # Preserve headers but ensure spacing
    text = re.sub(r'\n(#{1,6})\s+', r'\n\n\1 ', text)
    
    return text.strip()


def process_file(file_path: Path, base_dir: Path, rag_service) -> int:
    """
    Process a single markdown file
    
    Returns:
        Number of chunks uploaded
    """
    print(f"\n📄 Processing: {file_path.name}")
    
    # Read file
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Extract metadata
    metadata = extract_metadata_from_path(file_path, base_dir)
    
    # Clean markdown
    content = clean_markdown(content)
    
    # Skip if too short
    if count_tokens(content) < 50:
        print(f"   ⏭️  Skipped (too short)")
        return 0
    
    # Chunk text
    chunks = chunk_text(content, max_tokens=500, overlap_tokens=50)
    print(f"   ✂️  Created {len(chunks)} chunks")
    
    # Upload chunks to Qdrant
    uploaded = 0
    for i, chunk in enumerate(chunks):
        try:
            # Generate embedding
            embedding = rag_service.generate_embedding(chunk)
            
            # Create payload with metadata
            payload = {
                "content": chunk,
                "module": metadata["module"],
                "chapter": metadata["chapter"],
                "section": metadata["section"],
                "section_title": metadata["section_title"],
                "file_path": metadata["file_path"],
                "chunk_index": i
            }
            
            # Upload to Qdrant
            from qdrant_client.models import PointStruct
            point_id = f"{metadata['section']}_{i}"
            
            rag_service.qdrant_client.upsert(
                collection_name=rag_service.collection_name,
                points=[
                    PointStruct(
                        id=point_id,
                        vector=embedding,
                        payload=payload
                    )
                ]
            )
            
            uploaded += 1
            print(f"   📤 Uploaded chunk {i+1}/{len(chunks)}")
            
        except Exception as e:
            print(f"   ❌ Error uploading chunk {i}: {e}")
    
    return uploaded


def main():
    """Main function"""
    print("🚀 Starting Vector Database Seeding")
    print("=" * 60)
    
    # Initialize RAG service
    print("\n🔧 Initializing RAG service...")
    rag_service = get_rag_service()
    
    # Create collection if not exists
    print(f"🗄️  Creating collection: {rag_service.collection_name}")
    rag_service.create_collection()
    
    # Find base directory
    script_dir = Path(__file__).parent
    project_root = script_dir.parent.parent
    docs_dir = project_root / "frontend" / "docs"
    
    print(f"\n📂 Scanning directory: {docs_dir}")
    
    if not docs_dir.exists():
        print(f"❌ Error: Directory not found: {docs_dir}")
        return
    
    # Find all markdown files in module-01-ros2
    module_dir = docs_dir / "module-01-ros2"
    if not module_dir.exists():
        print(f"❌ Error: Module directory not found: {module_dir}")
        return
    
    markdown_files = list(module_dir.rglob("*.md"))
    print(f"📋 Found {len(markdown_files)} markdown files")
    
    # Also include intro.md if it exists
    intro_file = docs_dir / "intro.md"
    if intro_file.exists():
        markdown_files.append(intro_file)
        print(f"📋 Including intro.md")
    
    # Process each file
    total_chunks = 0
    for file_path in markdown_files:
        try:
            chunks_uploaded = process_file(file_path, docs_dir, rag_service)
            total_chunks += chunks_uploaded
        except Exception as e:
            print(f"❌ Error processing {file_path.name}: {e}")
    
    # Print summary
    print("\n" + "=" * 60)
    print(f"✅ Seeding complete!")
    print(f"📊 Files processed: {len(markdown_files)}")
    print(f"📊 Total chunks uploaded: {total_chunks}")
    
    # Get collection stats
    try:
        collection_info = rag_service.qdrant_client.get_collection(
            rag_service.collection_name
        )
        print(f"📊 Collection points: {collection_info.points_count}")
    except Exception as e:
        print(f"⚠️  Could not get collection stats: {e}")


if __name__ == "__main__":
    main()
