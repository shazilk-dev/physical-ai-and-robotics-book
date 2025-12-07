-- Document Embeddings Metadata Table
-- Tracks which documents have been embedded in Qdrant

CREATE TABLE IF NOT EXISTS document_embeddings (
    id SERIAL PRIMARY KEY,
    
    -- Document identification
    document_id VARCHAR(255) UNIQUE NOT NULL, -- Unique ID for the document chunk
    source_file VARCHAR(500) NOT NULL, -- Path to source markdown file
    chapter_id VARCHAR(100) NOT NULL, -- e.g., "01-module-1-ros2"
    section_id VARCHAR(100), -- e.g., "1.1.1-architecture"
    
    -- Content metadata
    content_type VARCHAR(50) NOT NULL, -- 'text', 'code', 'diagram_caption'
    chunk_index INTEGER NOT NULL, -- Position in document
    chunk_text TEXT NOT NULL, -- Actual text content
    chunk_length INTEGER NOT NULL, -- Character count
    
    -- Embedding metadata
    embedding_model VARCHAR(100) NOT NULL, -- e.g., "text-embedding-3-small"
    vector_dimension INTEGER NOT NULL, -- e.g., 1536
    qdrant_point_id UUID, -- Corresponding ID in Qdrant
    
    -- Versioning
    content_hash VARCHAR(64) NOT NULL, -- SHA256 of chunk_text for change detection
    version INTEGER DEFAULT 1,
    
    -- Timestamps
    embedded_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Indexes for efficient querying
CREATE INDEX idx_embeddings_document_id ON document_embeddings(document_id);
CREATE INDEX idx_embeddings_source_file ON document_embeddings(source_file);
CREATE INDEX idx_embeddings_chapter_id ON document_embeddings(chapter_id);
CREATE INDEX idx_embeddings_content_hash ON document_embeddings(content_hash);
CREATE INDEX idx_embeddings_qdrant_point_id ON document_embeddings(qdrant_point_id);

-- Composite index for chapter sections
CREATE INDEX idx_embeddings_chapter_section ON document_embeddings(chapter_id, section_id);

-- Trigger to update updated_at
CREATE TRIGGER document_embeddings_updated_at
BEFORE UPDATE ON document_embeddings
FOR EACH ROW
EXECUTE FUNCTION update_updated_at_column();
