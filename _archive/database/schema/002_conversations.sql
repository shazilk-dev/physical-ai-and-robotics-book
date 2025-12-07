-- Conversations Table
-- Stores chat history between users and RAG chatbot

CREATE TABLE IF NOT EXISTS conversations (
    id SERIAL PRIMARY KEY,
    user_id INTEGER REFERENCES users(id) ON DELETE CASCADE,
    session_id UUID NOT NULL,
    
    -- Message data
    role VARCHAR(20) NOT NULL, -- 'user' or 'assistant'
    content TEXT NOT NULL,
    
    -- Context tracking
    chapter_context VARCHAR(255), -- Which chapter user was reading
    query_type VARCHAR(50), -- 'general', 'selection', 'clarification'
    
    -- RAG metadata
    retrieved_chunks INTEGER, -- Number of chunks retrieved
    confidence_score FLOAT, -- RAG confidence (0-1)
    
    -- Timestamps
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Indexes for efficient querying
CREATE INDEX idx_conversations_user_id ON conversations(user_id);
CREATE INDEX idx_conversations_session_id ON conversations(session_id);
CREATE INDEX idx_conversations_created_at ON conversations(created_at DESC);
CREATE INDEX idx_conversations_chapter_context ON conversations(chapter_context);

-- Composite index for user session history
CREATE INDEX idx_conversations_user_session ON conversations(user_id, session_id, created_at DESC);
