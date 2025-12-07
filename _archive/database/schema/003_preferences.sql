-- User Preferences Table
-- Stores personalization settings

CREATE TABLE IF NOT EXISTS user_preferences (
    id SERIAL PRIMARY KEY,
    user_id INTEGER UNIQUE REFERENCES users(id) ON DELETE CASCADE,
    
    -- Content preferences
    preferred_difficulty VARCHAR(50) DEFAULT 'adaptive', -- beginner, advanced, adaptive
    preferred_language VARCHAR(10) DEFAULT 'en', -- en, ur
    show_code_examples BOOLEAN DEFAULT TRUE,
    show_diagrams BOOLEAN DEFAULT TRUE,
    
    -- Learning preferences
    learning_pace VARCHAR(50) DEFAULT 'moderate', -- slow, moderate, fast
    preferred_explanation_style VARCHAR(50) DEFAULT 'detailed', -- concise, detailed, visual
    
    -- UI preferences
    theme VARCHAR(20) DEFAULT 'system', -- light, dark, system
    font_size VARCHAR(20) DEFAULT 'medium', -- small, medium, large
    
    -- Progress tracking
    completed_modules TEXT[], -- Array of completed module IDs
    bookmarked_chapters TEXT[], -- Array of bookmarked chapter paths
    current_chapter VARCHAR(255), -- Last chapter visited
    
    -- Timestamps
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Index for faster user lookups
CREATE INDEX idx_user_preferences_user_id ON user_preferences(user_id);

-- Trigger to update updated_at
CREATE TRIGGER user_preferences_updated_at
BEFORE UPDATE ON user_preferences
FOR EACH ROW
EXECUTE FUNCTION update_updated_at_column();
