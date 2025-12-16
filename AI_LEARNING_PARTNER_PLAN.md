# AI Learning Partner Enhancement Plan
## Transform ChatWidget into an Intelligent Tutoring System

**Current State:** Basic RAG-powered Q&A chatbot
**Target State:** Adaptive, context-aware AI learning partner with multi-modal interaction

---

## 🎯 Vision

Create an AI tutor that:
- Adapts explanations to user's learning style and background
- Provides contextual help on any selected text
- Maintains conversation history for continuity
- Offers multiple interaction modes (quick, detailed, tutorial, quiz)
- Acts as a true learning companion, not just a search engine

---

## 📋 Feature Categories

### **Phase 1: Text Selection & Context Menu (Foundation)**
### **Phase 2: Conversation Modes & Personalization**
### **Phase 3: Context Awareness & Memory**
### **Phase 4: Advanced Learning Features**
### **Phase 5: Multi-modal Interaction**

---

## Phase 1: Text Selection & Context Menu (2-3 days)

### Feature: "Explain with AI" on Text Selection

**User Experience:**
1. User highlights any text in the documentation (e.g., "Pub/Sub pattern")
2. A floating tooltip appears: [🤖 Explain with AI] [📝 Simplify] [💡 Example]
3. Click triggers chatbot with pre-filled context
4. AI explains the selected concept

**Technical Implementation:**

#### Frontend Changes

**1. Add Selection Detection (`frontend/src/components/SelectionMenu/`)**

```tsx
// SelectionMenu.tsx
interface SelectionMenuProps {
  onExplain: (text: string, action: string) => void;
}

export default function SelectionMenu({ onExplain }: SelectionMenuProps) {
  const [selection, setSelection] = useState<string>('');
  const [position, setPosition] = useState<{ x: number; y: number } | null>(null);

  useEffect(() => {
    const handleSelection = () => {
      const selected = window.getSelection()?.toString().trim();
      if (selected && selected.length > 5) {
        const range = window.getSelection()?.getRangeAt(0);
        const rect = range?.getBoundingClientRect();
        if (rect) {
          setSelection(selected);
          setPosition({ x: rect.left + rect.width / 2, y: rect.top - 50 });
        }
      } else {
        setPosition(null);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  if (!position) return null;

  return (
    <div
      className={styles.selectionMenu}
      style={{
        position: 'fixed',
        left: position.x,
        top: position.y
      }}
    >
      <button onClick={() => onExplain(selection, 'explain')}>
        🤖 Explain
      </button>
      <button onClick={() => onExplain(selection, 'simplify')}>
        📝 Simplify
      </button>
      <button onClick={() => onExplain(selection, 'example')}>
        💡 Example
      </button>
      <button onClick={() => onExplain(selection, 'quiz')}>
        ❓ Quiz Me
      </button>
    </div>
  );
}
```

**2. Update ChatWidget to Accept Context**

```tsx
// ChatWidget.tsx additions
interface ChatContext {
  selectedText?: string;
  action?: 'explain' | 'simplify' | 'example' | 'quiz';
  currentPage?: string;
  userBackground?: UserProfile;
}

const handleSelectionExplain = (text: string, action: string) => {
  setIsOpen(true); // Open chat

  // Auto-generate question based on action
  const prompts = {
    explain: `Explain this concept: "${text}"`,
    simplify: `Explain this in simpler terms: "${text}"`,
    example: `Give me a practical example of: "${text}"`,
    quiz: `Quiz me on this concept: "${text}"`
  };

  setInputValue(prompts[action]);
  // Auto-submit or let user modify
};
```

**3. Integration Point**

```tsx
// Root.tsx or DocItem wrapper
<SelectionMenu onExplain={handleExplain} />
<ChatWidget />
```

#### Backend Changes

**1. Add Context-Aware Query Endpoint**

```python
# backend/app/routes/rag.py additions

class ContextualQueryRequest(BaseModel):
    question: str
    selected_text: Optional[str] = None  # Text user selected
    action: Optional[str] = None  # explain, simplify, example, quiz
    current_section: Optional[str] = None  # Which section they're reading
    user_background: Optional[dict] = None  # From user profile
    conversation_history: Optional[List[dict]] = None  # Previous Q&A

@router.post("/query/contextual")
async def contextual_query(request: ContextualQueryRequest):
    """Enhanced query with selection context"""
    rag_service = get_rag_service()

    # Build enhanced prompt based on action
    enhanced_prompt = build_contextual_prompt(
        question=request.question,
        selected_text=request.selected_text,
        action=request.action,
        user_background=request.user_background
    )

    result = rag_service.query(
        question=enhanced_prompt,
        num_results=5
    )

    return result
```

**2. Enhanced Prompt Engineering**

```python
# backend/app/services/rag.py additions

def build_contextual_prompt(
    question: str,
    selected_text: str = None,
    action: str = None,
    user_background: dict = None
) -> str:
    """Build context-aware prompts"""

    prompt_templates = {
        'explain': """
            The user is reading about: "{selected_text}"

            They want a clear explanation. Consider their background:
            - Software: {software_level}
            - ROS2: {ros2_level}
            - Robotics: {robotics_level}

            Question: {question}

            Provide a clear, level-appropriate explanation.
        """,

        'simplify': """
            The user found this confusing: "{selected_text}"

            Simplify this concept using:
            - Everyday analogies
            - Simple language
            - Step-by-step breakdown

            Question: {question}
        """,

        'example': """
            Concept: "{selected_text}"

            Provide:
            1. A concrete, real-world example
            2. Code snippet if applicable
            3. Expected output/behavior

            Question: {question}
        """,

        'quiz': """
            Topic: "{selected_text}"

            Create a brief quiz to test understanding:
            1. Ask 2-3 questions (mix of easy and medium)
            2. Wait for user's answers
            3. Provide feedback and explanations

            Question: {question}
        """
    }

    template = prompt_templates.get(action, "{question}")

    return template.format(
        selected_text=selected_text or "",
        question=question,
        software_level=user_background.get('software_background', 'beginner'),
        ros2_level=user_background.get('ros2_experience', 'none'),
        robotics_level=user_background.get('robotics_experience', 'beginner')
    )
```

**Deliverables:**
- ✅ Text selection menu component
- ✅ Context-aware query endpoint
- ✅ Enhanced prompt templates
- ✅ Integration with ChatWidget

**Estimated Time:** 2-3 days

---

## Phase 2: Conversation Modes & Personalization (3-4 days)

### Feature: Adjustable Response Styles

**User Experience:**
```
┌─────────────────────────────────────┐
│ AI Learning Assistant      [⚙️ Mode]│
│                                     │
│ Response Style:                     │
│ ○ Quick (concise, bullet points)   │
│ ● Detailed (comprehensive)         │
│ ○ Tutorial (step-by-step)          │
│ ○ Socratic (guided questions)      │
│                                     │
│ Explanation Depth:                  │
│ [Beginner]━━━●━━[Expert]           │
└─────────────────────────────────────┘
```

#### Frontend Implementation

**1. Settings Panel Component**

```tsx
// ChatSettings.tsx
interface ChatSettings {
  responseMode: 'quick' | 'detailed' | 'tutorial' | 'socratic';
  explanationDepth: number; // 1-5 scale
  includeCodeExamples: boolean;
  includeVisuals: boolean;
  languageStyle: 'formal' | 'casual' | 'technical';
}

export default function ChatSettingsPanel({
  settings,
  onUpdate
}: ChatSettingsProps) {
  return (
    <div className={styles.settingsPanel}>
      <h3>Conversation Settings</h3>

      <div className={styles.settingGroup}>
        <label>Response Style</label>
        <select value={settings.responseMode} onChange={...}>
          <option value="quick">⚡ Quick (1-2 paragraphs)</option>
          <option value="detailed">📚 Detailed (comprehensive)</option>
          <option value="tutorial">🎓 Tutorial (step-by-step)</option>
          <option value="socratic">💭 Socratic (guided learning)</option>
        </select>
      </div>

      <div className={styles.settingGroup}>
        <label>Explanation Depth</label>
        <input
          type="range"
          min="1"
          max="5"
          value={settings.explanationDepth}
          onChange={...}
        />
        <span>{depthLabels[settings.explanationDepth]}</span>
      </div>

      <div className={styles.settingGroup}>
        <label>
          <input type="checkbox" checked={settings.includeCodeExamples} />
          Include code examples
        </label>
      </div>
    </div>
  );
}
```

**2. Update ChatWidget State**

```tsx
// ChatWidget.tsx
const [settings, setSettings] = useState<ChatSettings>({
  responseMode: 'detailed',
  explanationDepth: 3,
  includeCodeExamples: true,
  includeVisuals: true,
  languageStyle: 'casual'
});

// Load settings from localStorage or user profile
useEffect(() => {
  const savedSettings = localStorage.getItem('chatSettings');
  if (savedSettings) {
    setSettings(JSON.parse(savedSettings));
  }
}, []);

// Send settings with every query
const handleSubmit = async (e: React.FormEvent) => {
  e.preventDefault();

  const response = await fetch(`${API_URL}/query/personalized`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      question: inputValue,
      settings: settings,  // Include settings
      user_background: userProfile
    })
  });
};
```

#### Backend Implementation

**1. Personalized Response Generation**

```python
# backend/app/services/rag.py

class ResponseStyler:
    """Adjust AI responses based on user preferences"""

    def __init__(self, settings: dict):
        self.mode = settings.get('responseMode', 'detailed')
        self.depth = settings.get('explanationDepth', 3)
        self.include_code = settings.get('includeCodeExamples', True)
        self.style = settings.get('languageStyle', 'casual')

    def get_system_prompt(self) -> str:
        """Generate system prompt based on settings"""

        base_prompt = """You are an expert robotics tutor helping students
        learn Physical AI and ROS 2."""

        mode_instructions = {
            'quick': """
                Keep responses CONCISE (2-3 short paragraphs max).
                Use bullet points for lists.
                Get straight to the point.
                Provide quick, actionable answers.
            """,

            'detailed': """
                Provide COMPREHENSIVE explanations.
                Include background context and reasoning.
                Use examples and analogies.
                Cover edge cases and common pitfalls.
            """,

            'tutorial': """
                Use STEP-BY-STEP format:
                1. Clear numbered steps
                2. Explain WHY each step matters
                3. Show expected outcomes
                4. Provide "check your understanding" questions
            """,

            'socratic': """
                Use SOCRATIC METHOD:
                - Ask guiding questions instead of direct answers
                - Help the student discover the answer
                - Build on their existing knowledge
                - Provide hints, not solutions
            """
        }

        depth_instructions = {
            1: "Assume NO prior knowledge. Use everyday language and simple analogies.",
            2: "Assume BASIC programming knowledge. Explain technical terms when used.",
            3: "Assume INTERMEDIATE understanding. Balance theory and practice.",
            4: "Assume ADVANCED knowledge. Use technical language freely.",
            5: "EXPERT level. Dive into implementation details and advanced concepts."
        }

        style_instructions = {
            'casual': "Use friendly, conversational tone. It's okay to use 'you' and emojis occasionally.",
            'formal': "Use professional, academic tone. Avoid slang or casual language.",
            'technical': "Focus on precise technical terminology. Cite documentation when relevant."
        }

        full_prompt = f"""
        {base_prompt}

        Response Mode: {mode_instructions[self.mode]}

        Explanation Depth: {depth_instructions[self.depth]}

        Language Style: {style_instructions[self.style]}

        {"Include code examples when relevant." if self.include_code else "Minimize code examples."}
        """

        return full_prompt
```

**2. Update RAG Service**

```python
# backend/app/services/rag.py

def generate_personalized_response(
    self,
    query: str,
    context_chunks: List[Dict],
    settings: dict,
    user_background: dict = None
) -> Dict[str, Any]:
    """Generate response adapted to user preferences"""

    styler = ResponseStyler(settings)
    system_prompt = styler.get_system_prompt()

    # Build context
    context = "\n\n".join([
        f"[Source: {chunk['section']}]\n{chunk['content']}"
        for chunk in context_chunks
    ])

    # Build user prompt
    user_prompt = f"""
    Context from Physical AI textbook:
    {context}

    Student's Background:
    - Software Experience: {user_background.get('software_background', 'beginner')}
    - ROS 2 Experience: {user_background.get('ros2_experience', 'none')}
    - Robotics Experience: {user_background.get('robotics_experience', 'beginner')}

    Student's Question: {query}

    Respond according to the instructions in your system prompt.
    """

    response = self.openai_client.chat.completions.create(
        model=self.chat_model,
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt}
        ],
        temperature=0.7,
        max_tokens=2000  # Increase for detailed mode
    )

    return {
        "answer": response.choices[0].message.content,
        "sources": context_chunks,
        "citations": list(set([c['section'] for c in context_chunks])),
        "model": self.chat_model,
        "settings_used": settings  # Return what settings were applied
    }
```

**Deliverables:**
- ✅ Settings panel UI
- ✅ Response mode system prompts
- ✅ Personalized query endpoint
- ✅ Settings persistence (localStorage + user profile)

**Estimated Time:** 3-4 days

---

## Phase 3: Context Awareness & Conversation Memory (4-5 days)

### Feature: Remember Conversation History & Current Location

**User Experience:**
- Bot remembers previous questions in the session
- Can reference earlier explanations ("as I mentioned before...")
- Knows which chapter/page user is currently reading
- Can follow multi-turn conversations naturally

#### Architecture Changes

**1. Conversation State Management**

```tsx
// frontend/src/lib/ConversationManager.ts

interface ConversationTurn {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  context: {
    currentSection: string;
    selectedText?: string;
    action?: string;
  };
  sources?: Source[];
}

class ConversationManager {
  private turns: ConversationTurn[] = [];
  private sessionId: string;

  constructor() {
    this.sessionId = this.generateSessionId();
    this.loadFromStorage();
  }

  addTurn(turn: Omit<ConversationTurn, 'id' | 'timestamp'>) {
    const fullTurn: ConversationTurn = {
      ...turn,
      id: crypto.randomUUID(),
      timestamp: new Date()
    };

    this.turns.push(fullTurn);
    this.saveToStorage();

    // Limit history to last 10 turns to avoid context overflow
    if (this.turns.length > 10) {
      this.turns = this.turns.slice(-10);
    }
  }

  getHistory(): ConversationTurn[] {
    return this.turns;
  }

  getRecentContext(count: number = 3): ConversationTurn[] {
    return this.turns.slice(-count);
  }

  clearHistory() {
    this.turns = [];
    this.saveToStorage();
  }

  private saveToStorage() {
    localStorage.setItem(
      `conversation_${this.sessionId}`,
      JSON.stringify(this.turns)
    );
  }

  private loadFromStorage() {
    const stored = localStorage.getItem(`conversation_${this.sessionId}`);
    if (stored) {
      this.turns = JSON.parse(stored);
    }
  }
}

export const conversationManager = new ConversationManager();
```

**2. Current Page Tracker**

```tsx
// frontend/src/components/PageTracker.tsx
import { useLocation } from '@docusaurus/router';

export function usePageContext() {
  const location = useLocation();

  const getCurrentSection = () => {
    const path = location.pathname;

    // Extract module, chapter, section from URL
    // e.g., /docs/module-01-ros2/ros2-fundamentals/1.1.1-architecture
    const match = path.match(/module-(\d+)-(.+?)\/(.+?)\/(\d+\.\d+\.\d+)-(.+)/);

    if (match) {
      return {
        module: `module-${match[1]}-${match[2]}`,
        chapter: match[3],
        section: match[4],
        sectionTitle: match[5].replace(/-/g, ' '),
        fullPath: path
      };
    }

    return null;
  };

  return {
    currentSection: getCurrentSection(),
    currentPath: location.pathname
  };
}
```

**3. Update ChatWidget to Use Context**

```tsx
// ChatWidget.tsx with context awareness

export default function ChatWidget() {
  const { currentSection } = usePageContext();
  const [conversationManager] = useState(() => new ConversationManager());

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    // Add user message to history
    conversationManager.addTurn({
      role: 'user',
      content: inputValue,
      context: {
        currentSection: currentSection?.section || 'unknown',
        selectedText: selectedText
      }
    });

    // Get recent conversation history
    const recentHistory = conversationManager.getRecentContext(3);

    // Send to backend with history
    const response = await fetch(`${API_URL}/query/conversational`, {
      method: "POST",
      body: JSON.stringify({
        question: inputValue,
        conversation_history: recentHistory.map(t => ({
          role: t.role,
          content: t.content
        })),
        current_section: currentSection,
        settings: settings,
        user_background: userProfile
      })
    });

    const data = await response.json();

    // Add assistant response to history
    conversationManager.addTurn({
      role: 'assistant',
      content: data.answer,
      context: {
        currentSection: currentSection?.section || 'unknown'
      },
      sources: data.sources
    });

    setMessages(prev => [...prev, assistantMessage]);
  };

  // Add "Clear History" button
  const handleClearHistory = () => {
    conversationManager.clearHistory();
    setMessages([{
      type: "assistant",
      content: "Hi! I'm your Physical AI learning assistant. Ask me anything!",
      timestamp: new Date()
    }]);
  };
}
```

#### Backend Implementation

**1. Conversational Query Endpoint**

```python
# backend/app/routes/rag.py

class ConversationalQueryRequest(BaseModel):
    question: str
    conversation_history: List[dict] = []  # Previous Q&A pairs
    current_section: Optional[dict] = None
    settings: dict = {}
    user_background: Optional[dict] = None

@router.post("/query/conversational")
async def conversational_query(request: ConversationalQueryRequest):
    """Query with conversation context"""
    rag_service = get_rag_service()

    result = rag_service.conversational_query(
        question=request.question,
        history=request.conversation_history,
        current_section=request.current_section,
        settings=request.settings,
        user_background=request.user_background
    )

    return result
```

**2. Enhanced RAG with Memory**

```python
# backend/app/services/rag.py

def conversational_query(
    self,
    question: str,
    history: List[dict],
    current_section: dict = None,
    settings: dict = {},
    user_background: dict = None
) -> Dict[str, Any]:
    """RAG query with conversation context"""

    # Step 1: Rewrite query if it references previous context
    contextualized_query = self.rewrite_query_with_context(
        question,
        history,
        current_section
    )

    # Step 2: Search for relevant chunks
    relevant_chunks = self.search_similar_chunks(
        query=contextualized_query,
        limit=5
    )

    # Step 3: Generate response with full context
    response = self.generate_conversational_response(
        question=question,
        context_chunks=relevant_chunks,
        conversation_history=history,
        current_section=current_section,
        settings=settings,
        user_background=user_background
    )

    return response

def rewrite_query_with_context(
    self,
    question: str,
    history: List[dict],
    current_section: dict
) -> str:
    """Rewrite queries that reference previous context"""

    # If question uses pronouns or references previous messages
    if any(word in question.lower() for word in ['it', 'this', 'that', 'more', 'previous', 'earlier']):

        # Use LLM to rewrite with context
        rewrite_prompt = f"""
        Given this conversation history:
        {self._format_history(history)}

        The user is currently reading: {current_section.get('sectionTitle', 'Unknown')}

        The user just asked: "{question}"

        Rewrite this question to be self-contained, replacing pronouns and
        references with specific concepts from the conversation history.

        Return only the rewritten question.
        """

        response = self.openai_client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": "You rewrite ambiguous questions to be self-contained."},
                {"role": "user", "content": rewrite_prompt}
            ],
            temperature=0.3,
            max_tokens=100
        )

        return response.choices[0].message.content.strip()

    return question

def generate_conversational_response(
    self,
    question: str,
    context_chunks: List[Dict],
    conversation_history: List[dict],
    current_section: dict,
    settings: dict,
    user_background: dict
) -> Dict[str, Any]:
    """Generate response with conversation memory"""

    # Build context from textbook
    textbook_context = "\n\n".join([
        f"[Source: {chunk['section']}]\n{chunk['content']}"
        for chunk in context_chunks
    ])

    # Format conversation history
    history_text = self._format_history(conversation_history)

    # Build system prompt
    styler = ResponseStyler(settings)
    system_prompt = f"""
    {styler.get_system_prompt()}

    IMPORTANT: You have conversation memory. Reference previous messages when relevant.
    Use phrases like "As I mentioned earlier..." or "Building on what we discussed..."

    The student is currently reading: Section {current_section.get('section', 'N/A')} -
    {current_section.get('sectionTitle', 'N/A')}
    """

    # Build user prompt with all context
    user_prompt = f"""
    **Conversation History:**
    {history_text}

    **Current Context:**
    The student is reading: {current_section.get('sectionTitle', 'the textbook')}

    **Relevant Content from Textbook:**
    {textbook_context}

    **Student's Current Question:** {question}

    Respond naturally, referencing previous conversation when relevant.
    """

    # Generate response
    messages = [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": user_prompt}
    ]

    # Include recent conversation in message history for better coherence
    for turn in conversation_history[-3:]:  # Last 3 turns
        messages.insert(-1, {
            "role": turn['role'],
            "content": turn['content']
        })

    response = self.openai_client.chat.completions.create(
        model=self.chat_model,
        messages=messages,
        temperature=0.7,
        max_tokens=2000
    )

    return {
        "answer": response.choices[0].message.content,
        "sources": context_chunks,
        "citations": list(set([c['section'] for c in context_chunks])),
        "model": self.chat_model,
        "conversation_aware": True
    }

def _format_history(self, history: List[dict]) -> str:
    """Format conversation history for prompt"""
    if not history:
        return "(No previous conversation)"

    formatted = []
    for turn in history[-5:]:  # Last 5 turns max
        role = "Student" if turn['role'] == 'user' else "Assistant"
        formatted.append(f"{role}: {turn['content']}")

    return "\n\n".join(formatted)
```

**Deliverables:**
- ✅ Conversation manager with persistence
- ✅ Page context tracker
- ✅ Query rewriting with context
- ✅ Conversational response generation
- ✅ Clear history feature

**Estimated Time:** 4-5 days

---

## Phase 4: Advanced Learning Features (5-7 days)

### Features

#### 4.1 Follow-up Suggestions

```tsx
// Show after each response
<div className={styles.followUpSuggestions}>
  <p>You might also want to ask:</p>
  <button onClick={() => askFollowUp("Can you show me a code example?")}>
    💻 Show me a code example
  </button>
  <button onClick={() => askFollowUp("What are common mistakes?")}>
    ⚠️ What are common mistakes?
  </button>
  <button onClick={() => askFollowUp("How does this relate to...")}>
    🔗 How does this relate to...
  </button>
</div>
```

#### 4.2 Progress Tracking

```tsx
// Track which sections user has asked about
interface LearningProgress {
  sectionsVisited: string[];
  sectionsAskedAbout: string[];
  questionsAnswered: number;
  quizzesTaken: number;
  strengthAreas: string[];  // Topics they understand well
  weaknessAreas: string[];  // Topics needing more practice
}

// Backend endpoint to analyze progress
@router.get("/user/progress")
async def get_learning_progress(user_id: str):
    """Analyze user's learning journey"""
    # Query conversation history
    # Identify patterns
    # Suggest next topics
    pass
```

#### 4.3 Adaptive Quiz Generation

```python
# Generate personalized quizzes based on:
# - Current topic
# - User's background level
# - Previous quiz performance

@router.post("/quiz/generate")
async def generate_adaptive_quiz(request: QuizRequest):
    """
    Generate quiz questions tailored to user's level

    Easy → Medium → Hard based on performance
    """
    pass
```

#### 4.4 Code Execution Sandbox (Optional - Advanced)

```tsx
// Allow users to run ROS 2 code snippets
<CodeExecutor
  code={codeFromAI}
  language="python"
  onRun={executeInSandbox}
/>
```

**Deliverables:**
- ✅ Follow-up suggestion system
- ✅ Progress tracking database schema
- ✅ Adaptive quiz generator
- ✅ Learning path recommendations

**Estimated Time:** 5-7 days

---

## Phase 5: Multi-modal Interaction (Optional - 3-4 days)

### Features

#### 5.1 Voice Input/Output

```tsx
// Add voice capabilities
import { useSpeechRecognition, useSpeechSynthesis } from 'react-speech-kit';

const VoiceControls = () => {
  const { listen, stop, listening } = useSpeechRecognition({
    onResult: (result) => {
      setInputValue(result);
      handleSubmit();
    }
  });

  const { speak } = useSpeechSynthesis();

  return (
    <div>
      <button onClick={listening ? stop : listen}>
        {listening ? '🎤 Listening...' : '🎤 Voice Input'}
      </button>
      <button onClick={() => speak({ text: lastResponse })}>
        🔊 Read Aloud
      </button>
    </div>
  );
};
```

#### 5.2 Diagram Generation

```python
# Generate Mermaid diagrams based on explanations
# Example: "Show me a diagram of the ROS 2 pub/sub architecture"

@router.post("/visualize")
async def generate_diagram(request: DiagramRequest):
    """Generate Mermaid diagram from concept"""

    prompt = f"""
    Create a Mermaid diagram for: {request.concept}

    Return only valid Mermaid syntax.
    """

    # Use GPT-4 to generate diagram code
    # Return for frontend to render
    pass
```

#### 5.3 Smart Highlighting

```tsx
// Highlight mentioned concepts in the current page
// When AI mentions "pub/sub", highlight it in the docs

useEffect(() => {
  if (lastResponse) {
    const mentionedConcepts = extractConcepts(lastResponse);
    highlightOnPage(mentionedConcepts);
  }
}, [lastResponse]);
```

**Deliverables:**
- ✅ Voice input/output integration
- ✅ Dynamic diagram generation
- ✅ Concept highlighting in docs

**Estimated Time:** 3-4 days

---

## 🏗️ Implementation Roadmap

### Sprint 1 (Week 1): Foundation
- **Days 1-3:** Phase 1 - Text selection & context menu
- **Days 4-5:** Initial testing & refinement

### Sprint 2 (Week 2): Personalization
- **Days 1-4:** Phase 2 - Conversation modes & settings
- **Day 5:** Integration testing

### Sprint 3 (Week 3): Intelligence
- **Days 1-5:** Phase 3 - Context awareness & memory

### Sprint 4 (Week 4): Advanced Features
- **Days 1-5:** Phase 4 - Progress tracking, quizzes, follow-ups

### Sprint 5 (Week 5): Polish & Optional Features
- **Days 1-3:** Phase 5 - Multi-modal (if desired)
- **Days 4-5:** End-to-end testing, bug fixes, documentation

**Total Estimated Time:** 4-5 weeks (working full-time)

---

## 📊 Database Schema Updates

### New Tables Needed

```sql
-- Store conversation history
CREATE TABLE conversations (
    id UUID PRIMARY KEY,
    user_id UUID REFERENCES users(id),
    session_id VARCHAR(255),
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);

CREATE TABLE conversation_turns (
    id UUID PRIMARY KEY,
    conversation_id UUID REFERENCES conversations(id),
    role VARCHAR(20), -- 'user' or 'assistant'
    content TEXT,
    context JSONB, -- Store current_section, selected_text, etc.
    sources JSONB, -- Store source references
    timestamp TIMESTAMP DEFAULT NOW()
);

-- Track learning progress
CREATE TABLE learning_progress (
    id UUID PRIMARY KEY,
    user_id UUID REFERENCES users(id),
    section VARCHAR(50),
    questions_asked INTEGER DEFAULT 0,
    quizzes_completed INTEGER DEFAULT 0,
    understanding_level INTEGER, -- 1-5 scale
    last_visited TIMESTAMP,
    created_at TIMESTAMP DEFAULT NOW()
);

-- Store user preferences
ALTER TABLE users ADD COLUMN chat_settings JSONB DEFAULT '{
    "responseMode": "detailed",
    "explanationDepth": 3,
    "includeCodeExamples": true,
    "languageStyle": "casual"
}'::jsonb;
```

---

## 💰 Cost Estimate

### OpenAI API Usage

**Current (Basic RAG):**
- ~1-2k tokens per query
- Cost: ~$0.001 per query

**After Enhancements:**
- Context-aware queries: ~3-4k tokens (history + context)
- Conversational: ~5-6k tokens (multi-turn)
- Cost: ~$0.003-0.005 per query

**Monthly estimates:**
- 1000 queries/month: ~$3-5
- 10,000 queries/month: ~$30-50

**Mitigation strategies:**
- Cache common questions
- Limit conversation history to last 5 turns
- Use GPT-4o-mini (cheaper) for most queries
- Use GPT-4 only for complex questions

---

## 🎨 UI/UX Mockups

### Enhanced Chat Interface

```
┌─────────────────────────────────────────────────┐
│ 🤖 AI Learning Partner        [⚙️] [📊] [🔍] [✕]│
├─────────────────────────────────────────────────┤
│                                                 │
│ Currently reading: 1.1.1 ROS 2 Architecture    │
│ Mode: Detailed · Level: Intermediate           │
│                                                 │
│ ┌──────────────────────────────────────────┐   │
│ │ 👤 You (2 min ago)                       │   │
│ │ What is a ROS 2 node?                    │   │
│ └──────────────────────────────────────────┘   │
│                                                 │
│ ┌──────────────────────────────────────────┐   │
│ │ 🤖 AI Tutor (2 min ago)                  │   │
│ │                                           │   │
│ │ A ROS 2 node is like a specialized       │   │
│ │ program that does one specific job...    │   │
│ │                                           │   │
│ │ [💻 See code example] [🔗 Related: Topics]│   │
│ │                                           │   │
│ │ 📚 Sources: 1.1.1, 1.1.2                 │   │
│ └──────────────────────────────────────────┘   │
│                                                 │
│ ┌──────────────────────────────────────────┐   │
│ │ 💡 You might also want to ask:           │   │
│ │ • How do nodes communicate?               │   │
│ │ • Show me a simple node example           │   │
│ │ • What's the difference from ROS 1?       │   │
│ └──────────────────────────────────────────┘   │
│                                                 │
├─────────────────────────────────────────────────┤
│ 📝 Ask about ROS 2, URDF, sensors...      [🎤] │
│                                            [📤] │
└─────────────────────────────────────────────────┘

Settings Panel:
┌─────────────────────────────────────┐
│ ⚙️ Chat Settings                    │
├─────────────────────────────────────┤
│ Response Style:                     │
│ ○ Quick  ● Detailed  ○ Tutorial    │
│                                     │
│ Explanation Depth:                  │
│ [Beginner]━━━●━━━[Expert]          │
│                                     │
│ ☑️ Include code examples            │
│ ☑️ Show follow-up suggestions       │
│ ☑️ Remember conversation            │
│                                     │
│ [Clear History] [Export Chat]      │
└─────────────────────────────────────┘
```

---

## ✅ Success Metrics

### Quantitative
- **Engagement:** 3x more messages per session
- **Time on site:** 50% increase
- **Return rate:** Users return 2-3x more often
- **Question quality:** More complex, follow-up questions

### Qualitative
- Users describe chatbot as "helpful tutor" not just "search"
- Positive feedback on adaptive explanations
- Users complete more chapters
- Reduced confusion/frustration

---

## 🚀 Quick Start Guide

### Recommended Implementation Order (MVP)

**Week 1-2: Core Enhancement (Highest ROI)**
1. ✅ Text selection menu (Phase 1)
2. ✅ Conversation modes - Quick/Detailed (Phase 2 - partial)
3. ✅ Basic conversation history (Phase 3 - partial)

**Week 3-4: Intelligence Layer**
4. ✅ Full context awareness (Phase 3)
5. ✅ Follow-up suggestions (Phase 4 - partial)
6. ✅ Settings persistence

**Week 5+: Advanced (Nice-to-have)**
7. Progress tracking
8. Adaptive quizzes
9. Voice features

---

## 🔧 Technical Stack Additions

### Frontend
```json
{
  "dependencies": {
    "react-speech-kit": "^3.0.1",  // Voice features
    "react-markdown": "^8.0.0",     // Better message rendering
    "rehype-highlight": "^6.0.0",   // Code syntax highlighting
    "mermaid": "^10.0.0"            // Diagram rendering
  }
}
```

### Backend
```txt
# requirements.txt additions
langchain==0.1.0              # For conversation memory
tiktoken==0.5.2               # Token counting
anthropic==0.8.0              # Optional: Claude for longer context
```

---

## 📝 Next Steps

**To begin implementation:**

1. **Review and approve this plan**
2. **Prioritize features** (recommend starting with Phases 1-3)
3. **Set up development environment**
4. **Create feature branches**
5. **Start with Phase 1: Text selection menu** (highest user impact, lowest complexity)

**Would you like me to:**
- Start implementing Phase 1 (text selection)?
- Create detailed component specs for a specific phase?
- Set up the database schema for conversation storage?
- Design the UI components in detail?

Let me know which part you'd like to tackle first! 🚀
