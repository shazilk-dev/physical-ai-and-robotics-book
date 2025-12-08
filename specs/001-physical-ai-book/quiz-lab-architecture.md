# Quiz & Lab Integration Architecture

## Visual Overview

```mermaid
graph TB
    subgraph "Content Layer"
        A[Chapter Content<br/>Markdown/MDX]
        B[Inline Quizzes<br/>React Components]
        C[Lab Callouts<br/>React Components]
    end

    subgraph "Assessment Components"
        D[MultipleChoice<br/>Component]
        E[CodeChallenge<br/>Component]
        F[ConceptCheck<br/>Component]
    end

    subgraph "Lab System"
        G[Labs Overview<br/>/docs/labs/]
        H[Lab Guides<br/>Markdown]
        I[Lab Files<br/>/labs/labXX/]
        J[Starter Code]
        K[Solutions]
        L[Tests]
    end

    subgraph "Progress Tracking"
        M[localStorage<br/>Client State]
        N[Progress Indicators<br/>Sidebar Icons]
        O[Analytics<br/>Optional Backend]
    end

    A --> B
    A --> C
    B --> D
    B --> E
    B --> F
    C --> G
    G --> H
    H --> I
    I --> J
    I --> K
    I --> L
    D --> M
    E --> M
    F --> M
    M --> N
    M --> O

    style A fill:#e1f5ff
    style B fill:#fef3c7
    style C fill:#fef3c7
    style G fill:#dbeafe
    style M fill:#fecaca
```

## Learning Flow Architecture

```mermaid
sequenceDiagram
    participant Student
    participant Chapter
    participant Quiz
    participant LabCallout
    participant LabGuide
    participant Code
    participant Tests

    Student->>Chapter: Read Section 1.1.1
    Chapter->>Quiz: Display inline quiz
    Quiz->>Student: 5 MCQ questions
    Student->>Quiz: Submit answers
    Quiz->>Student: Immediate feedback<br/>(scores, explanations)

    Student->>Chapter: Continue reading 1.1.2
    Chapter->>LabCallout: Show "Try This Lab"
    Student->>LabCallout: Click "Start Lab"
    LabCallout->>LabGuide: Navigate to lab page

    LabGuide->>Student: Show prerequisites,<br/>objectives, instructions
    Student->>Code: Copy starter files
    Student->>Code: Implement TODOs
    Student->>Tests: Run automated tests
    Tests->>Student: Pass/Fail feedback

    Student->>Chapter: Return to reading<br/>(with practical experience)
```

## Component Hierarchy

```
frontend/
├── docs/
│   ├── module-01-ros2/
│   │   ├── ros2-fundamentals/
│   │   │   ├── 1.1.1-architecture.md ──┐
│   │   │   │   (includes Quiz)         │
│   │   │   ├── 1.1.2-rclpy-patterns.md │
│   │   │   │   (includes LabCallout)───┼───► Links to Labs
│   │   │   └── ...                     │
│   │   └── ...                         │
│   │                                   │
│   └── labs/ ◄─────────────────────────┘
│       ├── overview.md (All labs index)
│       ├── lab01-ros2-basics.md
│       ├── lab02-urdf.md
│       └── rubrics.md
│
├── src/
│   └── components/
│       ├── Quiz/
│       │   ├── MultipleChoice.tsx
│       │   ├── CodeChallenge.tsx
│       │   ├── ConceptCheck.tsx
│       │   └── Quiz.module.css
│       │
│       └── LabCallout/
│           ├── LabLink.tsx
│           ├── LabCard.tsx
│           └── LabCallout.module.css
│
└── static/
    └── progress.json (optional analytics)
```

## Progressive Difficulty Curve

```mermaid
graph LR
    subgraph "Module 1: Foundation"
        A1[Quiz: Recall<br/>What is ROS 2?]
        B1[Lab: Guided<br/>Fill TODOs]
        C1[Simple Pass/Fail]
    end

    subgraph "Module 2: Application"
        A2[Quiz: Scenarios<br/>Which QoS?]
        B2[Lab: Open-ended<br/>Design choices]
        C2[Multiple solutions]
    end

    subgraph "Module 3: Analysis"
        A3[Quiz: Debug<br/>Fix this code]
        B3[Lab: Optimization<br/>Performance tuning]
        C3[Trade-offs]
    end

    subgraph "Module 4: Synthesis"
        A4[Quiz: Integration<br/>Multi-concept]
        B4[Capstone: Build system<br/>Voice-controlled robot]
        C4[Creative solutions]
    end

    A1 --> A2 --> A3 --> A4
    B1 --> B2 --> B3 --> B4
    C1 --> C2 --> C3 --> C4

    style A1 fill:#86efac
    style A2 fill:#fde047
    style A3 fill:#fdba74
    style A4 fill:#f87171
```

## Quiz Component State Machine

```mermaid
stateDiagram-v2
    [*] --> NotStarted
    NotStarted --> InProgress: User clicks question
    InProgress --> Submitted: User clicks Submit
    Submitted --> ShowingFeedback: Display result
    ShowingFeedback --> Correct: Answer correct
    ShowingFeedback --> Incorrect: Answer wrong
    Incorrect --> ShowingHint: Click "Need help?"
    ShowingHint --> InProgress: Try again
    Correct --> NextQuestion: Continue
    NextQuestion --> InProgress: Load next Q
    NextQuestion --> Complete: All done
    Complete --> [*]

    note right of Correct
        Green checkmark
        + Explanation
        + Next button
    end note

    note right of Incorrect
        Red X
        + Hint button
        + Retry button
    end note
```

## Lab Validation Pipeline

```mermaid
flowchart TD
    A[Student writes code] --> B{Syntax valid?}
    B -->|No| C[Show linting errors]
    C --> A
    B -->|Yes| D[Run automated tests]
    D --> E{All tests pass?}
    E -->|No| F[Show test failures<br/>with hints]
    F --> A
    E -->|Yes| G[Check code style]
    G --> H{Follows conventions?}
    H -->|No| I[Suggest improvements]
    I --> A
    H -->|Yes| J[✅ Lab Complete!]
    J --> K[Show solution comparison]
    K --> L[Award progress badge]

    style J fill:#86efac
    style F fill:#fca5a5
    style I fill:#fef08a
```

## Navigation UX Flow

```
Homepage
   │
   ├─► Introduction (Overview of book)
   │
   ├─► Module 1: ROS 2 Nervous System
   │    │
   │    ├─► Chapter 1.1: ROS 2 Fundamentals
   │    │    ├─► 1.1.1 Architecture ──► Quiz (inline) ──► ✅ 4/5 correct
   │    │    ├─► 1.1.2 rclpy Patterns ──► Quiz (inline) ──► ✅ 5/5 correct
   │    │    │                         └──► Lab Callout ──┐
   │    │    ├─► 1.1.3 Parameters                        │
   │    │    └─► 1.1.4 QoS Tuning                        │
   │    │                                                 │
   │    ├─► Chapter 1.2: URDF & Robot Description        │
   │    │    ├─► 1.2.1 URDF Basics                       │
   │    │    ├─► 1.2.2 Sensors ──► Lab Callout ──┐       │
   │    │    ├─► 1.2.3 Validation                │       │
   │    │    └─► 1.2.4 Testing                   │       │
   │    │                                         │       │
   │    └─► Module 1 Capstone (Project)          │       │
   │                                              │       │
   └─► 🔬 Labs & Exercises ◄────────────────────┴───────┘
        │
        ├─► Labs Overview (All labs matrix)
        ├─► Lab 1: First ROS 2 Node ◄──────────── (linked from 1.1.2)
        │    ├─► README (Instructions)
        │    ├─► Prerequisites checklist
        │    ├─► Download starter code
        │    └─► Run automated tests
        │
        ├─► Lab 2: Build URDF ◄────────────────── (linked from 1.2.2)
        │    └─► ...
        │
        └─► Lab Rubrics (Self-assessment)
```

## Progress Tracking Data Model

```typescript
// Client-side localStorage schema
interface UserProgress {
  userId?: string; // Optional if auth added later

  // Quiz progress
  quizzes: {
    [quizId: string]: {
      completed: boolean;
      score: number;
      attempts: number;
      lastAttempt: string; // ISO timestamp
      answers: number[]; // Selected options
    };
  };

  // Lab progress
  labs: {
    [labId: string]: {
      status: "not-started" | "in-progress" | "completed";
      startedAt?: string;
      completedAt?: string;
      testsPassedCount: number;
      totalTests: number;
    };
  };

  // Module progress
  modules: {
    [moduleId: string]: {
      chaptersCompleted: number;
      totalChapters: number;
      quizzesCompleted: number;
      totalQuizzes: number;
      labsCompleted: number;
      totalLabs: number;
      capstoneCompleted: boolean;
    };
  };

  // Preferences
  settings: {
    showHints: boolean;
    darkMode: boolean;
    language: "en" | "ur"; // For future translation
  };
}

// Example localStorage key
// physicalai_progress_v1
```

## Responsive Design Breakpoints

```css
/* Quiz Component Responsive Layout */

/* Desktop (>996px) */
.quiz-container {
  max-width: 800px;
  margin: 2rem auto;
  padding: 2rem;
}

.quiz-options {
  display: grid;
  grid-template-columns: 1fr 1fr; /* 2 columns */
  gap: 1rem;
}

/* Tablet (768px - 996px) */
@media (max-width: 996px) {
  .quiz-container {
    padding: 1.5rem;
  }

  .quiz-options {
    grid-template-columns: 1fr; /* 1 column */
  }
}

/* Mobile (<768px) */
@media (max-width: 768px) {
  .quiz-container {
    padding: 1rem;
    margin: 1rem 0;
  }

  .quiz-question {
    font-size: 1rem; /* Smaller text */
  }

  .quiz-button {
    width: 100%; /* Full-width buttons */
  }
}
```

## Color Coding System

```css
/* Consistent with modernized UI palette */

:root {
  /* Quiz feedback colors */
  --quiz-correct: #86efac; /* Green-300 */
  --quiz-incorrect: #fca5a5; /* Red-300 */
  --quiz-hint: #fef08a; /* Yellow-200 */
  --quiz-neutral: #cbd5e1; /* Slate-300 */

  /* Difficulty badges */
  --difficulty-beginner: #86efac; /* Green */
  --difficulty-intermediate: #fbbf24; /* Amber */
  --difficulty-advanced: #f87171; /* Red */

  /* Lab status indicators */
  --status-not-started: #cbd5e1; /* Slate */
  --status-in-progress: #60a5fa; /* Blue */
  --status-completed: #34d399; /* Emerald */
}
```

## API Endpoints (Optional Future Backend)

```typescript
// If authentication is added later

// Progress tracking
POST   /api/progress/quiz
GET    /api/progress/user/:userId
PUT    /api/progress/lab/:labId

// Analytics (instructor dashboard)
GET    /api/analytics/quiz/:quizId
GET    /api/analytics/lab/:labId
GET    /api/analytics/module/:moduleId

// Leaderboard (if gamification added)
GET    /api/leaderboard/global
GET    /api/leaderboard/cohort/:cohortId

// Example request/response
POST /api/progress/quiz
{
  "quizId": "1.1.1",
  "score": 80,
  "answers": [1, 2, 0, 3, 1], // Selected options
  "duration": 120 // seconds
}

Response 200:
{
  "success": true,
  "progress": {
    "totalQuizzesTaken": 5,
    "averageScore": 85,
    "moduleCompletion": 40
  }
}
```

## Accessibility Checklist

```markdown
## Quiz Components

- [ ] ARIA labels on all buttons (`aria-label="Submit answer"`)
- [ ] Keyboard navigation (Tab, Enter, Space)
- [ ] Screen reader announcements for feedback
- [ ] Focus indicators visible (outline: 2px solid blue)
- [ ] Color contrast ratio > 4.5:1 (WCAG AA)
- [ ] Skip to next question keyboard shortcut

## Lab Components

- [ ] Semantic HTML (`<article>`, `<section>`, `<nav>`)
- [ ] Heading hierarchy (h1 → h2 → h3, no skipping)
- [ ] Alt text for all diagrams
- [ ] Code examples use `<pre><code>` with syntax highlighting
- [ ] External links open in new tab with warning
- [ ] Form inputs have associated labels
```

## Performance Optimization

```typescript
// Lazy load quiz components (only when visible)
import { lazy, Suspense } from "react";

const MultipleChoice = lazy(
  () => import("@site/src/components/Quiz/MultipleChoice")
);

function QuizSection() {
  return (
    <Suspense fallback={<div>Loading quiz...</div>}>
      <MultipleChoice {...props} />
    </Suspense>
  );
}

// Debounce progress saves (avoid excessive localStorage writes)
import { debounce } from "lodash";

const saveProgress = debounce((progress) => {
  localStorage.setItem("physicalai_progress", JSON.stringify(progress));
}, 500); // Save after 500ms of inactivity
```

---

**This architecture supports:**

- ✅ Scalability (easy to add new quizzes/labs)
- ✅ Maintainability (clear separation of concerns)
- ✅ Performance (lazy loading, optimized renders)
- ✅ Accessibility (WCAG AA compliant)
- ✅ Extensibility (can add backend later without refactoring)
