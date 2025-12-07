<!--
SYNC IMPACT REPORT
==================
Version: 0.0.0 → 1.0.0 (Initial ratification)
Date: 2025-12-07

ADDED SECTIONS:
- Core Principles (7 principles established)
- Technical Standards (Docusaurus, accessibility, performance)
- Visual & Content Standards (diagrams, code examples, tables)
- RAG Chatbot Requirements
- Development Workflow
- Quality Gates

TEMPLATES STATUS:
- ✅ spec-template.md - Reviewed
- ✅ plan-template.md - Reviewed
- ✅ tasks-template.md - Reviewed
- ⚠ Commands (to be verified against constitution principles)

DEFERRED ITEMS: None

NOTES:
This is the initial constitution establishing governance for the Physical AI & Humanoid
Robotics book project. All subsequent changes MUST follow semantic versioning rules.
-->

# Physical AI & Humanoid Robotics Book Constitution

## Core Principles

### I. 2025-Accurate Technical Depth

All technical content MUST reflect 2025 state-of-the-art and be verifiable against authoritative sources.

**Requirements:**

- Every factual claim MUST be cited with a verifiable source (academic paper, manufacturer specification, market report)
- Technical specifications (robot models, compute platforms, market data) MUST be current as of 2025
- Historical content MUST distinguish clearly between past developments and present capabilities
- All code examples MUST use current API versions and best practices

**Rationale:** Outdated or inaccurate information undermines the book's credibility and educational value. This principle ensures readers can trust and apply the material in real-world contexts.

### II. Progressive Difficulty Architecture

Content MUST follow a beginner-to-advanced learning path that builds knowledge systematically.

**Requirements:**

- Each chapter MUST assume only the knowledge established in prior chapters
- Complex concepts MUST be introduced with foundational explanations before technical details
- Prerequisites MUST be explicitly stated at chapter beginning
- Chapters MUST be structured: Foundation → Application → Advanced Topics
- Learning objectives MUST be testable and aligned with master blueprint

**Rationale:** Readers progress from basic programming/AI background to deployment-ready knowledge. Breaking this progression creates frustration and knowledge gaps.

### III. Blueprint Conformance (NON-NEGOTIABLE)

All content MUST strictly adhere to the master blueprint structure, depth, and specifications.

**Requirements:**

- Each chapter MUST contain 5-8 pages with 3-4 subchapters as specified
- Content depth MUST match blueprint section requirements (no omissions or expansions without amendment)
- Visual content MUST include minimum 1 Mermaid diagram per chapter
- Code examples, tables, and practical exercises MUST be present where blueprint specifies
- Deviations from blueprint REQUIRE constitutional amendment or explicit ADR

**Rationale:** The blueprint represents approved scope and resource allocation. Deviations create inconsistency, delay delivery, and violate reader expectations set by project plan.

### IV. Visual-First Technical Communication

Technical concepts MUST be supported by diagrams, tables, and code examples—not text alone.

**Requirements:**

- Minimum 1 Mermaid diagram per chapter (architecture, flow, timeline, or comparison)
- Comparison tables MUST contain real data (not placeholder values)
- Code examples MUST be syntax-highlighted, complete, and runnable
- Complex architectures MUST have visual representation before textual explanation
- All visuals MUST be mobile-responsive and dark-mode compatible

**Rationale:** Visual aids accelerate comprehension of complex robotics/AI concepts. Text-only explanations increase cognitive load and reduce accessibility.

### V. Implementation-Oriented Content

Theory MUST be balanced with practical implementation guidance and real-world examples.

**Requirements:**

- Each technical concept MUST include "how to implement" guidance
- Code examples MUST demonstrate actual usage patterns (not pseudo-code)
- Real-world case studies MUST cite specific deployments (BMW, Figure AI, etc.)
- Performance specifications MUST include actual measurements where available
- Trade-offs MUST be explained with concrete scenarios

**Rationale:** Readers seek actionable knowledge, not just conceptual understanding. Implementation details enable readers to apply learning to projects.

### VI. Accessibility and Performance Standards

The book platform MUST meet modern web accessibility and performance benchmarks.

**Requirements:**

- WCAG AA compliance for all interactive elements and content
- Page load time MUST be <2 seconds on 4G connections
- Mobile-responsive design MUST maintain readability on 375px viewports
- Dark mode MUST be fully supported with appropriate contrast ratios
- Images MUST have descriptive alt text
- Code blocks MUST be keyboard-navigable

**Rationale:** Accessibility is a right, not optional feature. Performance directly impacts user engagement and SEO rankings.

### VII. Source Traceability for RAG Chatbot

All content MUST be structured to enable accurate citation and retrieval by the RAG system.

**Requirements:**

- Each section MUST have unique identifiers for citation
- Chatbot responses MUST cite chapter/section sources
- Content MUST be semantically structured (headings, lists, code blocks)
- RAG indexing MUST be tested after each chapter completion
- Response time target: <3 seconds for queries
- Out-of-scope queries MUST gracefully decline with suggestions

**Rationale:** RAG chatbot is a core feature differentiator. Poor source attribution undermines trust; slow responses degrade user experience.

## Technical Standards

### Platform Architecture

**Mandatory Technology Stack:**

- Docusaurus 3.9 as the static site generator
- React 18+ for custom components
- Mermaid JS for diagram rendering
- Prism/Shiki for syntax highlighting
- Algolia DocSearch for search (with RAG fallback)

**Configuration Requirements:**

- Mobile-first responsive design
- Dark mode toggle with user preference persistence
- Fast refresh in development (<1s)
- Optimized production build (code splitting, lazy loading)
- SEO meta tags (Open Graph, Twitter Cards)

### Performance Budgets

**Hard Limits:**

- Initial page load (FCP): <1.5s
- Time to Interactive (TTI): <2.0s
- Largest Contentful Paint (LCP): <2.5s
- Cumulative Layout Shift (CLS): <0.1
- Bundle size per page: <300KB (gzipped)

**Monitoring:**

- Lighthouse CI MUST score ≥90 on all metrics
- Performance regression tests on every PR
- Core Web Vitals tracked via Google Search Console

### Security and Privacy

**Requirements:**

- HTTPS-only (enforce via HSTS headers)
- Content Security Policy (CSP) headers configured
- No third-party tracking without explicit consent
- RAG chatbot queries MUST NOT be logged with PII
- Dependencies MUST be audited monthly (npm audit, Dependabot)

## Visual & Content Standards

### Diagram Requirements

**Mermaid Diagram Types (minimum 1 per chapter):**

- Architecture diagrams (system components, data flow)
- Timeline graphics (historical evolution)
- Flowcharts (algorithms, decision trees)
- Sequence diagrams (interaction protocols)

**Quality Standards:**

- Labels MUST be concise (≤5 words per node)
- Colors MUST meet WCAG contrast ratios
- Diagrams MUST render correctly in light and dark modes
- Complex diagrams (>15 nodes) MUST be split or hierarchical

### Code Example Standards

**All Code Blocks MUST:**

- Include language identifier for syntax highlighting
- Be complete and executable (no "..." placeholders unless pedagogically necessary)
- Include inline comments for non-obvious logic
- Follow language conventions (PEP 8 for Python, ROS 2 style for C++)
- Be tested for correctness before publication

**Example Format:**

```python
# Good: Complete, runnable, commented
import numpy as np

class ComplementaryFilter:
    """Fuses IMU and vision for orientation estimation."""
    def __init__(self, alpha=0.98):
        self.alpha = alpha  # High-freq IMU vs low-freq vision ratio
        self.angle = 0.0

    def update(self, imu_rate, vision_angle, dt):
        """Update filter with new measurements."""
        imu_estimate = self.angle + imu_rate * dt
        self.angle = self.alpha * imu_estimate + (1 - self.alpha) * vision_angle
        return self.angle
```

### Table Standards

**All Comparison Tables MUST:**

- Contain real data (no "TBD" or placeholder values)
- Include source citations in caption or footnote
- Use consistent units and formatting
- Be responsive (stack on mobile if >4 columns)
- Include totals/averages where applicable

## RAG Chatbot Requirements

### Functional Requirements

**Core Capabilities:**

- Answer questions using ONLY book content (no hallucination from general knowledge)
- Support "selected text" context queries (highlight → ask → scoped response)
- Cite source chapter/section for every factual claim
- Handle multi-turn conversations with context retention
- Gracefully decline out-of-scope queries with suggestion to expand scope

**Performance Requirements:**

- Response latency: <3 seconds (p95)
- Embedding generation: <500ms per query
- Concurrent users: Support 100+ simultaneous queries
- Accuracy: ≥85% correct citations (validated via test set)

### Error Handling

**Chatbot MUST:**

- Detect ambiguous queries and request clarification
- Provide "I don't have information on that" for out-of-scope topics
- Suggest related in-scope topics when declining
- Log failed queries for content gap analysis (anonymized)
- Fallback to keyword search if vector retrieval fails

### Privacy and Safety

**Requirements:**

- No PII collection (queries anonymized before logging)
- No retention of conversation history beyond session
- Content filtering to prevent injection attacks
- Rate limiting to prevent abuse (10 queries/minute per IP)
- Clear UI indicator distinguishing RAG from general chatbot

## Development Workflow

### Chapter Development Lifecycle

**Sequential Process (MUST be followed in order):**

1. **Specification Phase:**

   - Review master blueprint section
   - Identify dependencies on prior chapters
   - List required diagrams, tables, code examples
   - Create chapter spec document

2. **Content Creation Phase:**

   - Write draft content following blueprint structure
   - Insert placeholders for diagrams/code
   - Ensure 5-8 page target length
   - Verify learning objectives coverage

3. **Technical Review Phase:**

   - Validate all technical facts with citations
   - Test all code examples
   - Generate Mermaid diagrams
   - Populate comparison tables with real data
   - Verify WCAG AA compliance

4. **Integration Phase:**

   - Index chapter for RAG system
   - Test chatbot retrieval accuracy
   - Verify mobile responsiveness
   - Check dark mode rendering
   - Run performance benchmarks

5. **Deployment Phase:**
   - Git commit with semantic message (e.g., "feat(ch3): add Edge Compute & Perception chapter")
   - Deploy to preview environment
   - Run full test suite (Lighthouse, accessibility, RAG accuracy)
   - Merge to main upon passing all gates

### Git Workflow

**Commit Conventions:**

- One commit per completed chapter (atomic, revertable)
- Commit message format: `<type>(chapter<N>): <description>`
- Types: `feat` (new chapter), `fix` (content correction), `docs` (meta changes)
- Example: `feat(chapter5): add Whole-Body Control & Locomotion`

**Branch Strategy:**

- `main` branch MUST always be deployable
- Chapter development in `chapter/<N>-<slug>` branches
- Preview deployments on PR creation
- Require passing CI before merge

### Testing Requirements

**Pre-Merge Quality Gates:**

1. All code examples execute without errors
2. Mermaid diagrams render in light/dark modes
3. RAG system retrieves chapter content with ≥80% accuracy
4. Lighthouse score ≥90 (performance, accessibility, best practices, SEO)
5. No broken links (internal or external)
6. Mobile viewport renders correctly (375px, 768px, 1024px)

## Quality Gates

### Content Quality Checklist

**Every chapter MUST pass before publication:**

- [ ] 5-8 pages length (measured in Docusaurus rendered output)
- [ ] 3-4 subchapters with clear headings
- [ ] Minimum 1 Mermaid diagram (validated: renders in both modes)
- [ ] At least 1 comparison table with real data and citations
- [ ] Minimum 1 code example (tested, complete, commented)
- [ ] Learning objectives stated and addressed
- [ ] All facts cited with verifiable sources
- [ ] Progressive difficulty maintained (builds on prior chapters)
- [ ] No unexplained jargon (or defined on first use)
- [ ] Summary points section present

### Technical Quality Checklist

**Every deployment MUST pass:**

- [ ] Lighthouse Performance score ≥90
- [ ] Lighthouse Accessibility score ≥90 (WCAG AA)
- [ ] Lighthouse Best Practices score ≥90
- [ ] Lighthouse SEO score ≥90
- [ ] Page load time <2s on 4G throttling
- [ ] No console errors or warnings
- [ ] Mobile responsiveness verified (3 breakpoints)
- [ ] Dark mode fully functional
- [ ] All images have alt text
- [ ] RAG chatbot retrieves chapter with <3s latency

### RAG System Quality Checklist

**Per-chapter validation:**

- [ ] Chapter indexed successfully (no embedding errors)
- [ ] Test queries retrieve correct chapter/section
- [ ] Citations include chapter and section numbers
- [ ] Out-of-scope queries gracefully decline
- [ ] Selected text queries work for chapter content
- [ ] Response time <3s for 95th percentile queries
- [ ] No hallucination (responses only from chapter content)

## Governance

### Amendment Process

**This constitution MUST be amended for:**

- Adding, removing, or significantly modifying core principles
- Changing technical standards (platform, performance budgets)
- Modifying development workflow or quality gates
- Updating governance rules themselves

**Amendment Procedure:**

1. Proposal documented with rationale and impact analysis
2. Review against master blueprint alignment
3. Update constitution with version increment (semantic versioning)
4. Propagate changes to dependent templates (spec, plan, tasks)
5. Create ADR for architecturally significant changes
6. Commit with message: `docs: amend constitution to v<X.Y.Z> (<summary>)`

### Version Semantics

**Version Format:** MAJOR.MINOR.PATCH

- **MAJOR:** Backward-incompatible principle removal/redefinition, governance overhaul
- **MINOR:** New principle added, section expansion, new quality gate
- **PATCH:** Clarifications, wording improvements, typo fixes

### Compliance Reviews

**Frequency:** After every 2 chapters completed (Chapters 2, 4, 6, 8, 10)

**Review Focus:**

- Verify adherence to all core principles
- Check quality gate pass rates
- Analyze RAG system performance trends
- Review user feedback (if available)
- Identify constitution gaps or ambiguities
- Propose amendments if needed

### Conflict Resolution

**Priority Order (highest to lowest):**

1. Master Blueprint (project scope and structure)
2. Constitution Core Principles (quality and standards)
3. Technical Standards (implementation details)
4. Development Workflow (process preferences)

**In case of conflict:**

- Blueprint takes precedence over constitution on scope/structure
- Constitution takes precedence over individual preferences on quality
- Unresolvable conflicts require stakeholder escalation and ADR documentation

### Enforcement

**All contributors MUST:**

- Read and acknowledge this constitution before contributing
- Verify compliance before submitting PRs
- Document deviations with explicit justification in PR description
- Participate in compliance reviews

**PR Review Checklist:**

- [ ] Constitution compliance verified
- [ ] Quality gates passed (automated CI)
- [ ] Code examples tested
- [ ] Citations present for all facts
- [ ] RAG indexing tested
- [ ] Visual standards met (diagrams, tables)

**Non-compliance handling:**

- First instance: Request revision with specific constitution reference
- Repeated instances: Require constitution review/acknowledgment
- Systemic issues: Trigger constitution amendment to clarify ambiguity

---

**Version**: 1.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07
