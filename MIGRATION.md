# Migration Guide: docs/ → frontend/

**Date**: 2025-12-07  
**Reason**: Clarify project structure - `docs/` contains the entire Docusaurus application, not just documentation

## Problem Statement

The original structure was confusing:

- `docs/` folder name suggests it's only documentation
- But `docs/` actually contains the **entire Docusaurus frontend application** (package.json, src/, static/, node_modules/)
- Meanwhile, `backend/` folder clearly indicates the FastAPI backend
- This asymmetry creates confusion for contributors and judges

## Solution

Rename `docs/` to `frontend/` for clarity:

```
Before:                          After:
├── docs/                        ├── frontend/
│   ├── package.json             │   ├── package.json
│   ├── src/                     │   ├── src/
│   ├── static/                  │   ├── static/
│   ├── docs/                    │   ├── docs/          # Actual markdown chapters
│   └── docusaurus.config.ts     │   └── docusaurus.config.ts
├── backend/                     ├── backend/
└── ...                          └── ...
```

## Migration Steps

### Step 1: Stop Running Processes

```bash
# Stop the Docusaurus dev server if running
# Press Ctrl+C in the terminal where npm run start is running
```

### Step 2: Rename Directory

```bash
# Windows PowerShell
Move-Item -Path "docs" -Destination "frontend"

# macOS/Linux
mv docs frontend
```

### Step 3: Files Already Updated

The following files have been updated automatically:

- ✅ `package.json` (scripts now use `cd frontend`)
- ✅ `Dockerfile.frontend` (COPY paths updated)
- ✅ `docker-compose.yml` (volume mounts updated)
- ✅ `README.md` (installation instructions updated)

### Step 4: Restart Development

```bash
# New command
npm run dev

# Or directly
cd frontend
npm run start
```

## What Changed

### Root `package.json`

**Before:**

```json
"scripts": {
  "dev": "cd docs && npm run start",
  "build": "cd docs && npm run build"
}
```

**After:**

```json
"scripts": {
  "dev": "cd frontend && npm run start",
  "build": "cd frontend && npm run build"
}
```

### `Dockerfile.frontend`

**Before:**

```dockerfile
COPY docs/package.json docs/package-lock.json ./
COPY docs/ ./
```

**After:**

```dockerfile
COPY frontend/package.json frontend/package-lock.json ./
COPY frontend/ ./
```

### `docker-compose.yml`

**Before:**

```yaml
volumes:
  - ./docs:/app/docs:ro
```

**After:**

```yaml
volumes:
  - ./frontend:/app/frontend:ro
```

## Verification Checklist

After renaming, verify:

- [ ] `frontend/` directory exists (not `docs/`)
- [ ] `npm run dev` starts Docusaurus correctly
- [ ] `npm run build` builds without errors
- [ ] Docker build works: `docker-compose build frontend`
- [ ] All paths in error messages reference `frontend/` not `docs/`

## Rollback (if needed)

If you need to rollback for any reason:

```bash
# Windows
Move-Item -Path "frontend" -Destination "docs"

# macOS/Linux
mv frontend docs
```

Then revert the config files using git:

```bash
git checkout package.json Dockerfile.frontend docker-compose.yml README.md
```

## Benefits of This Change

1. **Clarity**: `frontend/` and `backend/` are symmetrical and self-explanatory
2. **Monorepo Standard**: Aligns with modern full-stack project conventions
3. **Easier Onboarding**: New contributors immediately understand the structure
4. **Better Separation**: Clear boundary between frontend app and content (frontend/docs/)
5. **Hackathon Presentation**: Judges instantly recognize the architecture

## Final Structure

```
physical-ai-humanoids-textbook/
├── frontend/                      # Docusaurus App
│   ├── docs/                     # ← Markdown chapters (nested clarity!)
│   ├── src/                      # ← React components
│   ├── static/                   # ← Static assets
│   ├── package.json              # ← Frontend dependencies
│   └── docusaurus.config.ts      # ← Docusaurus config
│
├── backend/                       # FastAPI App
│   ├── app/                      # ← Python modules
│   ├── tests/                    # ← Backend tests
│   ├── requirements.txt          # ← Python dependencies
│   └── .env.example              # ← Backend config
│
├── database/                      # Database schemas
├── labs/                          # Lab exercises
├── ros2_packages/                 # ROS 2 workspace
└── ...                            # Other project folders
```

## Questions?

If you encounter any issues during migration, check:

1. Is the dev server still running? (Stop it first)
2. Did you run `npm run dev` from the project root? (Not inside frontend/)
3. Are there any hardcoded `docs/` paths in custom scripts?

---

**Status**: ✅ Configuration files updated  
**Action Required**: Rename `docs/` folder to `frontend/` when dev server is stopped
