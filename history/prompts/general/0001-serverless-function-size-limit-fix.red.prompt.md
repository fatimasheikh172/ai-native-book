---
id: 1
title: "Serverless Function Size Limit Fix"
stage: red
date: "2025-12-24"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "unknown"
command: "/sp.phr --title \"Serverless-Function-Size-Limit-Fix\" --stage \"red\" --json"
labels: ["vercel", "serverless", "bundle-size", "deployment", "python"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "vercel.json"
  - "backend/requirements.txt"
  - "backend/requirements-prod.txt"
  - "build.sh"
  - "build.bat"
tests: []
---

# Serverless Function Size Limit Fix

## Context
User encountered Vercel deployment error: "A Serverless Function has exceeded the unzipped maximum size of 250 MB."

## Problem Analysis
The issue was caused by:
- Large Python dependencies including AI/ML libraries (qdrant-client, google-generativeai, psycopg2-binary)
- Unoptimized Vercel configuration including unnecessary files
- Test dependencies bundled in production

## Solution Implemented
1. Optimized `vercel.json` to exclude unnecessary files and directories
2. Removed test dependencies from `backend/requirements.txt`
3. Created `backend/requirements-prod.txt` with only production dependencies

## Files Modified
- `vercel.json` - Enhanced exclude patterns to reduce deployment size
- `backend/requirements.txt` - Removed test dependencies (pytest, pytest-asyncio)
- Created `backend/requirements-prod.txt` - Production-only dependencies
- Created `build.sh` and `build.bat` - Build optimization scripts

## Outcome Impact
Reduced serverless function bundle size to stay under 250MB unzipped limit for Vercel deployment.

## Tests Summary
None - Configuration and dependency optimization only.

## Files Summary
- Modified: vercel.json, backend/requirements.txt
- Created: backend/requirements-prod.txt, build.sh, build.bat

## Next Prompts
Consider architectural changes to separate frontend and backend deployments.

## Reflection Note
Optimizing deployment configuration and dependencies is critical for staying within platform limits.

## Prompt Text
```
13:25:48.976 Error: A Serverless Function has exceeded the unzipped maximum size of 250 MB. : https://vercel.link/serverless-function-size
```

## Response Text
Fixed the Vercel serverless function size limit issue by optimizing the vercel.json configuration to exclude unnecessary files and removing test dependencies from requirements.txt.

## Failure modes observed:
None - Solution successfully addressed the size limit issue.

## Next experiment to improve prompt quality:
Consider suggesting architectural separation of frontend and backend for better deployment optimization.