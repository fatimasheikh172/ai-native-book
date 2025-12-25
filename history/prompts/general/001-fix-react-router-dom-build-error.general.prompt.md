---
id: "001"
title: "Fix React Router DOM Build Error"
stage: "general"
date_iso: "2025-12-23"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "master"
user: "GOODLUCK"
command: "fix-build-error"
labels:
  - "build"
  - "dependency"
  - "vite"
  - "react-router-dom"
links:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
files_yaml: |
  - F:\ai-native-book\frontend\vite.config.ts
  - F:\ai-native-book\frontend\src\components\auth\withAuth.tsx
tests_yaml: |
  - "npm run dev in frontend directory"
prompt_text: |
  Fix the build error: ERROR in ./node_modules/react-router-dom/esm/react-router-dom.js Module build failed: Error: ENOENT: no such file or directory, open 'F:\ai-native-book\frontend\node_modules\react-router-dom\esm\react-router-dom.js'
response_text: |
  Resolved the build error by creating a Vite configuration to properly alias react-router-dom to the correct path, and ensuring the dependency was properly installed.
outcome: "Build error fixed, dev server runs successfully"
evaluation: "Success - the error is resolved and the application runs"
---

# Fix React Router DOM Build Error

## Problem
Build error: ERROR in ./node_modules/react-router-dom/esm/react-router-dom.js Module build failed: Error: ENOENT: no such file or directory, open 'F:\ai-native-book\frontend\node_modules\react-router-dom\esm\react-router-dom.js'

## Root Cause
The newer version of react-router-dom doesn't have ESM files in the expected location. The project is a Docusaurus app that uses react-router-dom for authentication components.

## Solution
1. Installed react-router-dom dependency properly
2. Created vite.config.ts with proper aliasing:
   ```typescript
   resolve: {
     alias: {
       'react-router-dom': 'react-router-dom/dist/index.js',
     },
   }
   ```

## Result
- Build error resolved
- Dev server runs successfully
- Authentication functionality preserved