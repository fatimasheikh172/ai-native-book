# Quickstart Guide: Physical AI and Human-Aided Robotics Book

**Date**: 2025-12-15
**Feature**: 1-physical-ai-book

## Overview

This guide provides instructions for setting up and running the Physical AI and Human-Aided Robotics educational platform. The system is built using Docusaurus for documentation and React for interactive components, with Node.js/Express for the backend API.

## Prerequisites

- Node.js (v18 or higher)
- npm or yarn package manager
- Git
- GitHub account for deployment

## Local Development Setup

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Install Dependencies

```bash
# Install frontend dependencies
cd website
npm install

# Install backend dependencies (if separate)
cd ../api
npm install
```

### 3. Environment Configuration

Create `.env` files in both the website and api directories:

**website/.env:**
```env
# Docusaurus Configuration
BASE_URL=/
DEPLOYMENT_BRANCH=gh-pages

# API Configuration
API_BASE_URL=http://localhost:3001/api
```

**api/.env:**
```env
# Server Configuration
PORT=3001
NODE_ENV=development

# Database Configuration
DB_CONNECTION_STRING=postgresql://username:password@localhost/physical_ai_book

# Authentication Configuration
AUTH_SECRET=your-secret-key-here
NEXTAUTH_URL=http://localhost:3000

# OpenAI Configuration (for Whisper integration)
OPENAI_API_KEY=your-openai-api-key

# ROS/NVIDIA Integration (placeholder values)
ROS_MASTER_URI=http://localhost:11311
NVIDIA_ISAAC_PATH=/path/to/nvidia/isaac
```

### 4. Run the Development Servers

**Start the backend API:**
```bash
cd api
npm run dev
```

**Start the Docusaurus frontend:**
```bash
cd website
npm start
```

The application will be available at `http://localhost:3000`.

## Project Structure

```
website/                    # Docusaurus documentation site
├── docs/                 # Curriculum content (organized by modules)
│   ├── module-1-nervous-system/
│   ├── module-2-digital-twin/
│   ├── module-3-ai-brain/
│   └── module-4-vla/
├── src/
│   ├── components/       # Interactive components (assessments, diagrams)
│   ├── pages/            # Custom pages
│   ├── css/              # Custom styles
│   └── theme/            # Custom theme components
├── static/               # Static assets
├── docusaurus.config.js  # Docusaurus configuration
├── sidebars.js          # Navigation configuration
└── package.json         # Dependencies and scripts

api/                      # Backend API server
├── src/
│   ├── auth/            # Authentication logic
│   ├── user-profiles/   # User profile and progress management
│   ├── assessments/     # Assessment and grading logic
│   └── offline-content/ # Offline content management
├── tests/               # API tests
└── package.json         # Dependencies and scripts
```

## Key Commands

### Development
```bash
# Start frontend development server
cd website && npm start

# Start backend development server
cd api && npm run dev

# Build static site for production
cd website && npm run build
```

### Testing
```bash
# Run frontend tests
cd website && npm test

# Run backend tests
cd api && npm test

# Run all tests
npm run test:all
```

### Deployment
```bash
# Build and deploy to GitHub Pages
cd website && npm run deploy
```

## Module Development

### Adding New Curriculum Content

1. Create a new directory in `website/docs/` for your module
2. Add markdown files with curriculum content
3. Update `website/sidebars.js` to include your new module in the navigation
4. Use MDX components for interactive elements like assessments

### Example Module Structure
```
website/docs/module-1-nervous-system/
├── index.md             # Module overview
├── ros-basics.md        # ROS fundamentals
├── rclpy-integration.md # Python bridge content
├── urdf-modeling.md     # Chassis design content
└── assessment-1.mdx     # Interactive assessment
```

## Interactive Components

### Assessment Component

To add an assessment to your curriculum:

```mdx
import Assessment from '@site/src/components/Assessment';

<Assessment
  id="module-1-quiz-1"
  title="ROS Basics Quiz"
  maxScore={10}
  timeLimit={600} // 10 minutes in seconds
>
  {/* Questions will be loaded from the backend */}
</Assessment>
```

### Technical Diagram Component

To add technical diagrams:

```mdx
import TechnicalDiagram from '@site/src/components/TechnicalDiagram';

<TechnicalDiagram
  type="mermaid"
  title="Robot Architecture"
  description="System architecture diagram showing component relationships"
>
graph TD
    A[ROS Nodes] --> B[Communication Layer]
    B --> C[Control Algorithms]
    C --> D[Physical Robot]
</TechnicalDiagram>
```

## API Endpoints

### Authentication
- `POST /api/auth/login` - User login
- `POST /api/auth/register` - User registration
- `GET /api/auth/me` - Get current user info

### User Progress
- `GET /api/progress/:userId` - Get user progress
- `POST /api/progress/:userId` - Update user progress
- `GET /api/progress/:userId/module/:moduleId` - Get progress for specific module

### Assessments
- `GET /api/assessments/module/:moduleId` - Get assessments for module
- `POST /api/assessments/:assessmentId/attempt` - Start assessment attempt
- `PUT /api/assessments/:assessmentId/attempt/:attemptId` - Submit answers
- `GET /api/assessments/:assessmentId/attempt/:attemptId` - Get attempt results

### Offline Content
- `GET /api/offline/content/:moduleId` - Get offline content for module
- `POST /api/offline/download/:contentId` - Request offline content download

## Deployment to GitHub Pages

The project is configured for GitHub Pages deployment using GitHub Actions:

1. Ensure your repository is configured for GitHub Pages
2. The workflow is defined in `.github/workflows/deploy.yml`
3. Push changes to the main branch to trigger deployment

The workflow will:
1. Build the Docusaurus site
2. Deploy to the `gh-pages` branch
3. Configure GitHub Pages to serve from the branch

## Troubleshooting

### Common Issues

**Port already in use:**
```bash
# Kill process on port 3000
lsof -ti:3000 | xargs kill -9
# Then restart
npm start
```

**Dependency conflicts:**
```bash
# Clean and reinstall dependencies
rm -rf node_modules package-lock.json
npm install
```

**Authentication not working:**
- Verify `AUTH_SECRET` is set in environment
- Check that `NEXTAUTH_URL` matches your deployment URL
- Ensure CORS settings allow your frontend domain

### Development Tips

- Use `npm run build` to check for build errors before committing
- Run tests frequently to catch regressions
- Use the Docusaurus live reload feature during development
- Check browser console for client-side errors
- Monitor API server logs for backend issues