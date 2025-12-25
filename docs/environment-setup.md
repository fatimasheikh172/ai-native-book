# Environment Setup for RAG Chatbot

This document provides instructions for setting up the environment variables required for the RAG Chatbot backend.

## Required Environment Variables

Create a `.env` file in the `backend/` directory with the following variables:

```bash
# Cohere API Configuration
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Configuration
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=docs_chunks

# Neon Postgres Configuration
DATABASE_URL=your_neon_postgres_connection_string_here

# Application Configuration
ENVIRONMENT=development
LOG_LEVEL=INFO
```

### Getting API Keys

#### Cohere API Key

1. Go to [Cohere](https://cohere.ai/)
2. Sign up for an account
3. Navigate to the API Keys section in your dashboard
4. Create a new API key
5. Copy the key and use it as `COHERE_API_KEY`

#### Qdrant Configuration

1. Sign up for [Qdrant Cloud](https://qdrant.tech/) or set up a local instance
2. If using Qdrant Cloud:
   - Get your cluster URL and API key from the dashboard
   - Use these as `QDRANT_URL` and `QDRANT_API_KEY`
3. If using local Qdrant:
   - Set `QDRANT_URL` to your local instance (e.g., `http://localhost:6333`)
   - API key may not be required for local instances

#### Neon Postgres Configuration

1. Sign up for [Neon](https://neon.tech/)
2. Create a new project
3. Copy the connection string from the project dashboard
4. Use this as `DATABASE_URL`

## Setting up Environment Variables

### Option 1: Using .env file (Recommended for Development)

1. Create a `.env` file in the `backend/` directory
2. Add the environment variables as shown above
3. The application will automatically load these variables

### Option 2: System Environment Variables

Set the environment variables at the system level:

**On Linux/Mac:**

```bash

export COHERE_API_KEY="your_key_here"
export QDRANT_URL="your_url_here"
export QDRANT_API_KEY="your_key_here"
export DATABASE_URL="your_database_url_here"
```

**On Windows:**

```cmd
set COHERE_API_KEY=your_key_here
set QDRANT_URL=your_url_here
set QDRANT_API_KEY=your_key_here
set DATABASE_URL=your_database_url_here
```

## Verification

To verify that your environment is properly configured:

1. Make sure all required environment variables are set
2. Test the API endpoints after starting the backend server
3. Check the application logs for any connection errors

## Security Notes

- Never commit the `.env` file to version control
- The `.gitignore` file should already include `.env` patterns
- Rotate your API keys periodically
- Use different keys for development and production environments
