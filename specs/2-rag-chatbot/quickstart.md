# Quickstart: Docucures RAG System

## Prerequisites

- Python 3.11+
- pip package manager
- Git
- Access to Google Gemini API
- Qdrant Cloud account
- Neon Postgres account

## Setup Instructions

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Create Virtual Environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies
```bash
cd backend-rag
pip install -r requirements.txt
```

### 4. Configure Environment Variables
Create a `.env` file in the backend-rag directory:
```env
# Google Gemini API
GEMINI_API_KEY=your_gemini_api_key_here

# Qdrant Cloud
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=book_content_chunks

# Neon Postgres
DATABASE_URL=postgresql://username:password@ep-xxxxxxx.us-east-1.aws.neon.tech/dbname?sslmode=require

# JWT Configuration
JWT_SECRET_KEY=your_jwt_secret_key_here
JWT_ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30
REFRESH_TOKEN_EXPIRE_DAYS=30

# Application Settings
DEBUG=false
APP_NAME=Docucures RAG Chatbot
```

### 5. Initialize the Database
```bash
# Run database migrations
python -m src.main init_db
```

### 6. Ingest Book Content
```bash
# Process and ingest your book content into the vector database
python scripts/ingest_book.py --file path/to/book.pdf --chunk-size 1500
```

### 7. Start the Application
```bash
# Using uvicorn
uvicorn src.main:app --host 0.0.0.0 --port 8000 --reload

# Or using the run script
python -m src.main
```

## API Usage Examples

### Authentication
```bash
# Register a new user
curl -X POST http://localhost:8000/api/auth/register \
  -H "Content-Type: application/json" \
  -d '{"email": "user@example.com", "password": "securepassword", "first_name": "John", "last_name": "Doe"}'

# Login
curl -X POST http://localhost:8000/api/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email": "user@example.com", "password": "securepassword"}'
```

### Query the RAG System
```bash
# Query with authentication token
curl -X POST http://localhost:8000/api/rag/query \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer your_access_token_here" \
  -d '{"query": "What are the key concepts in chapter 3?"}'
```

## Development Commands

### Running Tests
```bash
# Run all tests
python -m pytest

# Run tests with coverage
python -m pytest --cov=src
```

### Data Ingestion
```bash
# Re-ingest book content (this will clear existing vectors)
python scripts/ingest_book.py --file path/to/book.pdf --chunk-size 1500 --recreate
```

### Local Development
```bash
# Start with auto-reload
uvicorn src.main:app --host 0.0.0.0 --port 8000 --reload

# Start with logging
uvicorn src.main:app --host 0.0.0.0 --port 8000 --reload --log-level info
```

## Troubleshooting

### Common Issues

1. **Qdrant Connection Error**: Verify your QDRANT_URL and QDRANT_API_KEY are correct
2. **Gemini API Error**: Check that your GEMINI_API_KEY is valid and has sufficient quota
3. **Database Connection Error**: Ensure your DATABASE_URL is properly formatted
4. **Authentication Issues**: Verify JWT_SECRET_KEY is set and consistent across services

### Checking System Status
```bash
# Health check endpoint
curl http://localhost:8000/health
```