# AI Native Book Development Guidelines

Auto-generated from all feature plans. Last updated: 2025-12-16

## Active Technologies

- Python 3.11 (Backend API)
- FastAPI (Web framework)
- Qdrant (Vector database)
- Cohere API (LLM provider)
- Cohere models (Embeddings)
- Neon Postgres (Conversation history)
- React 18 (Frontend framework)
- Docusaurus (Documentation site)
- Transformers library (ML models)
- Pytest (Backend testing)
- Jest/React Testing Library (Frontend testing)

## Project Structure

```text
backend/
├── src/
│   ├── models/
│   ├── services/
│   ├── api/
│   └── utils/
├── scripts/
│   └── ingest_docs.py
└── tests/

frontend/
├── src/
│   ├── components/
│   │   └── ChatWidget/
│   ├── hooks/
│   └── services/
└── tests/
```

## Commands

### Backend Development
```bash
# Start backend server
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
uvicorn src.main:app --reload --port 8000

# Run backend tests
cd backend
pytest
```

### Data Ingestion
```bash
# Ingest documentation into vector database
cd backend
python scripts/ingest_docs.py --docs-path /path/to/your/docusaurus/docs
```

### Environment Setup
```bash
# Backend environment variables
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
DATABASE_URL=your_neon_postgres_connection_string
QDRANT_COLLECTION_NAME=docs_chunks
```

## Code Style

### Python
- Follow PEP 8 guidelines
- Use type hints for all function parameters and return values
- Prefer FastAPI dependency injection for shared resources
- Use Pydantic models for request/response validation

### JavaScript/React
- Use functional components with hooks
- Follow React best practices for state management
- Use TypeScript for type safety where possible
- Implement proper error boundaries

## Recent Changes

- RAG Chatbot: Added vector database integration (Qdrant) for documentation search
- RAG Chatbot: Implemented backend API with FastAPI for chat functionality
- RAG Chatbot: Added conversation history with Neon Postgres
- RAG Chatbot: Integrated Google Gemini API for advanced language processing
- RAG Chatbot: Added contextual chat feature for text selection queries
- RAG Chatbot: Implemented Docusaurus integration with floating chat widget

<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->