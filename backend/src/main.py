from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from src.api import rag, chat
from src.config.database import engine
from src.models import chat_session, chat_message
from contextlib import asynccontextmanager
import os
from dotenv import load_dotenv
import logging

# Load environment variables
load_dotenv()

# Create logs directory if it doesn't exist
os.makedirs("logs", exist_ok=True)

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    # Create database tables
    from sqlalchemy import text
    with engine.connect() as conn:
        # Only create PostgreSQL extensions if using PostgreSQL
        if "postgresql" in str(engine.url):
            conn.execute(text("CREATE EXTENSION IF NOT EXISTS \"uuid-ossp\""))  # For UUID generation
        conn.commit()

    # Create tables
    chat_session.Base.metadata.create_all(bind=engine)
    chat_message.Base.metadata.create_all(bind=engine)

    yield

    # Shutdown
    # Any cleanup code here if needed

app = FastAPI(
    title="Docucures RAG Chatbot API",
    description="API for the Docucures RAG Chatbot system",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Health check endpoint
@app.get("/health")
async def health_check():
    return {"status": "healthy", "service": "docucures-rag-chatbot"}

# Include API routes
app.include_router(rag.router, prefix="/api/rag", tags=["rag"])
app.include_router(chat.router, prefix="/api/chat", tags=["chat"])

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)