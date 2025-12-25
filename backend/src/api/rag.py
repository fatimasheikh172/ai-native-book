from fastapi import APIRouter, Depends, HTTPException, status, Request
from sqlalchemy.orm import Session
from typing import Optional, Dict, Any
from pydantic import BaseModel
import random
import json

from src.config.database import get_db
from src.services.rag_service import RAGService

router = APIRouter()


class RAGQueryRequest(BaseModel):
    query: str
    session_id: Optional[str] = None
    context_filter: Optional[Dict] = None


class RAGQuerySelectionRequest(BaseModel):
    query: str
    selection_text: str
    session_id: Optional[str] = None


@router.post("/query")
async def rag_query(
    request: RAGQueryRequest,
    db: Session = Depends(get_db)
):
    try:
        rag_service = RAGService()
        result = rag_service.query_global_rag(
            query=request.query,
            context_filter=request.context_filter
        )

        return {
            "success": True,
            "response": result["response"],
            "session_id": request.session_id,  # In a full implementation, this would be created/updated
            "sources": result["sources"]
        }
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"RAG query failed: {str(e)}"
        )


@router.post("/query-selection")
async def rag_query_selection(
    request: RAGQuerySelectionRequest,
    db: Session = Depends(get_db)
):
    try:
        rag_service = RAGService()
        response = rag_service.query_selection_based_rag(
            query=request.query,
            selection_text=request.selection_text
        )

        return {
            "success": True,
            "response": response,
            "session_id": request.session_id  # In a full implementation, this would be created/updated
        }
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"RAG selection query failed: {str(e)}"
        )


# Mock endpoint for testing when Qdrant is not available
@router.post("/query-mock")
async def mock_rag_query(
    request: RAGQueryRequest
):
    """
    Mock RAG query endpoint for testing purposes when Qdrant is not available.
    Returns simulated responses based on the query.
    """
    query = request.query.lower()

    # Simple response mapping based on keywords
    responses = {
        "hello": "Hello! I'm the AI assistant for the Physical AI and Human-Aided Robotics book. How can I help you today?",
        "book": "This book covers Physical AI and Human-Aided Robotics, exploring how to build intelligent machines for the real world, from sensors to humanoid intelligence.",
        "robotics": "The book covers various aspects of robotics including ROS 2, sensors, digital twins, AI agents, and humanoid systems.",
        "ai": "The book explores artificial intelligence in the context of physical systems and robotics, focusing on embodied intelligence.",
        "physical ai": "Physical AI refers to artificial intelligence systems that interact with the physical world through sensors and actuators.",
        "help": "I can answer questions about the Physical AI and Human-Aided Robotics book. Ask me about specific topics like robotics, AI, sensors, ROS 2, etc."
    }

    # Find a matching response based on keywords
    response = "I'm sorry, I don't have specific information about that topic in my knowledge base. The Physical AI and Human-Aided Robotics book covers advanced robotics, AI, ROS 2, sensors, digital twins, and humanoid systems. Is there a specific aspect you'd like to know more about?"

    for keyword, reply in responses.items():
        if keyword in query:
            response = reply
            break

    # Add some randomness for variety
    if "not sure" in response.lower() or "don't know" in response.lower():
        fallback_responses = [
            "The Physical AI and Human-Aided Robotics book explores building intelligent machines for real-world applications.",
            "I recommend checking the book modules for detailed information on robotics and AI systems.",
            "The book covers topics like ROS 2, sensors, digital twins, and AI brain systems."
        ]
        response = random.choice(fallback_responses)

    return {
        "success": True,
        "response": response,
        "session_id": request.session_id,
        "sources": ["Book Content", "Module Documentation"]
    }