from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import List, Dict, Any
from pydantic import BaseModel
import uuid

from src.config.database import get_db
from src.models.chat_session import ChatSession
from src.models.chat_message import ChatMessage

router = APIRouter()


class ChatSessionResponse(BaseModel):
    id: str
    title: str
    created_at: str
    updated_at: str


class ChatMessageResponse(BaseModel):
    id: str
    role: str
    content: str
    timestamp: str


class GetSessionResponse(BaseModel):
    session: ChatSessionResponse
    messages: List[ChatMessageResponse]


@router.get("/sessions")
async def list_sessions(
    db: Session = Depends(get_db)
):
    try:
        # For now, return all active sessions (without user filtering)
        # In a real implementation, you might want to implement session ownership differently
        sessions = db.query(ChatSession).filter(
            ChatSession.is_active == True
        ).all()

        session_list = []
        for session in sessions:
            session_list.append({
                "id": str(session.id),
                "title": session.title,
                "created_at": session.created_at.isoformat(),
                "updated_at": session.updated_at.isoformat()
            })

        return {
            "success": True,
            "sessions": session_list
        }
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to list sessions: {str(e)}"
        )


@router.get("/sessions/{session_id}")
async def get_session(
    session_id: str,
    db: Session = Depends(get_db)
):
    try:
        # Get the session (without user verification)
        session = db.query(ChatSession).filter(
            ChatSession.id == session_id,
            ChatSession.is_active == True
        ).first()

        if not session:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Session not found"
            )

        # Get messages for the session
        messages = db.query(ChatMessage).filter(
            ChatMessage.session_id == session_id
        ).order_by(ChatMessage.timestamp).all()

        message_list = []
        for message in messages:
            message_list.append({
                "id": str(message.id),
                "role": message.role,
                "content": message.content,
                "timestamp": message.timestamp.isoformat()
            })

        session_response = {
            "id": str(session.id),
            "title": session.title,
            "created_at": session.created_at.isoformat(),
            "updated_at": session.updated_at.isoformat()
        }

        return {
            "success": True,
            "session": session_response,
            "messages": message_list
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to get session: {str(e)}"
        )


@router.delete("/sessions/{session_id}")
async def delete_session(
    session_id: str,
    db: Session = Depends(get_db)
):
    try:
        # Get the session (without user verification)
        session = db.query(ChatSession).filter(
            ChatSession.id == session_id
        ).first()

        if not session:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Session not found"
            )

        # Instead of deleting, mark as inactive
        session.is_active = False
        db.commit()

        return {
            "success": True
        }
    except HTTPException:
        raise
    except Exception as e:
        db.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to delete session: {str(e)}"
        )