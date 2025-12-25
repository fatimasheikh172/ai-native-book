from sqlalchemy import Column, String, DateTime, Text, Integer, UUID, ForeignKey, JSON
from sqlalchemy.dialects.postgresql import UUID as PostgresUUID, JSONB
from sqlalchemy.dialects.sqlite import DATETIME
from sqlalchemy.sql import func
import uuid
from src.config.database import Base


class ChatMessage(Base):
    __tablename__ = "chat_messages"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(UUID(as_uuid=True), ForeignKey("chat_sessions.id"), nullable=False)
    role = Column(String, nullable=False)  # 'user', 'assistant', 'system'
    content = Column(Text, nullable=False)
    timestamp = Column(DateTime, server_default=func.now(), nullable=False)
    context_chunks = Column(JSON, nullable=True)  # references to Qdrant IDs used in response
    token_count = Column(Integer, nullable=True)  # for usage tracking