from sqlalchemy import Column, String, DateTime, Integer, UUID, ForeignKey, JSON
from sqlalchemy.dialects.postgresql import UUID as PostgresUUID, JSONB
from sqlalchemy.dialects.sqlite import DATETIME
from sqlalchemy.sql import func
import uuid
from src.config.database import Base


class APIUsageLog(Base):
    __tablename__ = "api_usage_logs"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id"), nullable=True)  # Optional for anonymous usage
    session_id = Column(UUID(as_uuid=True), ForeignKey("chat_sessions.id"), nullable=True)  # Optional
    api_type = Column(String, nullable=False)  # 'gemini', 'embedding', 'qdrant', 'neon'
    request_data = Column(JSON, nullable=True)  # for debugging
    response_time_ms = Column(Integer, nullable=False)
    timestamp = Column(DateTime, server_default=func.now(), nullable=False)
    token_usage = Column(JSON, nullable=True)  # for tracking API costs