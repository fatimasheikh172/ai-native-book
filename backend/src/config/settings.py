from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    # Application settings
    app_name: str = "Docucures RAG Chatbot"
    debug: bool = False

    # Database settings
    database_url: str

    # Qdrant settings
    qdrant_url: str
    qdrant_api_key: Optional[str] = None
    qdrant_collection_name: str = "book_content_chunks"

    # Gemini settings
    gemini_api_key: str

    class Config:
        env_file = ".env"


settings = Settings()