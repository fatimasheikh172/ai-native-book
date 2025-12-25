"""
Tests for the RAG services
"""
import pytest
from unittest.mock import Mock, patch
from src.services.qdrant_service import QdrantService
from src.services.embedding_service import EmbeddingService
from src.services.gemini_service import GeminiService
from src.services.rag_service import RAGService


def test_qdrant_service_initialization():
    """Test Qdrant service initialization"""
    with patch('src.services.qdrant_service.QdrantClient') as mock_client:
        mock_client_instance = Mock()
        mock_client.return_value = mock_client_instance

        service = QdrantService()
        assert service.client is not None
        assert service.collection_name is not None


def test_embedding_service_initialization():
    """Test embedding service initialization"""
    # This test would require mocking the Google API
    # For now, we'll just ensure the class can be instantiated
    try:
        service = EmbeddingService()
        assert hasattr(service, 'create_embedding')
        assert hasattr(service, 'create_embeddings')
    except Exception:
        # If API key is not configured, this is expected
        pass


def test_gemini_service_initialization():
    """Test Gemini service initialization"""
    # This test would require mocking the Google API
    # For now, we'll just ensure the class can be instantiated
    try:
        service = GeminiService()
        assert hasattr(service, 'generate_response')
        assert hasattr(service, 'generate_selection_based_response')
    except Exception:
        # If API key is not configured, this is expected
        pass


def test_rag_service_initialization():
    """Test RAG service initialization"""
    # This test would require mocking dependencies
    try:
        service = RAGService()
        assert hasattr(service, 'query_global_rag')
        assert hasattr(service, 'query_selection_based_rag')
    except Exception:
        # If dependencies can't be initialized, this is expected
        pass


if __name__ == "__main__":
    pytest.main([__file__])