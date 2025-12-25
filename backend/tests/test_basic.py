"""
Basic tests for the RAG system
"""
import pytest
from fastapi.testclient import TestClient
from src.main import app


def test_health_endpoint():
    """Test the health check endpoint"""
    client = TestClient(app)
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json() == {"status": "healthy", "service": "docucures-rag-chatbot"}


def test_api_routes_exist():
    """Test that main API routes exist"""
    client = TestClient(app)

    # Test that API routes exist (should return 422 for missing request body rather than 404)
    response = client.post("/api/rag/query")
    # Should get 422 for missing request body, not 404 Not Found
    assert response.status_code in [422, 500]  # 422 for validation errors, 500 for other errors

    response = client.get("/api/chat/sessions")
    # Should return 200 or 500, not 404
    assert response.status_code in [200, 500]

    # Note: /api/user/profile route has been removed as part of authentication removal


if __name__ == "__main__":
    pytest.main([__file__])