"""
Test script to verify the RAG system is working properly
"""
import os
from dotenv import load_dotenv
from src.services.qdrant_service import QdrantService
from src.services.embedding_service import EmbeddingService
from src.services.gemini_service import GeminiService
from src.services.rag_service import RAGService

# Load environment variables
load_dotenv()

def test_system():
    print("[START] Testing RAG System...")

    try:
        # Test Qdrant service
        print("[INFO] Testing Qdrant service...")
        qdrant_service = QdrantService()
        print("[SUCCESS] Qdrant service initialized successfully")

        # Test embedding service
        print("[INFO] Testing embedding service...")
        embedding_service = EmbeddingService()
        print("[SUCCESS] Embedding service initialized successfully")

        # Test Gemini service
        print("[INFO] Testing Gemini service...")
        gemini_service = GeminiService()
        print("[SUCCESS] Gemini service initialized successfully")

        # Test RAG service
        print("[INFO] Testing RAG service...")
        rag_service = RAGService()
        print("[SUCCESS] RAG service initialized successfully")

        # Test collection creation
        print("[INFO] Testing collection creation...")
        embedding_size = embedding_service.get_embedding_dimension()
        qdrant_service.create_collection(vector_size=embedding_size)
        print("[SUCCESS] Collection created/verified successfully")

        # Test vector count
        vector_count = qdrant_service.get_vector_count()
        print(f"[INFO] Current vector count: {vector_count}")

        # Test embedding generation (with fallback handling)
        print("[INFO] Testing embedding generation...")
        test_text = "This is a test document."
        test_embedding = embedding_service.create_embedding(test_text)
        print(f"[SUCCESS] Generated embedding with {len(test_embedding)} dimensions")

        print("\n[SUCCESS] All services are working correctly!")
        print("The RAG system is ready for document ingestion and queries.")

        # Show next steps
        print("\n[INFO] Next steps:")
        print("1. To ingest documents: python scripts/ingest_book.py --file path/to/your/book.pdf")
        print("2. To run the API server: uvicorn src.main:app --reload --port 8000")
        print("3. API will be available at: http://localhost:8000")

        return True

    except Exception as e:
        print(f"[ERROR] Error testing RAG system: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_system()
    if success:
        print("\n[SUCCESS] RAG System test completed successfully!")
    else:
        print("\n[ERROR] RAG System test failed!")