"""
Script to initialize the RAG system by creating the vector database collection
and testing the embedding services
"""
import os
from dotenv import load_dotenv
from src.services.qdrant_service import QdrantService
from src.services.embedding_service import EmbeddingService
from src.services.gemini_service import GeminiService

# Load environment variables
load_dotenv()

def initialize_rag_system():
    """Initialize the RAG system components"""
    print("[START] Initializing RAG System...")

    try:
        # Initialize services
        print("[INFO] Initializing Qdrant service...")
        qdrant_service = QdrantService()

        print("[INFO] Initializing embedding service...")
        embedding_service = EmbeddingService()

        print("[INFO] Initializing Gemini service...")
        gemini_service = GeminiService()

        # Create the collection in Qdrant
        print("[INFO] Creating/verifying Qdrant collection...")
        embedding_size = embedding_service.get_embedding_dimension()
        qdrant_service.create_collection(vector_size=embedding_size)

        # Test embedding generation
        print("[INFO] Testing embedding generation...")
        test_text = "This is a test document to verify the embedding service is working correctly."
        test_embedding = embedding_service.create_embedding(test_text)
        print(f"[SUCCESS] Generated embedding with {len(test_embedding)} dimensions")

        # Test Gemini response
        print("[INFO] Testing Gemini service...")
        test_response = gemini_service.generate_response("Hello, how are you?")
        print(f"[SUCCESS] Gemini response sample: {test_response[:50]}...")

        # Check vector count in collection
        vector_count = qdrant_service.get_vector_count()
        print(f"[INFO] Current vector count in collection: {vector_count}")

        print("\n[SUCCESS] RAG System initialized successfully!")
        print("You can now run the ingestion script to add your documents.")
        return True

    except Exception as e:
        print(f"[ERROR] Error initializing RAG system: {str(e)}")
        return False

def test_ingestion_sample():
    """Test the ingestion with a small sample"""
    print("\n[INFO] Testing with a sample document...")

    sample_content = """
    Chapter 1: Introduction to AI
    Artificial Intelligence (AI) is intelligence demonstrated by machines, in contrast to the natural intelligence displayed by humans and animals. Leading AI textbooks define the field as the study of "intelligent agents": any device that perceives its environment and takes actions that maximize its chance of successfully achieving its goals.

    The term "artificial intelligence" is often used to describe machines that mimic "cognitive" functions that humans associate with the human mind, such as "learning" and "problem solving". As machines become increasingly capable, tasks considered to require "intelligence" are often removed from the definition of AI, a phenomenon known as the AI effect.

    Modern machine learning techniques are at the heart of AI. Problems for AI applications include reasoning, knowledge representation, planning, learning, natural language processing, perception, and the ability to move and manipulate objects.
    """

    try:
        from src.services.qdrant_service import QdrantService
        from src.services.embedding_service import EmbeddingService
        import uuid

        qdrant_service = QdrantService()
        embedding_service = EmbeddingService()

        # Chunk the sample content
        from scripts.ingest_book import chunk_text
        chunks = chunk_text(sample_content, chunk_size=500, overlap=100)
        print(f"[SUCCESS] Content split into {len(chunks)} chunks")

        # Create embeddings for the chunks
        print("[INFO] Creating embeddings for sample chunks...")
        vectors_to_add = []
        for i, chunk in enumerate(chunks):
            embedding = embedding_service.create_embedding(chunk)

            vector_record = {
                "id": str(uuid.uuid4()),
                "vector": embedding,
                "payload": {
                    "content": chunk,
                    "chunk_index": i,
                    "source_file": "sample_test.txt",
                    "metadata": {
                        "chunk_size": len(chunk),
                        "source": "sample_test.txt"
                    }
                }
            }

            vectors_to_add.append(vector_record)

        # Add to Qdrant
        print("[INFO] Adding sample vectors to Qdrant...")
        qdrant_service.add_vectors(vectors_to_add)

        # Verify the count
        new_count = qdrant_service.get_vector_count()
        print(f"[INFO] New vector count in collection: {new_count}")

        print("[SUCCESS] Sample ingestion completed successfully!")
        return True

    except Exception as e:
        print(f"[ERROR] Error in sample ingestion: {str(e)}")
        return False

if __name__ == "__main__":
    success = initialize_rag_system()

    if success:
        test_ingestion_sample()

        print("\n" + "="*60)
        print("NEXT STEPS:")
        print("1. To ingest your own documents: python scripts/ingest_book.py --file path/to/your/book.pdf")
        print("2. To run the API server: uvicorn src.main:app --reload --port 8000")
        print("3. The API will be available at: http://localhost:8000")
        print("="*60)