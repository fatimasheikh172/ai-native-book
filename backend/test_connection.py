"""
Simple test script to verify Qdrant and Gemini connections
"""
import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
import google.generativeai as genai

# Load environment variables
load_dotenv()

def test_qdrant_connection():
    """Test Qdrant connection"""
    try:
        qdrant_client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY"),
        )

        collections = qdrant_client.get_collections()
        print("✅ Qdrant connection successful!")
        print(f"Available collections: {collections}")
        return True
    except Exception as e:
        print(f"❌ Qdrant connection failed: {e}")
        return False

def test_gemini_connection():
    """Test Gemini connection"""
    try:
        genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
        model = genai.GenerativeModel('gemini-pro')

        # Test with a simple prompt
        response = model.generate_content("Hello, how are you?")
        print("✅ Gemini connection successful!")
        print(f"Test response: {response.text[:50]}...")
        return True
    except Exception as e:
        print(f"❌ Gemini connection failed: {e}")
        return False

if __name__ == "__main__":
    print("Testing connections...")
    qdrant_ok = test_qdrant_connection()
    gemini_ok = test_gemini_connection()

    if qdrant_ok and gemini_ok:
        print("\n🎉 All connections are working!")
    else:
        print("\n⚠️  Some connections failed. Please check your API keys.")