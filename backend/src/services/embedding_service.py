import google.generativeai as genai
from typing import List
from src.config.settings import settings


class EmbeddingService:
    def __init__(self):
        genai.configure(api_key=settings.gemini_api_key)

    def create_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Create embeddings for the given texts using Google's embedding model
        """
        try:
            # Use the embedding method from the genai library
            # For multiple texts, we need to embed them one by one
            embeddings = []
            for text in texts:
                result = genai.embed_content(
                    model="models/embedding-001",  # Using Google's embedding model
                    content=text,
                    task_type="RETRIEVAL_DOCUMENT"
                )
                embeddings.append(result['embedding'])

            return embeddings
        except Exception as e:
            print(f"Error creating embeddings: {str(e)}")
            # Fallback: return zero vectors of appropriate dimension
            # In a real implementation, you might want to queue the request for later processing
            fallback_embedding = [0.0] * 768  # Default embedding dimension
            return [fallback_embedding for _ in texts]

    def create_embedding(self, text: str) -> List[float]:
        """
        Create a single embedding for the given text
        """
        embeddings = self.create_embeddings([text])
        return embeddings[0] if embeddings else [0.0] * 768

    def get_embedding_dimension(self) -> int:
        """
        Get the dimension of the embeddings (for collection creation)
        Google's embedding-001 model returns 768-dimensional embeddings
        """
        return 768