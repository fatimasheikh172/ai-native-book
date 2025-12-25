from typing import List, Dict, Any, Optional
from src.services.qdrant_service import QdrantService
from src.services.embedding_service import EmbeddingService
from src.services.gemini_service import GeminiService
from src.config.settings import settings


class RAGService:
    def __init__(self):
        self.qdrant_service = QdrantService()
        self.embedding_service = EmbeddingService()
        self.gemini_service = GeminiService()

    def query_global_rag(self, query: str, top_k: int = 5, score_threshold: float = 0.7, context_filter: Optional[Dict] = None) -> Dict[str, Any]:
        """
        Perform a global RAG query across the entire book content
        """
        # Generate embedding for the query
        query_embedding = self.embedding_service.create_embedding(query)

        # Search in Qdrant for similar content
        search_results = self.qdrant_service.search_vectors(
            query_vector=query_embedding,
            top_k=top_k,
            score_threshold=score_threshold,
            filters=context_filter
        )

        # Extract context from search results
        context_chunks = []
        sources = []
        for result in search_results:
            context_chunks.append({
                "content": result["payload"]["content"],
                "score": result["score"]
            })
            sources.append({
                "id": result["id"],
                "text": result["payload"]["content"][:200] + "..." if len(result["payload"]["content"]) > 200 else result["payload"]["content"],  # Truncate for display
                "source": result["payload"]["source_file"],
                "score": result["score"]
            })

        # Generate response using Gemini with the context
        if context_chunks:
            response = self.gemini_service.generate_response(
                prompt=query,
                context=context_chunks
            )
        else:
            response = "I couldn't find relevant information in the book to answer your question."

        return {
            "response": response,
            "sources": sources,
            "context_chunks_used": len(context_chunks)
        }

    def query_selection_based_rag(self, query: str, selection_text: str) -> str:
        """
        Perform a RAG query based only on the provided text selection
        """
        # Generate response using only the provided selection text
        response = self.gemini_service.generate_selection_based_response(
            query=query,
            selection_text=selection_text
        )

        return response

    def get_vector_count(self) -> int:
        """
        Get the total count of vectors in the Qdrant collection
        """
        return self.qdrant_service.get_vector_count()