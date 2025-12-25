from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct, FieldCondition, Filter, MatchAny, MatchValue
from src.config.settings import settings
import uuid


class QdrantService:
    def __init__(self):
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            prefer_grpc=True
        )
        self.collection_name = settings.qdrant_collection_name

    def create_collection(self, vector_size: int = 1536):
        """
        Create a collection in Qdrant for storing book content chunks
        """
        try:
            # Check if collection already exists
            self.client.get_collection(self.collection_name)
            print(f"Collection {self.collection_name} already exists")
        except:
            # Create collection if it doesn't exist
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=vector_size,
                    distance=models.Distance.COSINE
                )
            )
            print(f"Created collection {self.collection_name}")

    def add_vectors(self, vectors: List[Dict[str, Any]]):
        """
        Add vectors to the collection
        Each vector dict should contain: id, vector, payload
        """
        points = []
        for item in vectors:
            points.append(PointStruct(
                id=item["id"],
                vector=item["vector"],
                payload=item["payload"]
            ))

        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

    def search_vectors(self, query_vector: List[float], top_k: int = 5, score_threshold: float = 0.7,
                      filters: Optional[Dict[str, Any]] = None) -> List[Dict[str, Any]]:
        """
        Search for similar vectors in the collection with optional filters
        """
        # Build filters if provided
        search_filter = None
        if filters:
            filter_conditions = []
            for key, value in filters.items():
                if isinstance(value, list):
                    # Handle list of values (e.g., source filters)
                    filter_conditions.append(
                        FieldCondition(
                            key=f"metadata.{key}",
                            match=MatchAny(any=value)
                        )
                    )
                else:
                    # Handle single value
                    filter_conditions.append(
                        FieldCondition(
                            key=f"metadata.{key}",
                            match=MatchValue(value=value)
                        )
                    )

            if filter_conditions:
                search_filter = Filter(
                    must=filter_conditions
                )

        results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_vector,
            limit=top_k,
            score_threshold=score_threshold,
            query_filter=search_filter
        )

        # Format results
        formatted_results = []
        for result in results:
            formatted_results.append({
                "id": result.id,
                "score": result.score,
                "payload": result.payload
            })

        return formatted_results

    def delete_vectors(self, vector_ids: List[str]):
        """
        Delete vectors by their IDs
        """
        self.client.delete(
            collection_name=self.collection_name,
            points_selector=models.PointIdsList(
                points=vector_ids
            )
        )

    def get_vector_count(self) -> int:
        """
        Get the total count of vectors in the collection
        """
        collection_info = self.client.get_collection(self.collection_name)
        return collection_info.points_count

    def clear_collection(self):
        """
        Clear all vectors from the collection
        """
        self.client.delete(
            collection_name=self.collection_name,
            points_selector=models.FilterSelector(
                filter=models.Filter()
            )
        )