#!/usr/bin/env python3
"""
Script to process book content (PDF/Text) and upload to Qdrant
"""
import argparse
import os
import uuid
from pathlib import Path
from typing import List, Dict, Any
import PyPDF2
from src.config.settings import settings
from src.services.qdrant_service import QdrantService
from src.services.embedding_service import EmbeddingService


def read_pdf(file_path: str) -> str:
    """
    Read text content from a PDF file
    """
    text = ""
    with open(file_path, 'rb') as file:
        pdf_reader = PyPDF2.PdfReader(file)
        for page in pdf_reader.pages:
            text += page.extract_text()
    return text


def read_text(file_path: str) -> str:
    """
    Read text content from a text file
    """
    with open(file_path, 'r', encoding='utf-8') as file:
        return file.read()


def chunk_text(text: str, chunk_size: int = 1000, overlap: int = 200) -> List[str]:
    """
    Split text into overlapping chunks
    """
    chunks = []
    start = 0

    while start < len(text):
        end = start + chunk_size

        # If we're near the end, include the remainder
        if end > len(text):
            end = len(text)

        chunk = text[start:end]

        # Only add non-empty chunks
        if chunk.strip():
            chunks.append(chunk)

        # Move start forward by chunk_size - overlap
        start = end - overlap

        # Ensure we make progress to avoid infinite loop
        if start <= 0:
            start += 1
        elif start >= len(text):
            break

        # Safety check to prevent potential infinite loops
        if len(chunks) > 10000:  # Arbitrary large number as safety
            print(f"Warning: Too many chunks generated ({len(chunks)}), stopping to prevent memory issues")
            break

    return chunks


def main():
    parser = argparse.ArgumentParser(description="Ingest book content into Qdrant vector database")
    parser.add_argument("--file", type=str, required=True, help="Path to the book file (PDF or text)")
    parser.add_argument("--chunk-size", type=int, default=1500, help="Size of text chunks (default: 1500)")
    parser.add_argument("--overlap", type=int, default=200, help="Overlap between chunks (default: 200)")
    parser.add_argument("--recreate", action="store_true", help="Recreate the collection (clear existing data)")

    args = parser.parse_args()

    # Validate file exists
    if not os.path.exists(args.file):
        print(f"Error: File {args.file} does not exist")
        return

    # Determine file type and read content
    file_path = Path(args.file)
    if file_path.suffix.lower() == '.pdf':
        content = read_pdf(args.file)
    elif file_path.suffix.lower() in ['.txt', '.md']:
        content = read_text(args.file)
    else:
        print(f"Error: Unsupported file type {file_path.suffix}. Supported types: .pdf, .txt, .md")
        return

    print(f"Read {len(content)} characters from {args.file}")

    # Chunk the content
    chunks = chunk_text(content, args.chunk_size, args.overlap)
    print(f"Split content into {len(chunks)} chunks")

    # Initialize services
    qdrant_service = QdrantService()
    embedding_service = EmbeddingService()

    # Optionally recreate collection
    if args.recreate:
        print("Recreating collection...")
        qdrant_service.clear_collection()

    # Create collection if it doesn't exist
    qdrant_service.create_collection(vector_size=embedding_service.get_embedding_dimension())

    # Process each chunk
    print("Processing chunks and generating embeddings...")
    vectors_to_add = []

    for i, chunk in enumerate(chunks):
        print(f"Processing chunk {i+1}/{len(chunks)}...")

        # Generate embedding for the chunk
        embedding = embedding_service.create_embedding(chunk)

        # Create a vector record with metadata
        vector_record = {
            "id": str(uuid.uuid4()),
            "vector": embedding,
            "payload": {
                "content": chunk,
                "chunk_index": i,
                "source_file": str(file_path.name),
                "metadata": {
                    "chunk_size": len(chunk),
                    "source": str(file_path.name)
                }
            }
        }

        vectors_to_add.append(vector_record)

        # Batch upload every 10 chunks to avoid memory issues
        if len(vectors_to_add) >= 10:
            qdrant_service.add_vectors(vectors_to_add)
            print(f"Uploaded batch of {len(vectors_to_add)} vectors to Qdrant")
            vectors_to_add = []

    # Upload remaining vectors
    if vectors_to_add:
        qdrant_service.add_vectors(vectors_to_add)
        print(f"Uploaded final batch of {len(vectors_to_add)} vectors to Qdrant")

    # Print summary
    total_count = qdrant_service.get_vector_count()
    print(f"Ingestion complete! Total vectors in collection: {total_count}")


if __name__ == "__main__":
    main()