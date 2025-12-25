import google.generativeai as genai
from typing import List, Dict, Any, Optional
from src.config.settings import settings


class GeminiService:
    def __init__(self):
        genai.configure(api_key=settings.gemini_api_key)
        self.model = genai.GenerativeModel('gemini-pro')

    def generate_response(self, prompt: str, context: Optional[List[Dict[str, Any]]] = None) -> str:
        """
        Generate a response using the Gemini model with optional context
        """
        try:
            # Build the full prompt with context if provided
            if context:
                # Create a more structured context with proper formatting
                context_parts = []
                for i, item in enumerate(context):
                    context_parts.append(f"Source {i+1}:\n{item['content']}")

                context_text = "\n\n".join(context_parts)

                # System prompt that forces Gemini to use only the provided context
                full_prompt = f"""
                SYSTEM INSTRUCTIONS:
                - You are a helpful assistant for a book-related RAG system
                - You must ONLY use information from the provided context sources to answer questions
                - Do NOT use any external knowledge or general world knowledge
                - If the context doesn't contain the information needed to answer, explicitly state this
                - Be concise but thorough in your responses
                - Maintain a professional and helpful tone

                CONTEXT SOURCES:
                {context_text}

                USER QUERY:
                {prompt}

                Please provide a response based ONLY on the context sources provided above.
                """
            else:
                # If no context is provided, respond appropriately
                full_prompt = f"""
                SYSTEM INSTRUCTIONS:
                - You are a helpful assistant for a book-related RAG system
                - No context sources were provided to answer the user's query
                - Acknowledge this limitation and respond accordingly

                USER QUERY:
                {prompt}

                Please acknowledge that no context sources were provided to answer this query.
                """

            response = self.model.generate_content(full_prompt)
            return response.text
        except Exception as e:
            print(f"Error generating response: {str(e)}")
            # Fallback response when external API is unavailable
            if context:
                return ("I'm currently unable to generate a response due to a service issue. "
                       "The system has identified the following relevant sources, but I cannot "
                       "generate a synthesized answer at this time. Please try again later.\n\n"
                       f"Relevant sources found: {len(context)}")
            else:
                return "I'm currently unable to process your query due to a service issue. Please try again later."

    def generate_selection_based_response(self, query: str, selection_text: str) -> str:
        """
        Generate a response based only on the provided text selection
        """
        try:
            # System prompt that enforces strict adherence to the provided text selection
            prompt = f"""
            SYSTEM INSTRUCTIONS:
            - You are a helpful assistant for a book-related RAG system
            - You must ONLY use information from the provided text selection to answer questions
            - Do NOT use any external knowledge or general world knowledge
            - If the provided text doesn't contain the information needed to answer, explicitly state this
            - Be concise but thorough in your responses
            - Maintain a professional and helpful tone

            PROVIDED TEXT SELECTION:
            {selection_text}

            USER QUERY ABOUT THE ABOVE TEXT:
            {query}

            Please provide a response based ONLY on the provided text selection above.
            """

            response = self.model.generate_content(prompt)
            return response.text
        except Exception as e:
            print(f"Error generating selection-based response: {str(e)}")
            # Fallback response when external API is unavailable
            return ("I'm currently unable to generate a response due to a service issue. "
                   "I have received your query about the provided text selection, but I cannot "
                   "process it at this time. Please try again later.")

    def count_tokens(self, text: str) -> int:
        """
        Count the number of tokens in the given text
        """
        try:
            result = self.model.count_tokens(text)
            return result.total_tokens
        except Exception as e:
            print(f"Error counting tokens: {str(e)}")
            return len(text.split())  # Fallback to word count