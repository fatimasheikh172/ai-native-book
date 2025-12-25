# Data Model: Docucures RAG System

## User Account
- **id**: UUID (Primary Key)
- **email**: String (Unique, Required)
- **password_hash**: String (Required)
- **first_name**: String (Optional)
- **last_name**: String (Optional)
- **created_at**: DateTime (Required)
- **updated_at**: DateTime (Required)
- **last_login_at**: DateTime (Optional)
- **is_active**: Boolean (Default: True)

## Chat Session
- **id**: UUID (Primary Key)
- **user_id**: UUID (Foreign Key: User Account)
- **title**: String (Required, auto-generated from first query)
- **created_at**: DateTime (Required)
- **updated_at**: DateTime (Required)
- **is_active**: Boolean (Default: True)
- **metadata**: JSON (Optional, for storing conversation context)

## Chat Message
- **id**: UUID (Primary Key)
- **session_id**: UUID (Foreign Key: Chat Session)
- **user_id**: UUID (Foreign Key: User Account)
- **role**: String (Enum: 'user', 'assistant', 'system')
- **content**: Text (Required)
- **timestamp**: DateTime (Required)
- **context_chunks**: JSON (Optional, references to Qdrant IDs used in response)
- **token_count**: Integer (Optional, for usage tracking)

## Book Content Chunk
- **qdrant_id**: UUID (Primary Key, matches Qdrant vector ID)
- **content**: Text (Required, the actual chunk text)
- **metadata**: JSON (Required, contains: page_number, chapter, section, source_file)
- **embedding**: Vector (1536-dimensions from text-embedding-004)
- **created_at**: DateTime (Required)

## User Session Token
- **id**: UUID (Primary Key)
- **user_id**: UUID (Foreign Key: User Account)
- **token**: String (JWT token, Required, Indexed)
- **expires_at**: DateTime (Required)
- **created_at**: DateTime (Required)
- **is_revoked**: Boolean (Default: False)

## API Usage Log
- **id**: UUID (Primary Key)
- **user_id**: UUID (Foreign Key: User Account, Optional for anonymous usage)
- **session_id**: UUID (Foreign Key: Chat Session, Optional)
- **api_type**: String (Enum: 'gemini', 'embedding', 'qdrant', 'neon')
- **request_data**: JSON (Optional, for debugging)
- **response_time_ms**: Integer (Required)
- **timestamp**: DateTime (Required)
- **token_usage**: JSON (Optional, for tracking API costs)

## Relationships
- User Account (1) → (Many) Chat Session
- Chat Session (1) → (Many) Chat Message
- User Account (1) → (Many) User Session Token
- User Account (1) → (Many) API Usage Log
- Chat Session (1) → (Many) Chat Message

## Validation Rules
- User email must be unique and valid email format
- Password must meet minimum strength requirements (8+ chars, mixed case, numbers)
- Chat messages must have content length > 0
- Book content chunks must have content length between 100-2000 characters
- User sessions expire after 30 days of inactivity

## State Transitions
- User Account: active ↔ suspended (admin action)
- Chat Session: active → archived (after 1 year of inactivity)
- User Session Token: valid → expired (by expires_at) or revoked (user logout)