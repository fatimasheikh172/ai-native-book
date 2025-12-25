# Data Model: Physical AI and Human-Aided Robotics Book

**Date**: 2025-12-15
**Feature**: 1-physical-ai-book

## Entity: User

**Description**: Represents a system user with role-based access
**Fields**:
- id: UUID (primary key)
- email: String (unique, required)
- password_hash: String (required for registered users)
- role: Enum (student, educator, researcher) (required)
- created_at: DateTime (required)
- updated_at: DateTime (required)
- last_login: DateTime (nullable)

**Relationships**:
- One-to-many with UserProgress
- One-to-many with AssessmentAttempt
- One-to-many with OfflineContentDownload

**Validation Rules**:
- Email must be valid email format
- Role must be one of allowed values
- Password must meet security requirements

## Entity: Module

**Description**: A distinct learning unit containing content focused on a specific aspect of Physical AI and Robotics
**Fields**:
- id: UUID (primary key)
- title: String (required)
- description: Text (required)
- module_number: Integer (required, unique)
- prerequisites: Array of Module IDs (optional)
- estimated_duration: Integer (minutes, required)
- content_url: String (required)
- created_at: DateTime (required)
- updated_at: DateTime (required)

**Relationships**:
- One-to-many with UserProgress
- One-to-many with Assessment

**Validation Rules**:
- Module number must be unique
- Title and description must not be empty
- Estimated duration must be positive

## Entity: Curriculum

**Description**: The organized sequence of modules forming the complete learning experience
**Fields**:
- id: UUID (primary key)
- title: String (required)
- description: Text (required)
- modules: Array of Module IDs (required)
- total_duration: Integer (minutes, calculated)
- created_at: DateTime (required)
- updated_at: DateTime (required)

**Relationships**:
- One-to-many with UserProgress
- Contains many Modules

**Validation Rules**:
- Title must not be empty
- Must contain at least one module
- Module IDs must reference existing modules

## Entity: UserProgress

**Description**: Tracks individual user progress through the curriculum
**Fields**:
- id: UUID (primary key)
- user_id: UUID (foreign key to User, required)
- module_id: UUID (foreign key to Module, required)
- status: Enum (not_started, in_progress, completed) (required)
- progress_percentage: Integer (0-100, required)
- time_spent: Integer (seconds, required)
- last_accessed: DateTime (required)
- completed_at: DateTime (nullable)
- created_at: DateTime (required)
- updated_at: DateTime (required)

**Relationships**:
- Many-to-one with User
- Many-to-one with Module

**Validation Rules**:
- User and Module must exist
- Progress percentage must be between 0-100
- Status must be one of allowed values

## Entity: Assessment

**Description**: Evaluation tools including quizzes, tests, and assignments with grading capabilities
**Fields**:
- id: UUID (primary key)
- module_id: UUID (foreign key to Module, required)
- title: String (required)
- description: Text (required)
- assessment_type: Enum (quiz, assignment, exam) (required)
- questions: Array of Question objects (required)
- max_score: Integer (required)
- time_limit: Integer (seconds, nullable)
- created_at: DateTime (required)
- updated_at: DateTime (required)

**Relationships**:
- Many-to-one with Module
- One-to-many with AssessmentAttempt

**Validation Rules**:
- Module must exist
- Questions array must not be empty
- Max score must be positive

## Entity: AssessmentAttempt

**Description**: Represents a user's attempt at an assessment
**Fields**:
- id: UUID (primary key)
- user_id: UUID (foreign key to User, required)
- assessment_id: UUID (foreign key to Assessment, required)
- answers: Array of Answer objects (required)
- score: Integer (nullable)
- max_score: Integer (required)
- status: Enum (in_progress, submitted, graded) (required)
- started_at: DateTime (required)
- submitted_at: DateTime (nullable)
- graded_at: DateTime (nullable)
- created_at: DateTime (required)
- updated_at: DateTime (required)

**Relationships**:
- Many-to-one with User
- Many-to-one with Assessment

**Validation Rules**:
- User and Assessment must exist
- Score must not exceed max_score
- Status transitions must follow proper sequence

## Entity: Question

**Description**: Individual question within an assessment
**Fields**:
- id: UUID (primary key)
- assessment_id: UUID (foreign key to Assessment, required)
- question_text: Text (required)
- question_type: Enum (multiple_choice, short_answer, essay) (required)
- options: Array of String (for multiple choice, optional)
- correct_answer: String or Array (required)
- points: Integer (required)
- order: Integer (required)

**Relationships**:
- Many-to-one with Assessment

**Validation Rules**:
- Assessment must exist
- Points must be positive
- For multiple choice, options must exist and be non-empty

## Entity: LearningPath

**Description**: The sequential progression through modules designed to build knowledge systematically
**Fields**:
- id: UUID (primary key)
- name: String (required)
- description: Text (required)
- modules: Array of Module IDs in order (required)
- user_id: UUID (foreign key to User, nullable - for personalized paths)
- is_default: Boolean (required)
- created_at: DateTime (required)
- updated_at: DateTime (required)

**Relationships**:
- Many-to-one with User (optional, for personalized paths)
- Contains many Modules

**Validation Rules**:
- Module IDs must reference existing modules
- At least one module required

## Entity: TechnicalDiagram

**Description**: Visual representations of system architectures, component relationships, and data flows
**Fields**:
- id: UUID (primary key)
- title: String (required)
- module_id: UUID (foreign key to Module, required)
- diagram_type: Enum (mermaid, svg, custom) (required)
- content: Text or JSON (required)
- description: Text (required)
- created_at: DateTime (required)
- updated_at: DateTime (required)

**Relationships**:
- Many-to-one with Module

**Validation Rules**:
- Module must exist
- Content must not be empty
- Diagram type must be one of allowed values

## Entity: OfflineContent

**Description**: Content available for offline access by registered users
**Fields**:
- id: UUID (primary key)
- module_id: UUID (foreign key to Module, required)
- content_type: Enum (text, image, video, pdf) (required)
- content_url: String (required)
- file_size: Integer (bytes, required)
- download_count: Integer (required, default 0)
- created_at: DateTime (required)
- updated_at: DateTime (required)

**Relationships**:
- Many-to-one with Module
- One-to-many with OfflineContentDownload

**Validation Rules**:
- Module must exist
- File size must be positive
- Content URL must be valid

## Entity: OfflineContentDownload

**Description**: Tracks user downloads of offline content
**Fields**:
- id: UUID (primary key)
- user_id: UUID (foreign key to User, required)
- offline_content_id: UUID (foreign key to OfflineContent, required)
- download_time: DateTime (required)
- expires_at: DateTime (required)

**Relationships**:
- Many-to-one with User
- Many-to-one with OfflineContent

**Validation Rules**:
- User and OfflineContent must exist
- Expires_at must be in the future