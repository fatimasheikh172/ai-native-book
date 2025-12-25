# AI Native Book Backend

This is the backend API for the Physical AI and Human-Aided Robotics Book project.

## Setup

1. Install dependencies:
```bash
npm install
```

2. Create a `.env` file based on the example and add your configuration

3. Start the development server:
```bash
npm run dev
```

## API Endpoints

- `GET /api/users` - Get all users
- `POST /api/users/add` - Add a new user
- `GET /api/users/:id` - Get user by ID
- `POST /api/users/update/:id` - Update user
- `DELETE /api/users/delete/:id` - Delete user

## Technologies Used

- Node.js
- Express.js
- MongoDB/Mongoose
- JSON Web Tokens (JWT)
- Bcrypt for password hashing