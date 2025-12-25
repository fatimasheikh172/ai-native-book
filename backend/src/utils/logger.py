import logging
import sys
from datetime import datetime
from typing import Any, Dict
from src.config.settings import settings


def setup_logger(name: str, log_file: str = None, level: int = None) -> logging.Logger:
    """
    Function to setup a logger with standardized configuration
    """
    if level is None:
        level = logging.DEBUG if settings.debug else logging.INFO

    # Create logger
    logger = logging.getLogger(name)
    logger.setLevel(level)

    # Prevent adding multiple handlers if logger already exists
    if logger.handlers:
        return logger

    # Create console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(level)

    # Create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    console_handler.setFormatter(formatter)

    # Add handlers to logger
    logger.addHandler(console_handler)

    # Also add file handler if log_file is specified
    if log_file:
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(level)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

    return logger


def log_api_call(user_id: str, endpoint: str, method: str, response_time: float,
                 status_code: int, request_data: Dict[str, Any] = None) -> None:
    """
    Log API calls for monitoring and analytics
    """
    logger = setup_logger("api_logger", "logs/api.log")
    logger.info(f"API Call - User: {user_id}, Endpoint: {endpoint}, "
                f"Method: {method}, Response Time: {response_time}s, "
                f"Status: {status_code}")


def log_error(error: Exception, context: str = "", user_id: str = None) -> None:
    """
    Log errors with context information
    """
    logger = setup_logger("error_logger", "logs/error.log")
    logger.error(f"Error in {context} - User: {user_id}, Error: {str(error)}, "
                 f"Type: {type(error).__name__}, Time: {datetime.now()}")