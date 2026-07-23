"""
Configuration module - loads settings from environment variables.
"""
import os
from dotenv import load_dotenv

load_dotenv()

# LLM settings
LLM_API_KEY = os.getenv("LLM_API_KEY", "")
LLM_MODEL = os.getenv("LLM_MODEL", "claude-sonnet-4-20250514")
LLM_PROVIDER = os.getenv("LLM_PROVIDER", "anthropic")  # "anthropic" or "openai"

# Database
DATABASE_PATH = os.getenv("DATABASE_PATH", "data/queue.db")

# Server
HOST = os.getenv("HOST", "127.0.0.1")
PORT = int(os.getenv("PORT", "8000"))
