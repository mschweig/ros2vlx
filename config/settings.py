import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

class Settings:
    # NVIDIA AI Endpoints
    NVIDIA_API_KEY = os.getenv("NVIDIA_API_KEY")

    # OpenAI
    OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

settings = Settings()