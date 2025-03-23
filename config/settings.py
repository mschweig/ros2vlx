import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

class Settings:
    # Chat model configuration
    CHAT_MODEL = os.getenv("CHAT_MODEL", "AzureChatOpenAI")  # Default to AzureChatOpenAI

    # NVIDIA AI Endpoints
    NVIDIA_API_KEY = os.getenv("NVIDIA_API_KEY")

    # OpenAI
    OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

    # Azure OpenAI
    AZURE_OPENAI_API_VERSION=os.getenv("AZURE_OPENAI_API_VERSION")
    AZURE_OPENAI_TEMPERATURE=os.getenv("AZURE_OPENAI_TEMPERATURE")
    AZURE_DEPLOYMENT_MODEL_NAME=os.getenv("AZURE_DEPLOYMENT_MODEL_NAME")

settings = Settings()