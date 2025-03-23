from langchain_nvidia_ai_endpoints import ChatNVIDIA
from utils.chat_model import BaseChatModel

class ChatNVIDIAModel(BaseChatModel):
    def __init__(self, api_key: str):
        self.model = ChatNVIDIA(nvidia_api_key=api_key)

    def invoke(self, prompt: str) -> str:
        return self.model.invoke(prompt)