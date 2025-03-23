from langchain_openai import ChatOpenAI
from utils.chat_model import BaseChatModel

class ChatOpenAIModel(BaseChatModel):
    def __init__(self, api_key: str):
        self.model = ChatOpenAI(api_key=api_key)

    def invoke(self, prompt: str) -> str:
        return self.model.invoke(prompt)