from langchain_openai import AzureChatOpenAI
from utils.chat_model import BaseChatModel

class ChatAzureModel(BaseChatModel):
    def __init__(self):
        pass
    
    def register_and_get_model(deployment_model_name, azure_api_version, temperature):
        return AzureChatOpenAI(
            azure_deployment=deployment_model_name,
            api_version=azure_api_version,
            temperature=temperature
        )

    def invoke(self, prompt: str) -> str:
        return self.model.invoke(prompt)