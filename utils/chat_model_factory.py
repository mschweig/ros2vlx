from utils.chat_models.chat_nvidia import ChatNVIDIAModel
from utils.chat_models.chat_openai import ChatOpenAIModel
from utils.chat_models.chat_azure import ChatAzureModel
from config.settings import settings

class ChatModelFactory:
    @staticmethod
    def get_chat_model(model_name: str):
        """
        Returns the appropriate chat model based on the configuration.
        
        Args:
            model_name (str): The name of the chat model to use.
        
        Returns:
            BaseChatModel: The chat model instance.
        """
        if model_name == "AzureChatOpenAI":
            return ChatAzureModel.register_and_get_model(
                deployment_model_name=settings.AZURE_DEPLOYMENT_MODEL_NAME,
                azure_api_version=settings.AZURE_OPENAI_API_VERSION,
                temperature=settings.AZURE_OPENAI_TEMPERATURE
            )
        else:
            raise ValueError(f"Unsupported chat model: {model_name}")