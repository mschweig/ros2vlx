from abc import ABC, abstractmethod

class BaseChatModel(ABC):
    """Base class for all chat models."""

    @abstractmethod
    def invoke(self, prompt: str) -> str:
        """
        Invoke the chat model with a given prompt.
        
        Args:
            prompt (str): The input prompt.
        
        Returns:
            str: The model's response.
        """
        pass

    @abstractmethod
    def register_and_get_model(self):
        """
        Register and get the chat model.
        
        Returns:
            LangChainChatModel: The chat model instance.
        """
        pass