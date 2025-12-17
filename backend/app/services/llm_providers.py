"""
LLM Provider Abstraction Layer
Supports multiple LLM providers: OpenAI, Google Gemini, Qwen (Alibaba Cloud)
Supports runtime provider switching without restart
"""
import os
from abc import ABC, abstractmethod
from typing import List, Dict, Any, Optional
from openai import OpenAI


class BaseLLMProvider(ABC):
    """Abstract base class for LLM providers"""

    @abstractmethod
    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding vector for text

        Args:
            text: Input text to embed

        Returns:
            List of floats representing the embedding vector
        """
        pass

    @abstractmethod
    def generate_chat_completion(
        self,
        messages: List[Dict[str, str]],
        max_tokens: int,
        temperature: float
    ) -> str:
        """
        Generate chat completion response

        Args:
            messages: List of message dicts with 'role' and 'content'
            max_tokens: Maximum tokens in response
            temperature: Sampling temperature (0.0-1.0)

        Returns:
            Generated text response
        """
        pass

    @abstractmethod
    def get_embedding_dimension(self) -> int:
        """
        Get the dimension size of embeddings for this provider

        Returns:
            Integer dimension size (e.g., 1536 for OpenAI, 768 for Gemini)
        """
        pass

    @abstractmethod
    def get_provider_name(self) -> str:
        """
        Get the name of this provider

        Returns:
            Provider name string
        """
        pass


class OpenAIProvider(BaseLLMProvider):
    """OpenAI API provider (GPT-4o-mini + text-embedding-3-small)"""

    def __init__(self, api_key: str):
        """
        Initialize OpenAI provider

        Args:
            api_key: OpenAI API key
        """
        self.client = OpenAI(api_key=api_key)
        self.embedding_model = "text-embedding-3-small"
        self.chat_model = "gpt-4o-mini"
        print(f"[OK] OpenAI Provider initialized")
        print(f"   Embedding Model: {self.embedding_model}")
        print(f"   Chat Model: {self.chat_model}")

    def generate_embedding(self, text: str) -> List[float]:
        """Generate 1536-dim embedding using text-embedding-3-small"""
        response = self.client.embeddings.create(
            model=self.embedding_model,
            input=text
        )
        return response.data[0].embedding

    def generate_chat_completion(
        self,
        messages: List[Dict[str, str]],
        max_tokens: int,
        temperature: float
    ) -> str:
        """Generate chat completion using GPT-4o-mini"""
        response = self.client.chat.completions.create(
            model=self.chat_model,
            messages=messages,
            max_tokens=max_tokens,
            temperature=temperature
        )
        return response.choices[0].message.content

    def get_embedding_dimension(self) -> int:
        """OpenAI text-embedding-3-small uses 1536 dimensions"""
        return 1536

    def get_provider_name(self) -> str:
        return "OpenAI"


class GeminiProvider(BaseLLMProvider):
    """Google Gemini API provider (Free tier: 60 req/min, 1500 req/day)"""

    def __init__(self, api_key: str):
        """
        Initialize Gemini provider

        Args:
            api_key: Google AI Studio API key
        """
        try:
            import google.generativeai as genai
            self.genai = genai
            genai.configure(api_key=api_key)

            # Models
            self.embedding_model = "models/text-embedding-004"
            self.chat_model = "gemini-1.5-flash"

            print(f"[OK] Gemini Provider initialized")
            print(f"   Embedding Model: {self.embedding_model}")
            print(f"   Chat Model: {self.chat_model}")

        except ImportError:
            raise ImportError(
                "Google Generative AI package not installed. "
                "Install with: pip install google-generativeai"
            )

    def generate_embedding(self, text: str) -> List[float]:
        """Generate 768-dim embedding using text-embedding-004"""
        result = self.genai.embed_content(
            model=self.embedding_model,
            content=text,
            task_type="retrieval_document"
        )
        return result['embedding']

    def generate_chat_completion(
        self,
        messages: List[Dict[str, str]],
        max_tokens: int,
        temperature: float
    ) -> str:
        """Generate chat completion using Gemini 1.5 Flash"""
        # Convert OpenAI message format to Gemini format
        prompt = self._convert_messages_to_prompt(messages)

        model = self.genai.GenerativeModel(self.chat_model)
        response = model.generate_content(
            prompt,
            generation_config={
                'max_output_tokens': max_tokens,
                'temperature': temperature
            }
        )
        return response.text

    def _convert_messages_to_prompt(self, messages: List[Dict[str, str]]) -> str:
        """
        Convert OpenAI message format to Gemini single prompt

        Args:
            messages: List of dicts with 'role' and 'content'

        Returns:
            Combined prompt string
        """
        prompt_parts = []
        for msg in messages:
            role = msg['role']
            content = msg['content']

            if role == 'system':
                prompt_parts.append(f"INSTRUCTIONS:\n{content}\n")
            elif role == 'user':
                prompt_parts.append(f"USER:\n{content}\n")
            elif role == 'assistant':
                prompt_parts.append(f"ASSISTANT:\n{content}\n")

        return "\n".join(prompt_parts)

    def get_embedding_dimension(self) -> int:
        """Gemini text-embedding-004 uses 768 dimensions"""
        return 768

    def get_provider_name(self) -> str:
        return "Google Gemini"


class QwenProvider(BaseLLMProvider):
    """Alibaba Cloud Qwen API provider"""

    def __init__(self, api_key: str):
        """
        Initialize Qwen provider

        Args:
            api_key: Alibaba Cloud DashScope API key
        """
        try:
            import dashscope
            self.dashscope = dashscope
            dashscope.api_key = api_key

            # Models
            self.embedding_model = "text-embedding-v3"
            self.chat_model = "qwen-turbo"

            print(f"[OK] Qwen Provider initialized")
            print(f"   Embedding Model: {self.embedding_model}")
            print(f"   Chat Model: {self.chat_model}")

        except ImportError:
            raise ImportError(
                "DashScope package not installed. "
                "Install with: pip install dashscope"
            )

    def generate_embedding(self, text: str) -> List[float]:
        """Generate embedding using Qwen text-embedding-v3"""
        from dashscope import TextEmbedding

        response = TextEmbedding.call(
            model=self.embedding_model,
            input=text
        )

        if response.status_code == 200:
            return response.output['embeddings'][0]['embedding']
        else:
            raise Exception(f"Qwen embedding failed: {response.message}")

    def generate_chat_completion(
        self,
        messages: List[Dict[str, str]],
        max_tokens: int,
        temperature: float
    ) -> str:
        """Generate chat completion using Qwen-turbo"""
        from dashscope import Generation

        response = Generation.call(
            model=self.chat_model,
            messages=messages,  # Qwen uses same format as OpenAI
            max_tokens=max_tokens,
            temperature=temperature,
            result_format='message'
        )

        if response.status_code == 200:
            return response.output['choices'][0]['message']['content']
        else:
            raise Exception(f"Qwen chat completion failed: {response.message}")

    def get_embedding_dimension(self) -> int:
        """Qwen text-embedding-v3 uses 1024 dimensions"""
        return 1024

    def get_provider_name(self) -> str:
        return "Alibaba Qwen"


# Provider cache for runtime switching
_provider_cache: Dict[str, BaseLLMProvider] = {}


def get_llm_provider(provider_name: Optional[str] = None) -> BaseLLMProvider:
    """
    Factory function to get LLM provider based on parameter or environment variable
    Caches providers for efficient runtime switching

    Args:
        provider_name: Optional provider name ('openai', 'gemini', 'qwen')
                      If None, uses LLM_PROVIDER environment variable

    Environment Variables:
        LLM_PROVIDER: Default provider name (used when provider_name is None)
        OPENAI_API_KEY: OpenAI API key (if using OpenAI)
        GEMINI_API_KEY: Google AI Studio API key (if using Gemini)
        QWEN_API_KEY: Alibaba Cloud API key (if using Qwen)

    Returns:
        Initialized LLM provider instance (cached for reuse)

    Raises:
        ValueError: If provider name is invalid or API key is missing
    """
    # Use parameter or fall back to environment variable
    if provider_name is None:
        provider_name = os.getenv("LLM_PROVIDER", "openai")

    provider_name = provider_name.lower()

    # Return cached provider if available
    if provider_name in _provider_cache:
        return _provider_cache[provider_name]

    print(f"\n{'='*60}")
    print(f"[INIT] Initializing LLM Provider: {provider_name.upper()}")
    print(f"{'='*60}")

    # Create new provider instance
    provider = None

    if provider_name == "openai":
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise ValueError(
                "OPENAI_API_KEY environment variable not set. "
                "Get your key from: https://platform.openai.com/api-keys"
            )
        provider = OpenAIProvider(api_key)

    elif provider_name == "gemini":
        api_key = os.getenv("GEMINI_API_KEY")
        if not api_key:
            raise ValueError(
                "GEMINI_API_KEY environment variable not set. "
                "Get your free key from: https://ai.google.dev/"
            )
        provider = GeminiProvider(api_key)

    elif provider_name == "qwen":
        api_key = os.getenv("QWEN_API_KEY")
        if not api_key:
            raise ValueError(
                "QWEN_API_KEY environment variable not set. "
                "Get your key from: https://dashscope.console.aliyun.com/"
            )
        provider = QwenProvider(api_key)

    else:
        raise ValueError(
            f"Unknown LLM provider: '{provider_name}'. "
            f"Supported providers: 'openai', 'gemini', 'qwen'"
        )

    # Cache the provider for future use
    _provider_cache[provider_name] = provider
    return provider
