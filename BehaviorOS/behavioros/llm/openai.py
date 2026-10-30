"""
OpenAI client implementation.
"""

import os
from typing import Optional

from openai import AsyncOpenAI

from .base import BaseLLMClient


class OpenAIClient(BaseLLMClient):
    """OpenAI client for API calls."""

    def __init__(self, model: str = "gpt-4o-mini", api_key: Optional[str] = None):
        """Initialize OpenAI client."""
        api_key = api_key or os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise ValueError(
                "OpenAI API key required. Set OPENAI_API_KEY or pass api_key."
            )

        self.client = AsyncOpenAI(api_key=api_key)
        self.model = model

    async def generate(
        self, prompt: str, system_prompt: Optional[str] = None, **kwargs
    ) -> str:
        """Generate response from OpenAI API."""
        messages = []

        if system_prompt:
            messages.append({"role": "system", "content": system_prompt})

        messages.append({"role": "user", "content": prompt})

        # Default to JSON format for BehaviorOS use case
        kwargs.setdefault("response_format", {"type": "json_object"})

        response = await self.client.chat.completions.create(
            model=self.model, messages=messages, **kwargs
        )

        return response.choices[0].message.content
