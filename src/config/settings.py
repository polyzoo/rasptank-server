from typing import final

from pydantic import Field
from pydantic_settings import BaseSettings


@final
class Settings(BaseSettings):
    """Основные настройки приложения."""

    app_host: str = Field(..., alias="APP_HOST")
    app_port: int = Field(..., alias="APP_PORT")

    class Config:
        env_file: str = ".env"
        env_file_encoding: str = "utf-8"
        extra: str = "ignore"
