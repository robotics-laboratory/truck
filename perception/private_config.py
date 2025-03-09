import os
from pathlib import Path
from pydantic_settings import BaseSettings, SettingsConfigDict


class PrivateConfig(BaseSettings):
    # Получаем секретную информацию из .env файла или переменных среды
    model_config = SettingsConfigDict(
        env_file=(Path(__file__).parent / ".env" if not os.getenv("USE_ENV") else None),
        extra="ignore",
    )

    AWS_ACCESS_KEY_ID: str
    AWS_SECRET_ACCESS_KEY: str
    CVAT_LOGIN: str
    CVAT_PASSWORD: str
    CVAT_HOST: str


private_settings = PrivateConfig()
