import boto3
from private_config import private_settings


def get_conn() -> boto3.client:
    """
    Создание сессии и клиента S3
    """
    session = boto3.session.Session()

    s3 = session.client(
        service_name="s3",
        endpoint_url="https://storage.yandexcloud.net",
        aws_access_key_id=private_settings.AWS_ACCESS_KEY_ID,
        aws_secret_access_key=private_settings.AWS_SECRET_ACCESS_KEY,
    )

    return s3
