import os

import boto3
from botocore.exceptions import ClientError
from s3_storage.get_s3_conn import get_conn

# Создаем сессию и клиент S3
session = boto3.session.Session()

s3 = get_conn()

# Параметры
bucket = "the-lab-bucket"
zip_dir = "../output_zip/"
prefix = "cvat/input/"


# Проверка существования файла в S3
def file_exists_in_s3(bucket: str, key: str) -> bool:
    try:
        s3.head_object(Bucket=bucket, Key=key)
        return True
    except ClientError as e:
        if e.response["Error"]["Code"] == "404":
            return False
        else:
            raise


def delete_zip_from_s3(bucket: str, filename: str, prefix: str = "cvat/input/"):
    """
    Удаляет zip-файл из S3 по имени.

    :param bucket: Имя S3 бакета
    :param prefix: Префикс (путь внутри бакета)
    :param filename: Имя файла, включая .zip
    """
    key = os.path.join(prefix, filename)

    try:
        s3.delete_object(Bucket=bucket, Key=key)
        print(f"Файл '{key}' успешно удален из S3.")
    except ClientError as e:
        print(f"Ошибка при удалении файла '{key}': {e}")


def upload_files_to_s3(local_dir: str, bucket_name: str, s3_prefix: str):
    """
    Загружает zip-файлы из локальной директории в S3, если они еще не существуют

    :param local_dir: Путь к локальной директории с zip-файлами
    :param bucket_name: Имя S3 бакета
    :param s3_prefix: Префикс (путь внутри бакета)
    """
    for root, _, files in os.walk(local_dir):
        for file in files:
            if file.endswith(".zip"):
                local_path = os.path.join(root, file)
                s3_path = s3_prefix + file

                if file_exists_in_s3(bucket_name, s3_path):
                    print(
                        f"File {file} already exists in s3://{bucket_name}/{s3_path}. Skipping upload."
                    )
                else:
                    s3.upload_file(local_path, bucket_name, s3_path)
                    print(f"Uploaded {file} to s3://{bucket_name}/{s3_path}")


def upload_file_on_s3(file_path: str, bucket_name: str, s3_prefix: str = "cvat/input/"):
    file_name = os.path.basename(file_path)
    s3_path = s3_prefix + file_name

    if file_exists_in_s3(bucket_name, s3_path):
        print(
            f"File {file_name} already exists in s3://{bucket_name}/{s3_path}. Skipping upload."
        )
    else:
        s3.upload_file(file_path, bucket_name, s3_path)
        print(f"Uploaded {file_name} to s3://{bucket_name}/{s3_path}")


def download_files_from_s3(
    bucket_name: str = "the-lab-bucket", s3_prefix: str = "cvat/input/", file_names=None
):
    """
    Скачивает zip-файлы из S3.

    :param bucket_name: Имя S3 бакета
    :param s3_prefix: Префикс (путь внутри бакета)
    :param file_names: Список имен файлов, которые нужно скачать. При None - скачиваются все файлы
    """
    download_dir = "./downloaded_zip/"
    os.makedirs(download_dir, exist_ok=True)

    objects = s3.list_objects_v2(Bucket=bucket_name, Prefix=s3_prefix)
    if "Contents" not in objects:
        print(f"No files found in s3://{bucket_name}/{s3_prefix}")
        return

    for obj in objects["Contents"]:
        file_key = obj["Key"]
        file_name = os.path.basename(file_key)

        if file_name.endswith(".zip") and (
            file_names is None or file_name in file_names
        ):
            local_path = os.path.join(download_dir, file_name)

            try:
                s3.download_file(bucket_name, file_key, local_path)
                print(f"Downloaded {file_name} to {local_path}")
            except ClientError as e:
                print(f"Failed to download {file_name}: {e}")


if __name__ == "__main__":
    upload_files_to_s3(zip_dir, bucket, prefix)
