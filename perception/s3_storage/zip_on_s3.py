import os

from boto3.s3.transfer import TransferConfig
from botocore.exceptions import ClientError

from private_config import private_settings
from s3_storage.get_s3_conn import get_conn


s3 = get_conn()

# Параметры
bucket_name = "the-lab-bucket"
local_dir = "../output_zip/"
s3_prefix = "cvat/input/"


def download_files_from_s3(
    bucket_name="the-lab-bucket", s3_prefix="cvat/input/", file_names=None
):
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


# Проверка существования файла в S3
def file_exists_in_s3(bucket, key):
    try:
        s3.head_object(Bucket=bucket, Key=key)
        return True
    except ClientError as e:
        if e.response["Error"]["Code"] == "404":
            return False
        else:
            raise


# Загружаем файлы в S3 только если их нет


def upload_files_to_s3(local_dir, bucket_name, s3_prefix):
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


def download_files_from_s3(
    bucket_name="the-lab-bucket", s3_prefix="cvat/input/", file_names=None
):
    download_dir = "./downloaded_zip/"
    os.makedirs(download_dir, exist_ok=True)

    objects = s3.list_objects_v2(Bucket=bucket_name, Prefix=s3_prefix)
    if "Contents" not in objects:
        print(f"No files found in s3://{bucket_name}/{s3_prefix}")
        return

# Загружаем файлы
upload_files_to_s3(local_dir, bucket_name, s3_prefix)
