import os

import s3fs
from private_config import private_settings

# Connect to S3 using s3fs
fs = s3fs.S3FileSystem(
    key=private_settings.AWS_ACCESS_KEY_ID,
    secret=private_settings.AWS_SECRET_ACCESS_KEY,
    client_kwargs={"endpoint_url": "https://storage.yandexcloud.net"},
)

bucket = "the-lab-bucket"
zip_dir = "../output_zip/"
prefix = "cvat/input/"
s3_base_path = f"{bucket}/{prefix}".rstrip("/")


def file_exists_in_s3(bucket: str, key: str) -> bool:
    """
    Check if a file exists in the specified S3 bucket.

    :param bucket: S3 bucket name.
    :param key: Path to the file within the bucket.
    :return: True if the file exists, False otherwise.
    """
    s3_path = f"{bucket}/{key}"
    return fs.exists(s3_path)


def delete_zip_from_s3(bucket: str, filename: str, prefix: str = "cvat/input/"):
    """
    Delete a .zip file from S3.

    :param bucket: S3 bucket name.
    :param filename: Name of the zip file to delete.
    :param prefix: Path prefix within the bucket.
    """
    s3_path = f"{bucket}/{prefix}{filename}"
    try:
        fs.rm(s3_path)
        print(f"File '{s3_path}' was successfully deleted from S3.")
    except Exception as e:
        print(f"Error deleting file '{s3_path}': {e}")


def upload_zip_on_s3(local_dir: str, bucket_name: str, s3_prefix: str):
    """
    Upload all .zip files from a local directory to a specific S3 prefix.

    :param local_dir: Local directory to search for .zip files.
    :param bucket_name: S3 bucket name.
    :param s3_prefix: Path prefix within the bucket to upload to.
    """
    for root, _, files in os.walk(local_dir):
        for filename in files:
            if filename.endswith(".zip"):
                local_path = os.path.join(root, filename)
                s3_path = f"{bucket_name}/{s3_prefix}{filename}"

                if fs.exists(s3_path):
                    print(
                        f"File {filename} is already uploaded to s3://{s3_path}. "
                        f"Skipping."
                    )
                else:
                    fs.put(local_path, s3_path)
                    print(f"Uploaded {filename} to s3://{s3_path}")


def upload_file_on_s3(file_path: str, bucket_name: str, s3_prefix: str = "cvat/input/"):
    """
    Upload a single file to S3.

    :param file_path: Path to the local file.
    :param bucket_name: S3 bucket name.
    :param s3_prefix: Path prefix within the bucket.
    """
    filename = os.path.basename(file_path)
    s3_path = f"{bucket_name}/{s3_prefix}{filename}"

    if fs.exists(s3_path):
        print(f"File {filename} is already uploaded to s3://{s3_path}. Skipping.")
    else:
        fs.put(file_path, s3_path)
        print(f"{filename} uploaded to s3://{s3_path}")


def download_files_from_s3(
    bucket_name: str = "the-lab-bucket", s3_prefix: str = "cvat/input/", file_names=None
):
    """
    Download specified .zip files from S3 to the local 'downloaded_zip' directory.

    :param bucket_name: S3 bucket name.
    :param s3_prefix: Path prefix within the bucket.
    :param file_names: List of filenames to download. If None, download all .zip files.
    """
    download_dir = "./downloaded_zip/"
    os.makedirs(download_dir, exist_ok=True)

    s3_path_prefix = f"{bucket_name}/{s3_prefix}"
    try:
        files = fs.ls(s3_path_prefix)
    except FileNotFoundError:
        print(f"No files found in s3://{s3_path_prefix}")
        return

    for file in files:
        file_name = os.path.basename(file)

        if file_name.endswith(".zip") and (
            file_names is None or file_name in file_names
        ):
            local_path = os.path.join(download_dir, file_name)

            try:
                fs.get(file, local_path)
                print(f"Downloaded {file_name} to {local_path}")
            except Exception as e:
                print(f"Failed to download {file_name}: {e}")


if __name__ == "__main__":
    upload_zip_on_s3(zip_dir, bucket, prefix)
