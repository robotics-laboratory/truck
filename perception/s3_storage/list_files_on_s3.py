from s3_storage.get_s3_conn import get_conn
from typing import List


def get_list_files_in_buckets(buckets: List[str], prefix: str | None = None):
    """
    Выводит содержимое указанных бакетов на S3
    :param buckets: Список имен бакетов
    :param prefix: Префикс (путь внутри бакета)
    """
    s3 = get_conn()

    for bucket in buckets:
        print(f"📂 Содержимое бакета: {bucket}")
        try:
            params = {"Bucket": bucket}
            if prefix:
                params["Prefix"] = prefix

            response = s3.list_objects_v2(**params)

            if "Contents" in response:
                for obj in response["Contents"]:
                    print(f"  - {obj['Key']} (Размер: {obj['Size']} байт)")
            else:
                print("  Бакет пустой.")
        except Exception as e:
            print(f"  ❗ Ошибка при получении содержимого бакета: {e}")

        print("\n" + "=" * 50 + "\n")


if __name__ == "__main__":
    get_list_files_in_buckets(["the-lab-bucket"], prefix="cvat/")
