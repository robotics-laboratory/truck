from s3_storage.get_s3_conn import get_conn


def list_files_in_buckets(buckets, prefix=None):
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


# Пример использования:
list_files_in_buckets(["the-lab-bucket"], prefix="cvat/")
