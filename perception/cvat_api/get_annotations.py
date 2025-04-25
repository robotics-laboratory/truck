import os

import requests
from private_config import private_settings

CVAT_API_URL = f"{private_settings.CVAT_HOST}/api"
OUTPUT_DIR = "output_annotations"


def get_auth_token(cvat_username: str, cvat_password: str):
    """
    Получить токен авторизации в CVAT
    :param cvat_username: Имя пользователя в CVAT
    :param cvat_password: Пароль пользователя в CVAT
    """
    response = requests.post(
        f"{CVAT_API_URL}/auth/login",
        json={"username": cvat_username, "password": cvat_password},
    )

    if response.status_code == 200:
        return response.cookies
    else:
        raise Exception(
            f"Failed to get auth token: {response.status_code}, {response.text}"
        )


def download_annotations(job_id: str, cookies) -> dict:
    """
    Скачать разметку из CVAT по job_id
    :param job_id: ID задачи на CVAT
    :param cookies: Токен авторизации в CVAT
    """
    response = requests.get(
        f"{CVAT_API_URL}/jobs/{job_id}/annotations/",
        cookies=cookies,
        headers={"Accept": "application/vnd.cvat+json"},
    )

    if response.status_code == 200:
        return response.json()
    else:
        raise Exception(
            f"Failed to download annotations: {response.status_code}, {response.text}"
        )


def save_annotations(job_id: str, annotations: dict):
    """
    Сохранить разметку в JSON-файл
    :param job_id: ID задачи на CVAT
    :param annotations: Разметка
    """
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    file_path = os.path.join(OUTPUT_DIR, f"job_{job_id}_annotations.json")

    with open(file_path, "w", encoding="utf-8") as f:
        import json

        json.dump(annotations, f, ensure_ascii=False, indent=4)

    print(f"Annotations saved to {file_path}")


if __name__ == "__main__":
    username = private_settings.CVAT_LOGIN
    password = private_settings.CVAT_PASSWORD
    cvat_job_id = input("Enter CVAT job ID: ")

    try:
        cookies = get_auth_token(username, password)
        job_annotations = download_annotations(cvat_job_id, cookies)
        save_annotations(cvat_job_id, job_annotations)
    except Exception as e:
        print(f"Error: {e}")
