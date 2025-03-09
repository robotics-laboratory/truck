import requests
import os

from private_config import private_settings

CVAT_API_URL = f"{private_settings.CVAT_HOST}/api"
OUTPUT_DIR = "output_annotations"


def get_auth_token(username: str, password: str):
    response = requests.post(
        f"{CVAT_API_URL}/auth/login",
        json={"username": username, "password": password},
    )

    if response.status_code == 200:
        return response.cookies
    else:
        raise Exception(
            f"Failed to get auth token: {response.status_code}, {response.text}"
        )


def download_annotations(job_id: str, cookies):
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


def save_annotations(job_id: str, annotations):
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    file_path = os.path.join(OUTPUT_DIR, f"job_{job_id}_annotations.json")

    with open(file_path, "w", encoding="utf-8") as f:
        import json

        json.dump(annotations, f, ensure_ascii=False, indent=4)

    print(f"Annotations saved to {file_path}")


if __name__ == "__main__":
    username = private_settings.CVAT_LOGIN
    password = private_settings.CVAT_PASSWORD
    job_id = input("Enter CVAT job ID: ")

    try:
        cookies = get_auth_token(username, password)
        annotations = download_annotations(job_id, cookies)
        save_annotations(job_id, annotations)
    except Exception as e:
        print(f"Error: {e}")
