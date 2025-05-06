# 🚀 Data Preparation Pipeline for CVAT Annotation

## Project Structure

```tree
perception
├── data                        # папка с исходными данными
│   ├── run_name
│   │   ├── config.yaml
│   │   ├── run_name.mcap
│   │   ├── run_name.insv
│   │   ├── run_name-0.mp4
│   │   ├── run_name-90.mp4
│   │   ├── run_name-180.mp4
│   │   └── run_name-270.mp4
├── output_zip                  # папка с собранными данными для разметки
│   └── run_name.zip
├── cvat_api
│   └── get_annotations.py      # получение разметки с CVAT
├── pcd_utils
│   └── mcap_to_pcd.py          # конвертация mcap в pcd
├── s3_storage
│   └── zip_on_s3.py            # загрузка архива на S3
├── frames_from_mp4.py          # нарезка видео на кадры
├── pipeline.py                 # полный пайплайн
├── config.py                   # получение параметров конфигурации из config.yaml
└── privat_config.py            # получение чувствительных данных из переменных окружения
└── .env
```

## Output Structure

```tree
run_name.zip
├── pointcloud
│   ├── run_name-000.pcd
...
├── related_images
│   ├── run_name-000_pcd
│   │   ├── run_name-000-0.jpg
│   │   ├── run_name-000-90.jpg
│   │   ├── run_name-000-180.jpg
│   │   └── run_name-000-270.jpg
...

```

## 🛠️ Module Overview

### `pcd_utils/mcap_to_pcd.py`
Converts `.mcap` files (ROS2 format) into `.pcd` point cloud frames based on a configured frequency. Supports time alignment with associated videos.

### `cvat_api/get_annotations.py`
Fetches annotation data from CVAT via its API using a specified `job_id`. Simplifies CVAT integration.

### `s3_storage/zip_on_s3.py`
Handles upload and download of ZIP archives to/from AWS S3, used for transferring datasets between environments or for CVAT access.

### `frames_from_mp4.py`
Extracts frames from each of the four perspective videos (`0`, `90`, `180`, `270` degrees) and organizes them for annotation.

### `pipeline.py`
End-to-end automation for preparing a dataset:
1. Extracts frames from the input videos.
2. Converts `.mcap` files to point cloud data.
3. Aligns point clouds and images in time.
4. Packages all data into a structured ZIP archive.
5. Archive can be uploaded to S3 for annotation workflows.

### Configuration

#### `privat_config.py`
Retrieves sensitive credentials from environment variables or `.env` file:

- `AWS_ACCESS_KEY_ID` — AWS S3 Access Key  
- `AWS_SECRET_ACCESS_KEY` — AWS S3 Secret Key  
- `CVAT_LOGIN` — CVAT username  
- `CVAT_PASSWORD` — CVAT password  
- `CVAT_HOST` — URL of the CVAT server

#### `config.yaml`

- `frequency`: Sampling rate for both video frames and point cloud extraction  
- `mcap_elapsed`: Time offset (in seconds) to align MCAP data  
- `video_elapsed`: Time offset (in seconds) to align video frames

---

## ⚡ Quick Start Guide

1. Install the required dependencies:
```bash
pip install -r requirements.txt
```
2. Configure your .env file and make sure config.yaml is properly filled.

3. Place the input files into the data/{run_name} directory:

    - 4 video files: {run_name}-0.mp4, -90.mp4, -180.mp4, and -270.mp4
    - {run_name}.mcap
    - {run_name}.insv optionally

4. Run the pipeline:

```bash
python pipeline.py {run_name}
```