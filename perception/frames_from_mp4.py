import os
from typing import List

import cv2
from config import Config


def extract_frames_by_frequency(
    video_paths: List[str], frequency: int, root_output_dir: str
):
    """
    Извлекает кадры из видео с четырех ракурсов с заданной частотой
    и сохраняет их как изображения

    :param video_paths: Список путей к видеофайлам.
    :param frequency: Частота извлечения кадров в секундах.
    :param root_output_dir: Корневая директория для сохранения изображений.
    """
    # Проверяем наличие корневой директории и создаём её, если нужно
    if not os.path.exists(root_output_dir):
        os.makedirs(root_output_dir)

    # Используем название запуска (например, "run-7") из пути для группировки
    launch_name = os.path.basename(root_output_dir)

    # Проходим по каждому видео
    for video_path in video_paths:
        # Определяем угол обзора (0, 90, 180, 270) из имени файла
        angle = os.path.splitext(os.path.basename(video_path))[0].split("-")[-1]

        # Открываем видеофайл
        cap = cv2.VideoCapture(video_path)

        if not cap.isOpened():
            print(f"Не удалось открыть видеофайл: {video_path}")
            continue

        # Получаем частоту кадров (FPS)
        fps = cap.get(cv2.CAP_PROP_FPS)
        if fps == 0:
            print(f"Не удалось определить FPS видео: {video_path}")
            cap.release()
            continue

        # Расчет временных меток кадров, которые нужно извлечь
        video_duration = cap.get(cv2.CAP_PROP_FRAME_COUNT) / cap.get(cv2.CAP_PROP_FPS)
        elapsed_times = [i for i in range(0, int(video_duration), frequency)]

        # Извлекаем кадры для каждого времени
        for elapsed_time in elapsed_times:
            # Определяем директорию для сохранения
            sub_dir_name = (
                f"{launch_name}-{str(elapsed_time // frequency).zfill(3)}_pcd"
            )
            output_dir = os.path.join(root_output_dir, "related_images", sub_dir_name)

            if not os.path.exists(output_dir):
                os.makedirs(output_dir)

            # Рассчитываем целевой кадр
            frame_number = int(elapsed_time * fps)

            # Переходим к нужному кадру
            cap.set(cv2.CAP_PROP_POS_FRAMES, frame_number)

            # Читаем кадр
            ret, frame = cap.read()
            if not ret:
                print(
                    f"Не удалось извлечь кадр на времени {elapsed_time} сек. "
                    f"из {video_path}"
                )
                continue

            # Формируем имя для файла
            output_file_name = (
                f"{launch_name}-{str(elapsed_time // frequency).zfill(3)}-{angle}.jpg"
            )
            output_path = os.path.join(output_dir, output_file_name)

            # Сохраняем кадр в формате JPG
            cv2.imwrite(output_path, frame)
            print(
                f"Кадр на {elapsed_time} сек. из {video_path} успешно сохранен: "
                f"{output_path}"
            )

        # Освобождаем ресурсы
        cap.release()


def get_frames_from_mp4_main():
    run_name = Config.run_name
    video_files = [
        f"input_mp4/{run_name}-0.mp4",
        f"input_mp4/{run_name}-90.mp4",
        f"input_mp4/{run_name}-180.mp4",
        f"input_mp4/{run_name}-270.mp4",
    ]
    frequency = Config.frequency  # Частота извлечения кадров в секундах
    root_output_directory = run_name  # Корневая директория для сохранения изображений

    extract_frames_by_frequency(video_files, frequency, root_output_directory)


if __name__ == "__main__":
    get_frames_from_mp4_main()
