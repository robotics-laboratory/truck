from datetime import datetime, timedelta, timezone
import yaml
import pytz


def get_timestamp(start_time_string: str, elapsed_time: float) -> float:
    """
    Получает метку времени в формате UNIX timestamp и возвращает время в секундах с учетом "сдвига" elapsed_time
    :param start_time_string: строка с временем в формате "YYYY-MM-DD HH:mm:ss.ffffff AM
    В таком формате отображается дата начала записи в Foxglove
    :param elapsed_time: сдвиг времени в секундах
    """

    start_time_msk = datetime.strptime(start_time_string, "%Y-%m-%d %I:%M:%S.%f %p")
    start_time_utc = start_time_msk

    # Вычисляем целевую временную метку в UTC с учетом сдвига
    target_time_utc = start_time_utc + timedelta(seconds=elapsed_time)

    target_timestamp = target_time_utc.timestamp()

    print(f"Target timestamp (UTC): {target_timestamp:.6f}")
    return target_timestamp


def get_start_time_from_metadate(
    metadate_file_path: str, tz_name: str = "Europe/Moscow"
) -> str:
    """
    Получает метку времени в формате "YYYY-MM-DD HH:mm:ss.ffffff AM" из YAML-файла
    :param metadate_file_path: путь к YAML-файлу метаданных
    :param tz_name: название временной зоны
    """
    with open(metadate_file_path, "r", encoding="utf-8") as file:
        data = yaml.safe_load(file)

    try:
        nanoseconds_since_epoch = data["rosbag2_bagfile_information"]["files"][0][
            "starting_time"
        ]["nanoseconds_since_epoch"]

        # Конвертация в секунды и микросекунды
        timestamp_seconds = nanoseconds_since_epoch // 1_000_000_000
        timestamp_microseconds = (nanoseconds_since_epoch % 1_000_000_000) // 1_000

        # Преобразуем в datetime с учетом временной зоны
        utc_time = datetime.fromtimestamp(
            timestamp_seconds, tz=timezone.utc
        ) + timedelta(microseconds=timestamp_microseconds)
        local_time = utc_time.astimezone(pytz.timezone(tz_name))

        formatted_time = local_time.strftime("%Y-%m-%d %I:%M:%S.%f %p")

        return formatted_time

    except KeyError:
        return "Невозможно найти метку времени в файле! Проверьте структуру YAML."


def get_frames_num(metadate_file_path: str, elapsed_time: float, frequency: int) -> int:
    """
    Получает количество доступных кадров из YAML-файла с учетом заданных в config.py параметров
    :param metadate_file_path: путь к YAML-файлу метаданных
    :param elapsed_time: сдвиг времени в секундах
    :param frequency: частота извлечения кадров
    """
    with open(metadate_file_path, "r", encoding="utf-8") as file:
        data = yaml.safe_load(file)

    try:
        duration_ns = data["rosbag2_bagfile_information"]["files"][0]["duration"][
            "nanoseconds"
        ]
        duration_seconds = duration_ns / 1_000_000_000

        # Рассчитываем доступное время для кадров
        available_time = duration_seconds - elapsed_time

        if available_time <= 0:
            return 0

        frames_num = max(0, int(available_time // frequency))
        print("Количество доступных кадров:", frames_num)
        return frames_num

    except KeyError:
        raise "Невозможно прочитать продолжительность записи из файла YAML!"
