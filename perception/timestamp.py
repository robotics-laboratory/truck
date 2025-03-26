from datetime import datetime, timedelta, timezone
import yaml
import pytz


def get_timestamp(start_time_str, elapsed_time):
    # Парсим начальное время и переводим в UTC
    start_time_msk = datetime.strptime(start_time_str, "%Y-%m-%d %I:%M:%S.%f %p")
    start_time_utc = start_time_msk

    target_time_utc = start_time_utc + timedelta(seconds=elapsed_time)

    target_timestamp = target_time_utc.timestamp()

    print(f"Target timestamp (UTC): {target_timestamp:.6f}")
    return target_timestamp


def get_start_time(file_path, tz_name="Europe/Moscow") -> str:
    with open(file_path, "r", encoding="utf-8") as file:
        data = yaml.safe_load(file)

    try:
        nanoseconds_since_epoch = data["rosbag2_bagfile_information"]["files"][0][
            "starting_time"
        ]["nanoseconds_since_epoch"]

        timestamp_seconds = nanoseconds_since_epoch // 1_000_000_000
        timestamp_microseconds = (nanoseconds_since_epoch % 1_000_000_000) // 1_000

        utc_time = datetime.fromtimestamp(
            timestamp_seconds, tz=timezone.utc
        ) + timedelta(microseconds=timestamp_microseconds)
        local_time = utc_time.astimezone(pytz.timezone(tz_name))

        formatted_time = local_time.strftime("%Y-%m-%d %I:%M:%S.%f %p")

        return formatted_time

    except KeyError:
        return "Невозможно найти метку времени в файле! Проверьте структуру YAML."


def get_frames_num(file_path, elapsed_time: float, frequency: int) -> int:
    with open(file_path, "r", encoding="utf-8") as file:
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
