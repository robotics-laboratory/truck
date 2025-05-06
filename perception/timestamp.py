from datetime import datetime, timedelta, timezone

import pytz
import yaml
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


def get_timestamp(start_time_string: str, elapsed_time: float = 0) -> float:
    """
    Gets the UNIX timestamp and returns the time in seconds
    with the applied elapsed_time shift.

    :param start_time_string: Start time string in the format
    "YYYY-MM-DD HH:mm:ss.ffffff AM"
    :param elapsed_time: Time shift in seconds
    """
    start_time_msk = datetime.strptime(start_time_string, "%Y-%m-%d %I:%M:%S.%f %p")
    start_time_utc = start_time_msk

    # Compute the target timestamp in UTC considering the elapsed_time shift
    target_time_utc = start_time_utc + timedelta(seconds=elapsed_time)

    target_timestamp = target_time_utc.timestamp()

    print(f"Target timestamp (UTC): {target_timestamp:.6f}")
    return target_timestamp


def get_start_time_from_metadate(
    metadate_file_path: str, tz_name: str = "Europe/Moscow"
) -> str:
    """
    Gets the timestamp in the format "YYYY-MM-DD HH:mm:ss.ffffff AM" from a YAML file.

    :param metadate_file_path: Path to the YAML metadata file
    :param tz_name: Time zone name
    """
    with open(metadate_file_path, "r", encoding="utf-8") as file:
        data = yaml.safe_load(file)

    try:
        nanoseconds_since_epoch = data["rosbag2_bagfile_information"]["files"][0][
            "starting_time"
        ]["nanoseconds_since_epoch"]

        # Convert to seconds and microseconds
        timestamp_seconds = nanoseconds_since_epoch // 1_000_000_000
        timestamp_microseconds = (nanoseconds_since_epoch % 1_000_000_000) // 1_000

        # Convert to datetime considering the time zone
        utc_time = datetime.fromtimestamp(
            timestamp_seconds, tz=timezone.utc
        ) + timedelta(microseconds=timestamp_microseconds)
        local_time = utc_time.astimezone(pytz.timezone(tz_name))

        formatted_time = local_time.strftime("%Y-%m-%d %I:%M:%S.%f %p")

        return formatted_time

    except KeyError:
        return "Unable to find timestamp in the file! Check the YAML structure."


def get_start_time_from_mcap(
    input_mcap_file: str, tz_name: str = "Europe/Moscow"
) -> str:
    """
    Extracts the start time from an MCAP file and returns it as a formatted string.

    :param input_mcap_file: Path to the MCAP file
    :param tz_name: Time zone name
    """
    with open(input_mcap_file, "rb") as inp_file:
        reader = make_reader(inp_file, decoder_factories=[DecoderFactory()])

        for schema, channel, message, ros_msg in reader.iter_decoded_messages(
            topics=["/livox/lidar"]
        ):
            timestamp_seconds = message.log_time // 10**9
            timestamp_microseconds = (message.log_time % 10**9) / 1000

            utc_time = datetime.fromtimestamp(
                timestamp_seconds, tz=timezone.utc
            ) + timedelta(microseconds=timestamp_microseconds)

            local_time = utc_time.astimezone(pytz.timezone(tz_name))
            return local_time.strftime("%Y-%m-%d %I:%M:%S.%f %p")

    raise ValueError("No messages found on topic /livox/lidar.")


def get_frames_num(
        metadate_file_path: str,
        elapsed_time: float,
        frequency: int
) -> int:
    """
    Gets the number of available frames from a YAML metadata file,
    considering the given config parameters.

    :param metadate_file_path: Path to the YAML metadata file
    :param elapsed_time: Time shift in seconds
    :param frequency: Frame extraction frequency
    """
    with open(metadate_file_path, "r", encoding="utf-8") as file:
        data = yaml.safe_load(file)

    try:
        duration_ns = data["rosbag2_bagfile_information"]["files"][0]["duration"][
            "nanoseconds"
        ]
        duration_seconds = duration_ns / 1_000_000_000

        # Calculate available time for frames
        available_time = duration_seconds - elapsed_time

        if available_time <= 0:
            return 0

        frames_num = max(0, int(available_time // frequency))
        print("Number of available frames:", frames_num)
        return frames_num

    except KeyError:
        raise Exception("Unable to read the duration from the YAML file!")
