from datetime import timezone

import pytz
from hachoir.metadata import extractMetadata
from hachoir.parser import createParser


def get_insv_start_time(file_path: str, tz_name: str = "Europe/Moscow") -> str:
    """
    Extracts the recording start time from an INSV file and
    converts it to the specified timezone

    :param file_path: Path to the INSV file
    :param tz_name: Name of the timezone to convert the start time to
    (default is "Europe/Moscow")
    :return: The formatted start time string in the given timezone
    :raises ValueError: If the file cannot be parsed or metadata cannot be extracted
    """
    parser = createParser(file_path)
    if not parser:
        raise ValueError(f"Unable to parse file: {file_path}")

    metadata = extractMetadata(parser)
    if not metadata:
        raise ValueError(f"Unable to extract metadata from: {file_path}")

    if metadata.has("creation_date"):
        utc_time = metadata.get("creation_date").replace(tzinfo=timezone.utc)
        local_time = utc_time.astimezone(pytz.timezone(tz_name))
        return local_time.strftime("%Y-%m-%d %I:%M:%S.%f %p")
    else:
        raise ValueError("Creation date not found in metadata.")


if __name__ == "__main__":
    start_time = get_insv_start_time("test.insv")
    print("Recording start time:", start_time)
