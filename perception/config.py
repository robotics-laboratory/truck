import os
from typing import Tuple

import yaml


def get_settings(run_name: str) -> Tuple[float, float, int]:
    """
    Load configuration settings from a YAML file in the run directory.

    :param run_name: Name of the run (used to locate the config folder).
    :return: A tuple containing video_elapsed (float), mcap_elapsed (float),
             and frequency (int) from the config file.
    """
    base_dir = os.path.dirname(os.path.abspath(__file__))
    # Path to the current .py file
    folder_path = os.path.join(base_dir, "data", run_name)

    if not os.path.exists(folder_path):
        raise FileNotFoundError(f"Run folder {folder_path} not found")

    config_path = os.path.join(folder_path, "config.yaml")

    if os.path.exists(config_path):
        with open(config_path, "r") as f:
            config_data = yaml.safe_load(f) or {}
    else:
        raise FileNotFoundError(f"Config file {config_path} not found")

    if (
        "video_elapsed" not in config_data
        or "mcap_elapsed" not in config_data
        or "frequency" not in config_data
    ):
        raise Exception("Config is incorrect")

    return (
        float(config_data["video_elapsed"]),
        float(config_data["mcap_elapsed"]),
        config_data["frequency"],
    )
