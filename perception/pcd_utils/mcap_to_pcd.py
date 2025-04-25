import os

import numpy as np
from config import Config
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from timestamp import get_frames_num, get_start_time_from_metadate, get_timestamp

# Описание структуры данных
dtype = np.dtype(
    [
        ("x", "float32"),
        ("y", "float32"),
        ("z", "float32"),
        ("intensity", "float32"),
        ("tag", "uint8"),
        ("line", "uint8"),
        ("timestamp", "float64"),
    ],
    align=False,
)


def save_pcd_manual(points, filename):
    """
    Сохраняет точки в формате PCD
    """
    xyz = np.stack((points["x"], points["y"], points["z"]), axis=-1)
    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {xyz.shape[0]}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {xyz.shape[0]}
DATA ascii
"""
    with open(filename, "w") as f:
        f.write(header)
        np.savetxt(f, xyz, fmt="%.6f %.6f %.6f")

    print(f"Saved {xyz.shape[0]} points to {filename}")


def mcap_to_pcd_main():
    """
    Получает кадры pcd из mcap с учетом заданных в config.py параметров
    """
    # Задаем параметры
    input_mcap_file = f"input_mcap/{Config.run_name}.mcap"
    input_metadate_file = f"input_mcap/{Config.run_name}_metadata.yaml"
    start_time = get_start_time_from_metadate(input_metadate_file)
    elapsed_time = Config.elapsed_time

    if not os.path.exists(f"{Config.run_name}/pointcloud"):
        os.makedirs(f"{Config.run_name}/pointcloud")

    root_output_dir = Config.run_name
    frames_num = get_frames_num(
        input_metadate_file, Config.elapsed_time, Config.frequency
    )

    i = 0
    with open(input_mcap_file, "rb") as inp_file:
        reader = make_reader(inp_file, decoder_factories=[DecoderFactory()])
        for schema, channel, message, ros_msg in reader.iter_decoded_messages(
            topics=["/livox/lidar"]
        ):
            if i >= frames_num - 1:
                break
            timestamp = message.log_time / 1e9  # Преобразуем в секунды
            if timestamp > get_timestamp(start_time, elapsed_time):
                output_file = (
                    root_output_dir
                    + "/pointcloud/"
                    + root_output_dir
                    + "-"
                    + str(i).zfill(3)
                    + ".pcd"
                )
                print(f"Target timestamp {timestamp:.6f} found! Saving points.")
                points = np.frombuffer(ros_msg.data, dtype=dtype)
                save_pcd_manual(points, output_file)

                i += 1
                elapsed_time += Config.frequency


if __name__ == "__main__":
    mcap_to_pcd_main()
