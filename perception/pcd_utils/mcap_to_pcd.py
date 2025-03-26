import os

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

from perception.config import Config
from timestamp import get_timestamp, get_start_time, get_frames_num
import numpy as np


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
    Сохраняет точки в формате PCD вручную.
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


def find_and_save_points_by_time(input_file, output_file, target_time, epsilon=1e-1):
    """
    Ищет и сохраняет облако точек для указанного времени.

    Args:
        input_file (str): Путь к файлу MCAP.
        output_file (str): Путь для сохранения PCD.
        target_time (float): Целевой timestamp (UNIX time).
        epsilon (float): Допустимое отклонение в секундах.
    """
    with open(input_file, "rb") as inp_file:
        reader = make_reader(inp_file, decoder_factories=[DecoderFactory()])
        found = False
        for schema, channel, message, ros_msg in reader.iter_decoded_messages(
            topics=["/livox/lidar"]
        ):
            timestamp = message.log_time / 1e9
            if abs(timestamp - target_time) <= epsilon:
                print(f"Target timestamp {target_time:.6f} found! Saving points.")
                points = np.frombuffer(ros_msg.data, dtype=dtype)
                save_pcd_manual(points, output_file)
                found = True
                break

        if not found:
            print(f"Timestamp {target_time} not found in the file.")


def mcap_to_pcd_main():
    input_mcap_file = f"input_mcap/{Config.run_name}.mcap"
    input_metadate_file = f"input_mcap/{Config.run_name}_metadata.yaml"
    start_time = get_start_time(input_metadate_file)
    elapsed_time = Config.elapsed_time

    if not os.path.exists(f"{Config.run_name}/pointcloud"):
        os.makedirs(f"{Config.run_name}/pointcloud")

    root_output_dir = Config.run_name
    frames_num = get_frames_num(
        input_metadate_file, Config.elapsed_time, Config.frequency
    )
    for i in range(frames_num):
        output_pcd_file = (
            root_output_dir
            + "/pointcloud/"
            + root_output_dir
            + "-"
            + str(i).zfill(3)
            + ".pcd"
        )
        target_timestamp = get_timestamp(start_time, elapsed_time)
        find_and_save_points_by_time(input_mcap_file, output_pcd_file, target_timestamp)
        elapsed_time += Config.frequency


if __name__ == "__main__":
    mcap_to_pcd_main()
