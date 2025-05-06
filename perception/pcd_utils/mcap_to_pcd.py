import os

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from timestamp import get_start_time_from_mcap, get_timestamp

# Description of the point cloud data structure
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
    Save point cloud data to a PCD (Point Cloud Data) file.

    :param points: Structured NumPy array with fields ['x', 'y', 'z'].
    :param filename: Path to the output PCD file.
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


def mcap_to_pcd_main(run_name: str, elapsed_time: float = 0, frequency: int = 2) -> int:
    """
    Extract and save PCD frames from an MCAP file
    based on the given frequency and start offset.

    :param run_name: Name of the run used to locate the MCAP file and save output.
    :param elapsed_time: Initial time offset in seconds
    from the start of the MCAP recording.
    :param frequency: Time interval (in seconds) between saved frames.
    :return: Number of PCD files saved.
    """
    # Set input and output paths
    input_mcap_file = f"data/{run_name}/{run_name}.mcap"
    start_time = get_start_time_from_mcap(input_mcap_file)

    if not os.path.exists(f"{run_name}/pointcloud"):
        os.makedirs(f"{run_name}/pointcloud")

    root_output_dir = run_name

    i = 0
    with open(input_mcap_file, "rb") as inp_file:
        reader = make_reader(inp_file, decoder_factories=[DecoderFactory()])
        for schema, channel, message, ros_msg in reader.iter_decoded_messages(
            topics=["/livox/lidar"]
        ):
            timestamp = message.log_time / 1e9  # Convert nanoseconds to seconds
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
                elapsed_time += frequency
    last_pcd_index = i
    return last_pcd_index


if __name__ == "__main__":
    test_run_name = "example"
    mcap_to_pcd_main(test_run_name)
