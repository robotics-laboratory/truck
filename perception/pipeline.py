import argparse
import os
import re
import shutil

from config import get_settings
from frames_from_mp4 import get_frames_from_mp4_main
from pcd_utils.mcap_to_pcd import mcap_to_pcd_main

parser = argparse.ArgumentParser(
    description="Convert mcap and mp4 files to frames for annotating"
)
parser.add_argument("run_name", type=str, help="Run ID (required)")
args = parser.parse_args()

run_name = str(args.run_name)

video_elapsed_sec, mcap_elapsed_sec, frequency = get_settings(run_name)
last_images_index = get_frames_from_mp4_main(run_name, video_elapsed_sec, frequency)
last_pcd_index = mcap_to_pcd_main(run_name, mcap_elapsed_sec, frequency)


def sync_pointcloud_and_images(
    run_dir: str, last_pcd_index: int, last_images_index: int
):
    """
    Removes extra .pcd files or image folders to synchronize the number of samples.

    :param run_dir: Directory containing the run data.
    :param last_pcd_index: Index of the last valid point cloud file.
    :param last_images_index: Index of the last valid image set.
    """
    pointcloud_dir = os.path.join(run_dir, "pointcloud")
    images_dir = os.path.join(run_dir, "related_images")

    pcd_pattern = re.compile(rf"^{re.escape(run_dir)}-(\d{{3}})\.pcd$")
    img_pattern = re.compile(rf"^{re.escape(run_dir)}_(\d{{3}})_pcd$")

    # Remove extra .pcd files (index > last_images_index)
    for filename in os.listdir(pointcloud_dir):
        match = pcd_pattern.match(filename)
        if match:
            idx = int(match.group(1))
            if idx > last_images_index:
                full_path = os.path.join(pointcloud_dir, filename)
                print(f"Removing excess PCD file: {full_path}")
                os.remove(full_path)

    # Remove extra image folders (index > last_pcd_index)
    for dirname in os.listdir(images_dir):
        match = img_pattern.match(dirname)
        if match:
            idx = int(match.group(1))
            full_path = os.path.join(images_dir, dirname)
            if idx > last_pcd_index:
                print(f"Removing excess image folder: {full_path}")
                shutil.rmtree(full_path)

    print("Synchronization complete.")


sync_pointcloud_and_images(run_name, last_pcd_index, last_images_index)

output_dir = "output_zip"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

archive_path = os.path.join(output_dir, run_name)
shutil.make_archive(archive_path, "zip", root_dir=run_name)
shutil.rmtree(run_name)
print(f"Folder {run_name} archived to {output_dir}/{run_name}.zip and removed.")
