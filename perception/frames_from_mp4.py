import os
from typing import List

import cv2


def extract_frames_by_frequency(
    video_paths: List[str], frequency: int, elapsed: float, root_output_dir: str
) -> int:
    """
    Extract frames from four-angle video recordings at a specified frequency,
    starting from a given time offset (elapsed), and save them as images.

    :param video_paths: List of paths to video files.
    :param frequency: Frequency of frame extraction in seconds.
    :param elapsed: Time offset in seconds from the start of the video.
    :param root_output_dir: Root directory where images will be saved.
    :return: Index of the last saved frame batch.
    """
    if not os.path.exists(root_output_dir):
        os.makedirs(root_output_dir)

    launch_name = os.path.basename(root_output_dir)

    last_images_index = -1

    for video_path in video_paths:
        angle = os.path.splitext(os.path.basename(video_path))[0].split("-")[-1]
        cap = cv2.VideoCapture(video_path)

        if not cap.isOpened():
            print(f"Failed to open video file: {video_path}")
            continue

        fps = cap.get(cv2.CAP_PROP_FPS)
        if fps == 0:
            print(f"Could not determine FPS for video: {video_path}")
            cap.release()
            continue

        video_duration = cap.get(cv2.CAP_PROP_FRAME_COUNT) / fps
        # Account for the elapsed time offset
        elapsed_times = [
            elapsed + i * frequency
            for i in range(0, int((video_duration - elapsed) // frequency))
        ]

        for i, elapsed_time in enumerate(elapsed_times):
            sub_dir_name = f"{launch_name}-{str(i).zfill(3)}_pcd"
            output_dir = os.path.join(root_output_dir, "related_images", sub_dir_name)
            os.makedirs(output_dir, exist_ok=True)

            frame_number = int(elapsed_time * fps)
            cap.set(cv2.CAP_PROP_POS_FRAMES, frame_number)

            ret, frame = cap.read()
            if not ret:
                print(
                    f"Failed to extract frame at {elapsed_time:.2f} sec from "
                    f"{video_path}"
                )
                continue

            output_file_name = f"{launch_name}-{str(i).zfill(3)}-{angle}.jpg"
            output_path = os.path.join(output_dir, output_file_name)

            cv2.imwrite(output_path, frame)
            print(
                f"Frame at {elapsed_time:.2f} sec "
                f"from {video_path} successfully saved: "
                f"{output_path}"
            )
            if i > last_images_index:
                last_images_index = i

        cap.release()

    return last_images_index


def get_frames_from_mp4_main(
    run_name: str, elapsed: float = 0, frequency: int = 2
) -> int:
    """
    Main function to extract frames from four-angle video files for a given run.

    :param run_name: Name of the run
    (used to find video files and determine output paths).
    :param elapsed: Time offset in seconds from which to start extracting frames.
    :param frequency: Frame extraction interval in seconds.
    :return: Index of the last saved frame batch.
    """
    video_files = [
        f"data/{run_name}/{run_name}-0.mp4",
        f"data/{run_name}/{run_name}-90.mp4",
        f"data/{run_name}/{run_name}-180.mp4",
        f"data/{run_name}/{run_name}-270.mp4",
    ]
    root_output_directory = run_name  # Root directory for saving images

    last_images_index = extract_frames_by_frequency(
        video_files, frequency, elapsed, root_output_directory
    )
    return last_images_index


if __name__ == "__main__":
    test_run_name = "example"
    get_frames_from_mp4_main(test_run_name)
