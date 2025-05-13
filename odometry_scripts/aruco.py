import argparse
import json
import os
import concurrent.futures
import math
from multiprocessing import Manager
import threading
import time

import cv2
import numpy as np
from tqdm import tqdm

NUM_WORKERS = 8

parser = argparse.ArgumentParser(
    description="Detect Aruco markers on video and save them to json."
)
parser.add_argument(
    "-p",
    "--path",
    default="./",
    required=True,
    help="Path to folder with experiment data.",
)

args = parser.parse_args()

video_path = args.path + "/video.mp4"
if not os.path.exists(video_path):
    raise Exception(f"No video file {video_path}")

os.environ["OPENCV_FFMPEG_READ_ATTEMPTS"] = "16000"

def monitor_progress(counter, total):
    with tqdm(total=total, desc="Proccessed frames") as pbar:
        last = 0
        while last < total:
            time.sleep(0.1)
            with lock:
                current = counter.value
                pbar.update(current - last)
                last = current

def process_chunk(start_idx, end_idx, counter, lock):
    results = []
    cap = cv2.VideoCapture(video_path)
    cap.set(cv2.CAP_PROP_POS_FRAMES, start_idx)

    for frame_idx in range(start_idx, end_idx):
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.undistort(frame, camera_matrix, distortion_coeffs, None, new_camera_matrix)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco_detector.detectMarkers(gray)

        centers = []
        if ids is not None:
            for i, id in enumerate(ids):
                marker_corners = corners[i][0]
                centers.append({
                    "marker_id": int(id[0]),
                    "x_px": int(np.mean(marker_corners[:, 0])),
                    "y_px": int(np.mean(marker_corners[:, 1])),
                })

        results.append({"frame_num": frame_idx, "centers": centers})

        with lock:
            counter.value += 1

    cap.release()
    return results

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
aruco_params.adaptiveThreshWinSizeMin = 3
aruco_params.adaptiveThreshWinSizeMax = 30
aruco_params.adaptiveThreshWinSizeStep = 3
aruco_params.adaptiveThreshConstant = 8
aruco_params.minMarkerPerimeterRate = 0.02
aruco_params.maxMarkerPerimeterRate = 0.2
aruco_params.polygonalApproxAccuracyRate = 0.03
aruco_params.minCornerDistanceRate = 0.1
aruco_params.minDistanceToBorder = 10
aruco_params.minMarkerDistanceRate = 0.1
aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
aruco_params.cornerRefinementWinSize = int(19 // 2 * 2 + 1)
aruco_params.cornerRefinementMaxIterations = 30
aruco_params.cornerRefinementMinAccuracy = 0.1
aruco_params.markerBorderBits = 1
aruco_params.perspectiveRemovePixelPerCell = 19
aruco_params.perspectiveRemoveIgnoredMarginPerCell = 0.18
aruco_params.maxErroneousBitsInBorderRate = 0.35
aruco_params.minOtsuStdDev = 5
aruco_params.errorCorrectionRate = 0.6
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

with open(args.path + "/params.json", "r") as params_file:
    params = json.load(params_file)
    distortion_coeffs = np.asarray(params["grid_camera_dist"], dtype=np.float32)
    camera_matrix = np.asarray(params["grid_camera_matrix"], dtype=np.float32)
    width, height = params["frame_size"]

ALPHA = 0.6
new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coeffs, (width, height), ALPHA, (width, height))

cap = cv2.VideoCapture(video_path)

frame_path = args.path + "/frame.png"
if not os.path.exists(frame_path):
    ret, frame = cap.read()
    if ret:
        cv2.imwrite(frame_path, frame)
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
cap.release()

chunk_size = math.ceil(total_frames / NUM_WORKERS)
chunks = [(i, min(i + chunk_size, total_frames)) for i in range(0, total_frames, chunk_size)]

if __name__ == "__main__":
    output = []

    manager = Manager()
    lock = manager.Lock()
    counter = manager.Value('i', 0)

    monitor_thread = threading.Thread(target=monitor_progress, args=(counter, total_frames))
    monitor_thread.start()

    with concurrent.futures.ProcessPoolExecutor(max_workers=NUM_WORKERS) as executor:
        futures = [executor.submit(process_chunk, start, end, counter, lock) for start, end in chunks]
        for f in concurrent.futures.as_completed(futures):
            output.extend(f.result())

    monitor_thread.join()

    with open(args.path + "/markers.json", "w+") as markers_file:
        json.dump(sorted(output, key=lambda x: x["frame_num"]), markers_file)