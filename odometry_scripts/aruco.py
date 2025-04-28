import argparse
import json
import os

import cv2
import numpy as np
from tqdm import tqdm

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

cap = cv2.VideoCapture(video_path)

frame_path = args.path + "/frame.png"
if not os.path.exists(frame_path):
    ret, frame = cap.read()
    if ret:
        cv2.imwrite(frame_path, frame)
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

# Define the ArUco dictionary and detector
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

frame_num = 0
output = []

progress = tqdm(total=int(cap.get(cv2.CAP_PROP_FRAME_COUNT)))

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco_detector.detectMarkers(gray)

    centers = []
    if ids is not None:
        for i, id in enumerate(ids):
            marker_corners = corners[i][0]
            centers.append(
                {
                    "marker_id": int(id[0]),
                    "x_px": int(np.mean(marker_corners[:, 0])),
                    "y_px": int(np.mean(marker_corners[:, 1])),
                }
            )
    output.append({"frame_num": int(frame_num), "centers": centers})
    frame_num += 1
    progress.update(1)

with open(args.path + "/params.json", "w+") as params_file:
    try:
        params = json.load(params_file)
    except json.JSONDecodeError:
        params = {}
    params["fps"] = int(round(cap.get(cv2.CAP_PROP_FPS)))
    params["frame_size"] = [
        int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
        int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
    ]

    json.dump(params, params_file)

with open(args.path + "/markers.json", "w+") as markers_file:
    json.dump(output, markers_file)
