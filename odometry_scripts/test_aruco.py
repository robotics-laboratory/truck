import cv2
import numpy as np
import os
import json

import argparse

START_FRAME_NUM = 0

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
with open(args.path + "/params.json", "r") as params_file:
    params = json.load(params_file)
    distortion_coeffs = np.asarray(params["grid_camera_dist"], dtype=np.float32)
    camera_matrix = np.asarray(params["grid_camera_matrix"], dtype=np.float32)
    width, height = params["frame_size"]

new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coeffs, (width, height), 0.6, (width, height))

frame_index = START_FRAME_NUM
is_undistor = False
step = 1

video_path = args.path + "/video.mp4"
cap = cv2.VideoCapture(video_path)

def load_frame(index):
    cap.set(cv2.CAP_PROP_POS_FRAMES, index * step)

    ret, img = cap.read()
    if not ret:
        return None, None

    if is_undistor:
        img = cv2.undistort(img, camera_matrix, distortion_coeffs, None, new_camera_matrix)
    return img, cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

image, gray = load_frame(frame_index)
if image is None:
    raise FileNotFoundError("Frame not found")

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

cv2.namedWindow("Aruco Detection")

need_to_update = True

def update(x):
    global need_to_update
    need_to_update = True

cv2.createTrackbar("WinMin", "Aruco Detection", 3, 50, update)
cv2.createTrackbar("WinMax", "Aruco Detection", 30, 100, update)
cv2.createTrackbar("WinStep", "Aruco Detection", 3, 50, update)
cv2.createTrackbar("ThreshC", "Aruco Detection", 6, 20, update)
cv2.createTrackbar("Approx x100", "Aruco Detection", 3, 100, update)

cv2.createTrackbar("Pixels per cell", "Aruco Detection", 19, 40, update)
cv2.createTrackbar("Cr_marg x100", "Aruco Detection", 18, 40, update)
cv2.createTrackbar("Otsu x10", "Aruco Detection", 50, 500, update)

last_crop_center = None
frame_height, frame_width = image.shape[:2]
crop_width, crop_height = 800, 600

while True:
    win_min = cv2.getTrackbarPos("WinMin", "Aruco Detection")
    win_max = cv2.getTrackbarPos("WinMax", "Aruco Detection")
    win_step = cv2.getTrackbarPos("WinStep", "Aruco Detection")
    thresh_c = cv2.getTrackbarPos("ThreshC", "Aruco Detection")
    min_perim = 2.0 / 100.0
    max_perim = 20.0 / 100.0
    approx = cv2.getTrackbarPos("Approx x100", "Aruco Detection") / 100.0
    min_border = 10

    pix_per_cell = cv2.getTrackbarPos("Pixels per cell", "Aruco Detection")
    cropped_margin = cv2.getTrackbarPos("Cr_marg x100", "Aruco Detection") / 100.0
    otsu_dev = cv2.getTrackbarPos("Otsu x10", "Aruco Detection") / 10.0

    if need_to_update:
        if image is None or gray is None:
            continue
        params = cv2.aruco.DetectorParameters()
        params.adaptiveThreshWinSizeMin = win_min
        params.adaptiveThreshWinSizeMax = win_max
        params.adaptiveThreshWinSizeStep = win_step
        params.adaptiveThreshConstant = thresh_c
        params.minMarkerPerimeterRate = min_perim
        params.maxMarkerPerimeterRate = max_perim
        params.polygonalApproxAccuracyRate = approx
        params.minCornerDistanceRate = 0.1
        params.minDistanceToBorder = min_border
        params.minMarkerDistanceRate = 0.1
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        params.cornerRefinementWinSize = int(pix_per_cell // 2 * 2 + 1)
        params.cornerRefinementMaxIterations = 30
        params.cornerRefinementMinAccuracy = 0.1
        params.markerBorderBits = 1
        params.perspectiveRemovePixelPerCell = pix_per_cell
        params.perspectiveRemoveIgnoredMarginPerCell = cropped_margin
        params.maxErroneousBitsInBorderRate = 0.35
        params.minOtsuStdDev = otsu_dev
        params.errorCorrectionRate = 0.6

        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        corners, ids, rejected = detector.detectMarkers(gray)
        crop_center = None
        if ids is not None and len(corners) > 0:
            all_pts = np.concatenate(corners, axis=0)
            mean_point = np.mean(all_pts, axis=1)[0]
            crop_center = (int(mean_point[0]), int(mean_point[1]))
            last_crop_center = crop_center
        elif last_crop_center is not None:
            crop_center = last_crop_center

        vis = image.copy()
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(vis, rejected, borderColor=(0, 0, 255))  # Красным
            cv2.aruco.drawDetectedMarkers(vis, corners, ids)

        if crop_center is not None:
            cx, cy = crop_center
            x1 = max(0, cx - crop_width // 2)
            y1 = max(0, cy - crop_height // 2)
            x2 = min(frame_width, x1 + crop_width)
            y2 = min(frame_height, y1 + crop_height)
            x1 = max(0, x2 - crop_width)
            y1 = max(0, y2 - crop_height)
            vis = vis[y1:y2, x1:x2]
        else:
            vis = vis

        cv2.putText(vis, f"Frame {frame_index}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(vis, "Undistorted image" if is_undistor else "Original image", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Aruco test", vis)
        need_to_update = False
    key = cv2.waitKey(20)

    if key == 2:  # left arrow
        frame_index = max(0, frame_index - 1)
        image, gray = load_frame(frame_index)
        need_to_update = True
    elif key == 3:  # right arrow
        frame_index += 1
        image, gray = load_frame(frame_index)
        need_to_update = True
    elif key == 32: # space
        is_undistor = not is_undistor
        image, gray = load_frame(frame_index)
        need_to_update = True
    elif key == 0:  # up
        frame_index += 20
        image, gray = load_frame(frame_index)
        need_to_update = True
    elif key == 1:  # down
        frame_index = max(0, frame_index - 20)
        image, gray = load_frame(frame_index)
        need_to_update = True

    if key == 27:
        break

cv2.destroyAllWindows()