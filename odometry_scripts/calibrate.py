import argparse
import json
import os

import cv2
from matplotlib import pyplot as plt
from tqdm import tqdm

DICT = cv2.aruco.DICT_6X6_250  # Тип маркеров aruco
CELLS_X, CELLS_Y = 10, 14  # Размер доски (количество клеток)
CELL_WIDTH, MARKER_WIDTH = 0.049, 0.023  # Размер клетки и маркера в метрах

DICT = cv2.aruco.getPredefinedDictionary(DICT)
BOARD = cv2.aruco.CharucoBoard((CELLS_X, CELLS_Y), CELL_WIDTH, MARKER_WIDTH, DICT)
BOARD.setLegacyPattern(True)

parser = argparse.ArgumentParser(description="Calculate calibration values for camera")
parser.add_argument(
    "-p",
    "--path",
    default="./",
    required=True,
    help="Path to folder with experiment data.",
)
parser.add_argument(
    "-v",
    "--video_path",
    default="./",
    required=True,
    help="Path to video with Charuco board",
)
parser.add_argument(
    "-f", "--show_frames", action="store_true", help="Show frames with charuco board"
)
parser.add_argument(
    "-b", "--show_board", action="store_true", help="Show charuco board"
)

args = parser.parse_args()

if not os.path.exists(args.video_path):
    raise Exception(f"No video file {args.video_path}")

if args.show_board:
    im = BOARD.generateImage((CELLS_X * 100, CELLS_Y * 100))
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot()
    ax.imshow(im, cmap="gray")
    plt.show()

# Изображение можно распечатать, измерить размер клетки
# и обновить константы CELL_WIDTH и MARKER_WIDTH

# UNCOMMENT TO SAVE BOARD IMAGE
# cv2.imwrite('board.png', im)

cap = cv2.VideoCapture(args.video_path)
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

all_corners = []
all_ids = []
current = 0
skip = 5

cv2.namedWindow("frame", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
progress = tqdm(total=int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) / skip)

while True:
    current += 1
    ok, frame = cap.read()
    if not ok:
        break
    if current % skip:
        continue
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aru_corners, aru_ids, _ = cv2.aruco.detectMarkers(gray, DICT)

    if aru_corners:
        ok, char_corners, char_ids = cv2.aruco.interpolateCornersCharuco(
            aru_corners, aru_ids, gray, BOARD
        )

        if char_ids is not None and len(char_corners) > 6:
            all_corners.append(char_corners)
            all_ids.append(char_ids)

        if args.show_frames:
            cv2.aruco.drawDetectedMarkers(frame, aru_corners, aru_ids)
            cv2.aruco.drawDetectedCornersCharuco(frame, char_corners, char_ids)

    if args.show_frames:
        cv2.imshow("frame", frame)
        key = cv2.waitKey(20)
        if key & 0xFF == ord("q"):
            break

    progress.update(1)

cap.release()
cv2.destroyAllWindows()

print("CALIBRATING...")
rms, matrix, dist_coefs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
    all_corners, all_ids, BOARD, (width, height), None, None
)

with open(args.path + "/params.json", "w+") as params_file:
    try:
        params = json.load(params_file)
    except json.JSONDecodeError:
        params = {}

    params["charuco_rms"] = rms
    params["charuco_camera_matrix"] = matrix.tolist()
    params["charuco_camera_dist"] = dist_coefs.ravel().tolist()

    json.dump(params, params_file, indent=4)
