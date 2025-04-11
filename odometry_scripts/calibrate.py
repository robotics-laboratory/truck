import json

import cv2
from matplotlib import pyplot as plt

DICT = cv2.aruco.DICT_6X6_250  # Тип маркеров aruco
CELLS_X, CELLS_Y = 10, 14  # Размер доски (количество клеток)
CELL_WIDTH, MARKER_WIDTH = 0.049, 0.023  # Размер клетки и маркера в метрах

DICT = cv2.aruco.getPredefinedDictionary(DICT)
BOARD = cv2.aruco.CharucoBoard((CELLS_X, CELLS_Y), CELL_WIDTH, MARKER_WIDTH, DICT)
BOARD.setLegacyPattern(True)

im = BOARD.generateImage((CELLS_X * 100, CELLS_Y * 100))
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot()
ax.imshow(im, cmap="gray")
# plt.show()

# Изображение можно распечатать, измерить размер клетки
# и обновить константы CELL_WIDTH и MARKER_WIDTH

# UNCOMMENT TO SAVE BOARD IMAGE
# cv2.imwrite('board.png', im)

# Загружаем видео, где перед камерой показана
# калибровчная доска в различных положениях
cap = cv2.VideoCapture("GX010440.MP4")
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"VIDEO SIZE: {width}x{height}")

all_corners = []
all_ids = []
current = 0
skip = 5

cv2.namedWindow("frame", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)

while True:
    current += 1
    ok, frame = cap.read()
    if not ok:
        break
    if current % skip:
        continue
    print("frame", current)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aru_corners, aru_ids, _ = cv2.aruco.detectMarkers(gray, DICT)

    if aru_corners:
        ok, char_corners, char_ids = cv2.aruco.interpolateCornersCharuco(
            aru_corners, aru_ids, gray, BOARD
        )

        if char_ids is not None and len(char_corners) > 6:
            all_corners.append(char_corners)
            all_ids.append(char_ids)

        cv2.aruco.drawDetectedMarkers(frame, aru_corners, aru_ids)
        cv2.aruco.drawDetectedCornersCharuco(frame, char_corners, char_ids)

    cv2.imshow("frame", frame)
    key = cv2.waitKey(20)
    if key & 0xFF == ord("q"):
        break

print("TOTAL CORNERS:", len(all_corners))
cap.release()
cv2.destroyAllWindows()

print("CALIBRATING...")
rms, matrix, dist_coefs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
    all_corners, all_ids, BOARD, (width, height), None, None
)

print("--> CALIBRATION RESULTS:")
print("RMS:", rms)
print("CAMERA MATRIX:\n", matrix)
print("DISTORTION COEFFICIENTS:\n", dist_coefs.ravel())

with open("params.json", "w+") as params_file:
    params_str = params_file.read()

    if params_str:
        params = json.loads(params_str)
    else:
        params = {}
    params["frame_size"] = [width, height]
    params["matrix"] = matrix
    params["dist"] = dist_coefs.ravel()
    params["rms"] = matrix
    params_file.seek(0)
    print(json.dumps(params), file=params_file)
# [[1772.55581, 0.0, 2005.69013], [0.0, 1759.59489, 1506.87308], [0.0, 0.0, 1.0]]
# [-0.235848581, 0.061941727, 0.00016036801, 0.000348600473, -0.00736220717]
