import json

import cv2
import numpy as np

tvec = (1.6520955562591553, -0.0860648825764656, 3.3243510723114014)
rvec = (-2.730807065963745, -1.1112924814224243, 0.49096018075942993)
# camera_matrix = np.array(((1618.1551038169448, 0.0, 1999.5),
# (0.0, 1618.1551038169448, 1499.5), (0.0, 0.0, 1.0)))
# distortion_coeffs = np.array([-0.14999111472353502,
#  0.014430372109933374, 0.0, 0.0, 0.0])

camera_matrix = np.array(
    [
        [1772.55581, 0.00000000, 2005.69013],
        [0.00000000, 1759.59489, 1506.87308],
        [0.00000000, 0.00000000, 1.00000000],
    ],
    dtype=np.float32,
)

distortion_coeffs = np.array(
    [-2.35848581e-01, 6.19417270e-02, 1.60368010e-04, 3.48600473e-04, -7.36220717e-03],
    dtype=np.float32,
)
# tvec = (0.8427895903587341, -0.026787780225276947, 2.05094575881958)
# rvec = (-2.8236916065216064, -1.1281156539916992, 0.2954719066619873)

# Load the video file
video_path = "/Users/zhora/Downloads/Aruco_Odometry/run-01.mp4"
cap = cv2.VideoCapture(video_path)

# Define the ArUco dictionary and detector
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

frame_num = 0

output = []
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers
    corners, ids, rejected = aruco_detector.detectMarkers(gray)

    centers = []
    if ids is not None:
        for i, id in enumerate(ids):
            marker_corners = corners[i][0]  # Shape: (4, 2)
            centers.append(
                {
                    "marker_id": str(id[0]),
                    "x_px": str(int(np.mean(marker_corners[:, 0]))),
                    "y_px": str(int(np.mean(marker_corners[:, 1]))),
                }
            )
    output.append({"frame_num": str(frame_num), "centers": centers})
    frame_num += 1


with open("markers.json", "w+") as output_file:
    print(json.dumps(output), file=output_file)

# # Release the video capture object and close all OpenCV windows
# cap.release()
# cv2.destroyAllWindows()
