import json

import cv2
import numpy as np


def frame_to_ray(point, matrix):
    # https://github.com/ros-perception/vision_opencv/blob/noetic/image_geometry
    # /src/image_geometry/cameramodels.py#L128
    x, y = point
    cx, cy = matrix[0, 2], matrix[1, 2]
    fx, fy = matrix[0, 0], matrix[1, 1]
    x, y = (x - cx) / fx, (y - cy) / fy
    norm = np.sqrt(x * x + y * y + 1)
    return x / norm, y / norm, 1.0 / norm


def frame_to_world(point, position, rotation, matrix, z: float = 0.0):
    rvec, _ = cv2.Rodrigues(rotation)
    tvec = np.array(position, dtype="float32").reshape((3, 1))
    rvec = np.linalg.inv(rvec)

    ray = frame_to_ray(point, matrix)
    ray = np.array(ray, dtype="float32").reshape((3,))
    ray = ray.dot(rvec)

    plane_normal = np.array([0, 0, 1], dtype="float32")
    plane_point = np.array([0, 0, z], dtype="float32")
    ndotu = plane_normal.dot(ray)
    if abs(ndotu) < 1e-6:
        return None

    w = tvec.reshape((3,)) - plane_point
    si = -plane_normal.dot(w) / ndotu
    psi = w + si * ray + plane_point
    return tuple(psi)


# tvec = (1.6520955562591553, -0.0860648825764656, 3.3243510723114014)
# rvec = (-2.730807065963745, -1.1112924814224243, 0.49096018075942993)
# camera_matrix = np.array(((1618.1551038169448, 0.0, 1999.5),
# (0.0, 1618.1551038169448, 1499.5), (0.0, 0.0, 1.0)))
# distortion_coeffs = np.array([-0.14999111472353502, 0.014430372109933374,
# 0.0, 0.0, 0.0])

# camera_matrix = np.array([[1772.55581, 0.00000000, 2005.69013],
#                           [0.00000000, 1759.59489, 1506.87308],
#                           [0.00000000, 0.00000000, 1.00000000]], dtype=np.float32)

# distortion_coeffs = np.array([-2.35848581e-01, 6.19417270e-02, 1.60368010e-04,
# 3.48600473e-04, -7.36220717e-03], dtype=np.float32)
# tvec = (0.8427895903587341, -0.026787780225276947, 2.05094575881958)
# rvec = (-2.8236916065216064, -1.1281156539916992, 0.2954719066619873)


camera_matrix = np.array(
    [
        [1.74376435e03, 0.00000000e00, 1.99247927e03],
        [0.00000000e00, 1.74325098e03, 1.51085422e03],
        [0.00000000e00, 0.00000000e00, 1.00000000e00],
    ],
    dtype=np.float32,
)

distortion_coeffs = np.array(
    [
        -2.58957090e-01,
        9.14005474e-02,
        -3.39398790e-05,
        -9.52750304e-05,
        -1.73351552e-02,
    ],
    dtype=np.float32,
)

# rvec = (2.72847023,
#         1.11215459,
#        -0.50500178)

# tvec = (-0.22881838,
#        -0.29328163,
#         3.80346644)

tvec = (1.72401372, -0.099824, 3.40918477)
rvec = (-2.72847023, -1.11215459, 0.50500178)

width, height = (4000, 3000)
# with open("params.json", "r") as params_file:
#     params = json.load(params_file)
#     tvec = params["tvecs"]
#     rvec = params["rvecs"]
#     distortion_coeffs = params["dist"]
#     camera_matrix = params["matrix"]
#     width, height = params["frame_size"]

with open("markers.json", "r") as markers_file:
    input_markers = json.load(markers_file)

output = []

# Correct the distortion in the frame
ALPHA = 0.6
mtx, _ = cv2.getOptimalNewCameraMatrix(
    camera_matrix, distortion_coeffs, (width, height), ALPHA
)
# map1, map2 = cv2.initUndistortRectifyMap(camera_matrix, distortion_coeffs, None, mtx,
# (width, height), cv2.CV_16SC2)

for frame in input_markers:
    frame_num = frame["frame_num"]
    car_center = (None, None)
    if len(frame["centers"]) == 2:
        front_marker = frame["centers"][0]
        back_marker = frame["centers"][1]
        car_center = (None, None)
        if (int(front_marker["marker_id"]) == 1) and (
            int(back_marker["marker_id"]) == 0
        ):
            front_marker, back_marker = back_marker, front_marker

        if (int(front_marker["marker_id"]) == 0) and (
            int(back_marker["marker_id"]) == 1
        ):
            front_pose = frame_to_world(
                (float(front_marker["x_px"]), float(front_marker["y_px"])),
                tvec,
                rvec,
                mtx,
                0.15,
            )
            back_pose = frame_to_world(
                (float(back_marker["x_px"]), float(back_marker["y_px"])),
                tvec,
                rvec,
                mtx,
                0.15,
            )
            car_center = (np.array(front_pose) + np.array(back_pose)) / 2

    output.append(
        {
            "frame_num": frame_num,
            "x_pos": str(car_center[0]),
            "y_pos": str(car_center[1]),
        }
    )

with open("odom.json", "w+") as output_file:
    json.dump(output, fp=output_file)

# width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
# height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# frame_num = 450
# prev_frame_num = frame_num - 1
# cap.set(cv2.CAP_PROP_POS_FRAMES, frame_num - 1)

# prev_pose = (0,0,0)
# car_centers =[]


# while cap.isOpened():
# # for _ in range(500):
#     ret, frame = cap.read()
#     if not ret:
#         break

#     # undistorted_frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

#     # undistorted_frame = cv2.undistort(frame, camera_matrix, distortion_coeffs)

#     # Convert the frame to grayscale
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#     # Detect ArUco markers
#     corners, ids, rejected = aruco_detector.detectMarkers(gray)

#     if ids is not None:
#         # Draw the detected markers
#         # cv2.aruco.drawDetectedMarkers(undistorted_frame, corners, ids)

#         # Estimate the pose of each marker
#         # rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.1,
# camera_matrix, distortion_coeffs)
#         if (len(ids) == 2):
#             centers = np.zeros((2, 2))
#             for i in range(len(ids)):
#                 marker_corners = corners[i][0]  # Shape: (4, 2)
#                 centers[ids[i][0]] = np.array([int(np.mean(marker_corners[:, 0])),
#  int(np.mean(marker_corners[:, 1]))])
#             centers = centers.astype(np.int64)
#             car_center = (int(np.mean(centers[:, 0])), int(np.mean(centers[:, 1])))
#             # undistorted_frame = cv2.arrowedLine(undistorted_frame, car_center,
#  tuple(centers[0]), (0, 0, 255), 5)
#             # cv2.circle(undistorted_frame, car_center, 10, (0, 255, 0), -1)
#             # print(car_center)
#             # cv2.imwrite("./img2.png", undistorted_frame)
#             pose = frame_to_world(car_center, tvec, rvec, mtx, 0.15)
#             print(pose)
#             car_centers.append(pose)

#             font                   = cv2.FONT_HERSHEY_SIMPLEX
#             bottomLeftCornerOfText = (30,60)
#             fontScale              = 2
#             fontColor              = (250,0,255)
#             thickness              = 2
#             lineType               = 2

#             # speed = (np.linalg.norm(np.array(prev_pose) - np.array(pose)))
# / ((frame_num - prev_frame_num) / 30.0)
#             # cv2.putText(undistorted_frame, f'speed {round(speed, 2)}',
#             #     bottomLeftCornerOfText,
#             #     font,
#             #     fontScale,
#             #     fontColor,
#             #     thickness,
#             #     lineType)

#             prev_pose = pose
#             prev_frame_num = frame_num

#     # Display the frame with detected markers
#     # cv2.imshow('ArUco Marker Detection', undistorted_frame)
#     # Break the loop if 'q' is pressed
#     # key = cv2.waitKey(1)
#     # # Exit on pressing the 'Esc' key
#     # if key == 27:
#     #     break
#     frame_num += 1


# with open("./poses.pickle", "wb") as file:
#     pickle.dump(car_centers, file)

# # # Release the video capture object and close all OpenCV windows
# # cap.release()
# # cv2.destroyAllWindows()
