import argparse
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


parser = argparse.ArgumentParser(description="Calculate car odometry")
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
    tvec = np.asarray(params["tvecs"])
    rvec = np.asarray(params["rvecs"])
    distortion_coeffs = np.asarray(params["grid_camera_dist"], dtype=np.float32)
    camera_matrix = np.asarray(params["grid_camera_matrix"], dtype=np.float32)
    width, height = params["frame_size"]

with open(args.path + "/markers.json", "r") as markers_file:
    input_markers = json.load(markers_file)

output = []

ALPHA = 0.6
mtx, _ = cv2.getOptimalNewCameraMatrix(
    camera_matrix, distortion_coeffs, (width, height), ALPHA
)

for frame in input_markers:
    frame_num = frame["frame_num"]
    car_center = (None, None)
    if len(frame["centers"]) == 2:
        front_marker = frame["centers"][0]
        back_marker = frame["centers"][1]
        car_center = (None, None)
        if (front_marker["marker_id"] == 1) and (back_marker["marker_id"] == 0):
            front_marker, back_marker = back_marker, front_marker

        undist_front = cv2.undistortPoints(
            np.asarray(
                [[front_marker["x_px"], front_marker["y_px"]]],
                dtype=np.float32,
            ),
            camera_matrix,
            distortion_coeffs,
            None,
            mtx,
        )
        undist_back = cv2.undistortPoints(
            np.asarray(
                [[back_marker["x_px"], back_marker["y_px"]]],
                dtype=np.float32,
            ),
            camera_matrix,
            distortion_coeffs,
            None,
            mtx,
        )

        front_pose = frame_to_world(
            undist_front[0][0],
            tvec,
            rvec,
            mtx,
            0.15,
        )
        back_pose = frame_to_world(
            undist_back[0][0],
            tvec,
            rvec,
            mtx,
            0.15,
        )
        car_center = ((np.array(front_pose) + np.array(back_pose)) / 2).tolist()

    output.append(
        {
            "frame_num": frame_num,
            "x_pos": car_center[0],
            "y_pos": car_center[1],
        }
    )

with open(args.path + "/odom_grid.json", "w+") as output_file:
    json.dump(output, output_file)
