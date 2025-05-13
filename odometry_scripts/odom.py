import argparse
import json

import cv2
import numpy as np


def frame_to_ray(point, matrix):
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


def select_correct_markers(frame):
    front_marker = None
    back_marker = None

    for center in frame["centers"]:
        if center["marker_id"] == 0:
            front_marker = center
        elif center["marker_id"] == 1:
            back_marker = center

    return front_marker, back_marker


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

    if (tvec[2] < 0): # TODO Debug case when Z tvec is negative
        tvec *= -1

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

prev_car_center = None
for frame in input_markers:
    frame_num = frame["frame_num"]
    car_center = (None, None)
    velocity = 0.0

    front_marker, back_marker = select_correct_markers(frame)
    if front_marker and back_marker:
        car_center = (None, None)

        undist_front = np.asarray(
                [[front_marker["x_px"], front_marker["y_px"]]],
                dtype=np.float32,
            )

        undist_back = np.asarray(
                [[back_marker["x_px"], back_marker["y_px"]]],
                dtype=np.float32,
            )

        front_pose = frame_to_world(
            undist_front[0],
            tvec,
            rvec,
            mtx,
            0.15,
        )
        back_pose = frame_to_world(
            undist_back[0],
            tvec,
            rvec,
            mtx,
            0.15,
        )
        car_center = ((np.array(front_pose) + np.array(back_pose)) / 2).tolist()

        if prev_car_center:
            velocity = float(np.linalg.norm(np.asarray(car_center) - np.asarray(prev_car_center)) * 30)

        prev_car_center = car_center.copy()
        orientation_vector = np.array(front_pose) - np.array(back_pose)
        car_orientation = np.arctan2(orientation_vector[1], orientation_vector[0])
    else:
        prev_car_center = None

    output.append(
        {
            "frame_num": frame_num,
            "x_pos": car_center[0],
            "y_pos": car_center[1],
            "velocity": velocity,
            "orientation": float(car_orientation),
        }
    )

with open(args.path + "/odom_grid.json", "w+") as output_file:
    json.dump(output, output_file)
