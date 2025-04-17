import argparse
import json

import cv2
import numpy as np

parser = argparse.ArgumentParser(description="Calculate camera params and tvec, rvec")
parser.add_argument(
    "-p",
    "--path",
    default="./",
    required=True,
    help="Path to folder with experiment data.",
)
parser.add_argument(
    "-l",
    "--grid_size",
    default=None,
    required=True,
    help="Edge length of the grid (in metres).",
)

args = parser.parse_args()

input = []
grid_size = float(args.grid_size)

with open(args.path + "/params.json", "r") as params_file:
    try:
        params = json.load(params_file)
    except json.JSONDecodeError:
        params = {}

with open(args.path + "/grid.json", "r") as input_file:
    input = json.load(input_file)

grid_coords = [
    [point["x_grid"] * grid_size, point["y_grid"] * grid_size, 0.0] for point in input
]
img_coords = [[point["x_img"], point["y_img"]] for point in input]

if ("charuco_camera_matrix" in params.keys()) and (
    "charuco_camera_dist" in params.keys()
):
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        [np.array(grid_coords, dtype=np.float32).reshape(-1, 1, 3)],
        [np.array(img_coords, dtype=np.float32).reshape(-1, 1, 2)],
        (4000, 3000),
        np.asarray(params["charuco_camera_matrix"], dtype=np.float32),
        np.asarray(params["charuco_camera_dist"], dtype=np.float32),
        flags=cv2.CALIB_USE_INTRINSIC_GUESS,
    )
else:
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        [np.array(grid_coords, dtype=np.float32).reshape(-1, 1, 3)],
        [np.array(img_coords, dtype=np.float32).reshape(-1, 1, 2)],
        (4000, 3000),
    )

rvecs = [a[0] for a in rvecs[0]]
tvecs = [a[0] for a in tvecs[0]]

R, _ = cv2.Rodrigues(np.asarray(rvecs))
R_inv = R.T
camera_position = -np.dot(R_inv, np.asarray(tvecs))
rvec_inv, _ = cv2.Rodrigues(R_inv)

with open(args.path + "/params.json", "w+") as params_file:
    params["grid_rms"] = ret
    params["tvecs"] = camera_position.tolist()
    params["rvecs"] = rvec_inv.tolist()
    params["grid_size"] = grid_size
    params["grid_camera_matrix"] = camera_matrix.tolist()
    params["grid_camera_dist"] = dist_coeffs.flatten().tolist()
    json.dump(params, params_file)
