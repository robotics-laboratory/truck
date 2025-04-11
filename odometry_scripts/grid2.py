import json

import cv2
import numpy as np

input = []

grid_len = 0.6
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

with open("grid.json", "r") as input_file:
    input_str = input_file.read()

input = json.loads(input_str)

grid_coords = [
    [float(point["x_grid"]) * grid_len, float(point["y_grid"]) * grid_len, 0.0]
    for point in input
]
img_coords = [[float(point["x_img"]), float(point["y_img"])] for point in input]

ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    [np.array(grid_coords, dtype=np.float32).reshape(-1, 1, 3)],
    [np.array(img_coords, dtype=np.float32).reshape(-1, 1, 2)],
    (4000, 3000),
    camera_matrix,
    distortion_coeffs,
    flags=cv2.CALIB_USE_INTRINSIC_GUESS,
)

with open("params.json", "w+") as params_file:
    params_str = params_file.read()
    if params_str:
        params = json.loads(params_str)
    else:
        params = {}
    params["tvecs"] = str(tvecs)
    params["rvecs"] = str(rvecs)

    params_file.seek(0)
    print(json.dumps(params), file=params_file)


print(ret, camera_matrix, dist_coeffs, rvecs, tvecs, sep="\n")
rvecs = [a[0] for a in rvecs[0]]
tvecs = [a[0] for a in tvecs[0]]

print(rvecs, tvecs)
R, _ = cv2.Rodrigues(np.asarray(rvecs))
# Транспонируем R (обратная матрица поворота)
R_inv = R.T
# Инвертируем трансляцию
camera_position = -np.dot(R_inv, np.asarray(tvecs))
rvec_inv, _ = cv2.Rodrigues(R_inv)
print(camera_position)
print(rvec_inv)
