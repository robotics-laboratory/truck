import argparse
import json
import os

import cv2
import numpy as np

# Constants
DRAG_THRESHOLD = 10
MOVE_STEP_PX = 100
SCALE_FACTOR = 1.1
GRID_SIZE = 20

parser = argparse.ArgumentParser(description="Select grid nodes on video frame.")
parser.add_argument(
    "-p",
    "--path",
    default="./",
    required=True,
    help="Path to folder with experiment data.",
)
parser.add_argument(
    "-v",
    "--view",
    default=None,
    help="Filename of JSON (in path folder) with grid nodes to view/edit grid label",
)

args = parser.parse_args()

frame_path = args.path + "/frame.png"
if not os.path.exists(frame_path):
    raise Exception(f"No image file {frame_path}")

if args.view and (not os.path.exists(args.path + "/" + args.view)):
    raise Exception(f"No JSON file {args.path + '/' + args.view}")

orig_image = cv2.imread(frame_path)

zoom = 1.0
offset = np.array([0.0, 0.0])
display_height, display_width = orig_image.shape[:2]

corner_points = []

# Variables for drag
dragging = False
selected_point_idx = None


def image_to_display(pt):
    p_scaled = (pt[0] * zoom, pt[1] * zoom)

    return (p_scaled[0] - offset[0], p_scaled[1] - offset[1])


def display_to_image(pt):
    return ((pt[0] + offset[0]) / zoom, (pt[1] + offset[1]) / zoom)


def recalculate_predictions():
    global corner_points, corners

    manual_img_points = np.array([pt[:2] for pt in corner_points if pt[2] == 1], dtype=np.float32)
    manual_grid_points = np.array([corners[i] for i, pt in enumerate(corner_points) if pt[2] == 1], dtype=np.float32)

    if len(manual_img_points) < 4: # Need at least 4 points
        return

    H, _ = cv2.findHomography(manual_grid_points, manual_img_points)

    to_predict = [corners[i] for i, pt in enumerate(corner_points) if pt[2] == 0]
    if not to_predict:
        return

    grid_img_points = cv2.perspectiveTransform(
        np.array(to_predict, dtype=np.float32).reshape(-1, 1, 2), H
    )
    new_img_points = grid_img_points.reshape(-1, 2)

    j = 0
    for i, pt in enumerate(corner_points):
        if pt[2] == 0:
            corner_points[i][0] = new_img_points[j][0]
            corner_points[i][1] = new_img_points[j][1]
            j += 1

def redraw():
    global orig_image, corner_points, zoom, offset
    scaled_image = cv2.resize(
        orig_image, None, fx=zoom, fy=zoom, interpolation=cv2.INTER_LINEAR
    )
    scaled_h, scaled_w = scaled_image.shape[:2]

    offset[0] = max(0, min(offset[0], scaled_w - display_width))
    offset[1] = max(0, min(offset[1], scaled_h - display_height))
    display_img = scaled_image[
        int(offset[1]) : int(offset[1] + display_height),
        int(offset[0]) : int(offset[0] + display_width),
    ].copy()
    for i, pt in enumerate(corner_points):
        p_disp = image_to_display(pt)
        if 0 <= p_disp[0] < display_width and 0 <= p_disp[1] < display_height:
            if pt[2] == 1:  # dragged point
                cv2.putText(
                    display_img,
                    f"[{corners[i][0]},{corners[i][1]}]",
                    (int(p_disp[0]) - 50, int(p_disp[1]) - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 255, 0),
                    2,
                )
                color = (0, 255, 0)
            else:
                color = (0, 0, 255)
            cv2.circle(display_img, (int(p_disp[0]), int(p_disp[1])), 12, color, 2)

    cv2.imshow("Image", display_img)


def mouse_callback(event, x, y, flags, param):
    global corner_points, dragging, selected_point_idx, selected_type

    img_pt = display_to_image((x, y))

    if event == cv2.EVENT_LBUTTONDOWN:
        for i, pt in enumerate(corner_points):
            p_disp = image_to_display(pt)
            if np.hypot(p_disp[0] - x, p_disp[1] - y) < DRAG_THRESHOLD:
                dragging = True
                selected_point_idx = i
                return
        if len(corner_points) < 4:
            corner_points.append([*img_pt, 1])
    elif event == cv2.EVENT_MOUSEMOVE and dragging:
        corner_points[selected_point_idx] = [*img_pt, 1]

    elif event == cv2.EVENT_LBUTTONUP:
        dragging = False
        selected_point_idx = None
        selected_type = None
        recalculate_predictions()

    elif event == cv2.EVENT_RBUTTONDOWN:
        for i, pt in enumerate(corner_points):
            p_disp = image_to_display(pt)
            if np.hypot(p_disp[0] - x, p_disp[1] - y) < DRAG_THRESHOLD:
                corner_points[i][2] = 0
                return
    redraw()


def keyboard_routine():
    global zoom, offset
    key = cv2.waitKey(20)

    if key == ord("+") or key == ord("="):
        zoom *= SCALE_FACTOR
    elif key == ord("-") or key == ord("_"):
        if zoom < SCALE_FACTOR:
            zoom = 1
        else:
            zoom /= SCALE_FACTOR
    elif key == 2:  # left
        offset[0] -= MOVE_STEP_PX
    elif key == 3:  # right
        offset[0] += MOVE_STEP_PX
    elif key == 0:  # up
        offset[1] -= MOVE_STEP_PX
    elif key == 1:  # down
        offset[1] += MOVE_STEP_PX

    return key


cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Image", display_width, display_height)
cv2.setMouseCallback("Image", mouse_callback)
redraw()

if args.view is None:
    corners = [[0, 0], [1, 0], [1, 1], [0, 1]]

    while True:
        key = keyboard_routine()

        if key != -1:
            redraw()
        if key == 13 and len(corner_points) == 4:  # 13 == Enter (CR, carriage return)
            break
        elif key == 27:
            cv2.destroyAllWindows()
            exit()

    image_corners = np.array([pt[:2] for pt in corner_points], dtype=np.float32)
    H = cv2.getPerspectiveTransform(np.array(corners, dtype=np.float32), image_corners)

    for i in range(-GRID_SIZE // 2 + 1, GRID_SIZE // 2 + 1):
        for j in range(-GRID_SIZE // 2 + 1, GRID_SIZE // 2 + 1):
            if (i, j) in [(0, 0), (1, 0), (1, 1), (0, 1)]:
                continue
            corners.append([i, j])
    grid_img_points = cv2.perspectiveTransform(
        np.array(corners[4:], dtype=np.float32).reshape(-1, 1, 2), H
    )
    predict_points = grid_img_points.reshape(-1, 2).tolist()
    corner_points.extend([[pt[0], pt[1], 0] for pt in predict_points])
else:
    with open(args.path + "/" + args.view, "r") as input_file:
        input = json.load(input_file)

    corners = [[point["x_grid"], point["y_grid"]] for point in input]
    corner_points = [[point["x_img"], point["y_img"], 1] for point in input]

    for i in range(-GRID_SIZE // 2 + 1, GRID_SIZE // 2 + 1):
        for j in range(-GRID_SIZE // 2 + 1, GRID_SIZE // 2 + 1):
            if not([i, j] in corners):
                corners.append([i, j])
                corner_points.append([0, 0, 0])
    recalculate_predictions()

redraw()

while True:
    key = keyboard_routine()

    if key != -1:
        redraw()
    if key == 13:  # 13 == Enter (CR, carriage return)
        break

output_array = []

for i in range(len(corners)):
    if corner_points[i][2] == 0:
        continue

    if (
        (corner_points[i][0] > display_width)
        or (corner_points[i][0] < 0)
        or (corner_points[i][1] > display_height)
        or (corner_points[i][1] < 0)
    ):
        continue

    output_array.append(
        {
            "x_grid": int(corners[i][0]),
            "y_grid": int(corners[i][1]),
            "x_img": float(corner_points[i][0]),
            "y_img": float(corner_points[i][1]),
        }
    )

if args.view is None:
    json_filename = "/grid.json"
else:
    json_filename = "/" + args.view

with open(args.path + json_filename, "w+") as output_file:
    json.dump(output_array, output_file)

cv2.destroyAllWindows()
