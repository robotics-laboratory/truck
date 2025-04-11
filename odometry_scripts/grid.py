import json

import cv2
import numpy as np

# Parametres
drag_threshold = 10
move_step_px = 100
scale_factor = 1.1
grid_size = 20

video_path = "/Users/zhora/Downloads/Aruco_Odometry/run-01.mp4"
cap = cv2.VideoCapture(video_path)
ret, orig_image = cap.read()
if orig_image is None:
    raise IOError("Image not found")

zoom = 1.0
offset = np.array([0.0, 0.0])
display_height, display_width = orig_image.shape[:2]

corner_points = []

# Variables for drag
dragging = False
selected_point_idx = None


def image_to_display(pt):
    """
    Преобразует точку из координат оригинального изображения
    в координаты окна отображения.
    """
    # Масштабирование
    p_scaled = (pt[0] * zoom, pt[1] * zoom)
    # Смещение
    return (p_scaled[0] - offset[0], p_scaled[1] - offset[1])


def display_to_image(pt):
    """
    Преобразует точку из координат окна отображения
    в координаты оригинального изображения.
    """
    return ((pt[0] + offset[0]) / zoom, (pt[1] + offset[1]) / zoom)


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
    for pt in corner_points:
        p_disp = image_to_display(pt)
        if 0 <= p_disp[0] < display_width and 0 <= p_disp[1] < display_height:
            if pt[2] == 1:  # dragged point
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
            if np.hypot(p_disp[0] - x, p_disp[1] - y) < drag_threshold:
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
    elif event == cv2.EVENT_RBUTTONDOWN:
        pass
        # delete point
        # for i, pt in enumerate(other_points):
        #     p_disp = image_to_display(pt)
        #     if np.hypot(p_disp[0] - x, p_disp[1] - y) < drag_threshold:
        #         other_points = np.delete(other_points, i)
        #         return
    redraw()


def keyboard_routine():
    global zoom, offset
    key = cv2.waitKey(20)

    if key == ord("+") or key == ord("="):
        zoom *= scale_factor
    elif key == ord("-") or key == ord("_"):
        if zoom < scale_factor:
            zoom = 1
        else:
            zoom /= scale_factor
    elif key == 2:  # left
        offset[0] -= move_step_px
    elif key == 3:  # right
        offset[0] += move_step_px
    elif key == 0:  # up
        offset[1] -= move_step_px
    elif key == 1:  # down
        offset[1] += move_step_px

    return key


cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Image", display_width, display_height)
cv2.setMouseCallback("Image", mouse_callback)
redraw()

while True:
    key = keyboard_routine()

    if key != -1:
        redraw()
    if key == ord("n") and len(corner_points) == 4:
        break
    elif key == 27:
        cv2.destroyAllWindows()
        exit()

cv2.destroyAllWindows()

corners = [[0, 0], [1, 0], [1, 1], [0, 1]]

image_corners = np.array([pt[:2] for pt in corner_points], dtype=np.float32)
H = cv2.getPerspectiveTransform(np.array(corners, dtype=np.float32), image_corners)

for i in range(-grid_size // 2 + 1, grid_size // 2 + 1):
    for j in range(-grid_size // 2 + 1, grid_size // 2 + 1):
        if (i, j) in [(0, 0), (1, 0), (1, 1), (0, 1)]:
            continue
        corners.append([i, j])
grid_img_points = cv2.perspectiveTransform(
    np.array(corners[4:], dtype=np.float32).reshape(-1, 1, 2), H
)
predict_points = grid_img_points.reshape(-1, 2).tolist()
corner_points.extend([[pt[0], pt[1], 0] for pt in predict_points])


cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Image", display_width, display_height)
cv2.setMouseCallback("Image", mouse_callback)
redraw()

while True:
    key = keyboard_routine()
    if key != -1:
        redraw()
    if key == 27:
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
    if (corner_points[i][0] < 40) and (corner_points[i][1] < 40):
        continue
    output_array.append(
        {
            "x_grid": str(corners[i][0]),
            "y_grid": str(corners[i][1]),
            "x_img": str(corner_points[i][0]),
            "y_img": str(corner_points[i][1]),
        }
    )

with open("grid.json", "w+") as output_file:
    json.dump(output_array, fp=output_file)

cv2.destroyAllWindows()


# with open("grid.json", "r") as grid_file:
#     grid = json.load(grid_file)

#     corner_points = []
#     for pt in grid:
#         corner_points.append([float(pt['x_img']), float(pt['y_img'])])

#     redraw()
#     while(True):
#         key = cv2.waitKey(20)
#         pass
