import argparse
import json

import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
import matplotlib.transforms as transf
from matplotlib.colors import Normalize
from matplotlib import cm
import numpy as np
from matplotlib.animation import FuncAnimation
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from scipy.ndimage import gaussian_filter1d
from scipy.signal import savgol_filter
from matplotlib.widgets import Slider

WHEEL_RADIUS_M = 0.04913
HALF_WHEEL_BASE_WIDTH = 0.265 / 2.0
WHEEL_BASE_LENGTH = 0.405
ENCODERS_RPS = 50

parser = argparse.ArgumentParser(description="Plot car odometry")
parser.add_argument(
    "-p", "--path", default="./", help="Path to folder with experiment data."
)

args = parser.parse_args()

with open(args.path + "/params.json", "r") as params_file:
    params = json.load(params_file)
    fps = params["fps"]
    grid_size = params["grid_size"]
    blink_frame_num = params["blink_frame_num"]

def read_json_poses(path, blink_frame):
    data = json.load(open(path))
    result = []
    blink_offset = blink_frame / fps
    for item in data:
        x, y, velocity, orientation = None, None, None, None
        if item["x_pos"] is not None:
            x, y, velocity, orientation = item["x_pos"], item["y_pos"], item["velocity"], item["orientation"] - np.pi/2
        ts = 1 / fps * int(item["frame_num"]) - blink_offset
        result.append((ts, x, y, velocity, orientation))
    return result

def read_mcap_poses(path):
    mcap = []
    ts0 = None
    magn_data = []

    targ_curv = []
    back_wheel_speed = []
    front_wheel_data = []
    rear_wheel_data = []

    with open(path, "rb") as mcap_file:
        reader = make_reader(mcap_file, decoder_factories=[DecoderFactory()])

        for _, _, message, ros_msg in reader.iter_decoded_messages(
            topics=["/control/mode"]
        ):
            mode = ros_msg.mode
            if mode == 1: # REMOTE mode
                ts0 = message.publish_time / 10**9
                break

        for _, _, message, ros_msg in reader.iter_decoded_messages(
            topics=["/control/command"]
        ):
            targ_curv.append((message.publish_time / 10**9 - ts0, ros_msg.curvature))

        for _, _, message, ros_msg in reader.iter_decoded_messages(
            topics=["/ekf/odometry/filtered"]
        ):
            x = ros_msg.pose.pose.position.x
            y = ros_msg.pose.pose.position.y
            curv = ros_msg.twist.twist.angular.z / ros_msg.twist.twist.linear.x if ros_msg.twist.twist.linear.x > 0.05 else 0

            mcap.append((message.publish_time / 10**9 - ts0, x, y, curv))
        for _, _, message, ros_msg in reader.iter_decoded_messages(
            topics=["/imu/mag"]
        ):
            magn_data.append((message.publish_time / 10**9 - ts0, ros_msg.x, ros_msg.y, ros_msg.z))

        for _, _, message, ros_msg in reader.iter_decoded_messages(
            topics=["/hardware/telemetry"]
        ):
            front_wheel_data.append((message.publish_time / 10**9 - ts0,
                                     ros_msg.front_left_wheel_velocity, ros_msg.front_right_wheel_velocity,
                                     ros_msg.current_left_steering, ros_msg.current_right_steering))
            rear_wheel_data.append((message.publish_time / 10**9 - ts0,
                                    ros_msg.rear_left_wheel_velocity, ros_msg.rear_right_wheel_velocity))

        for _, _, message, ros_msg in reader.iter_decoded_messages(
            topics=["/hardware/wheel/odometry"]
        ):
            back_wheel_speed.append((message.publish_time / 10**9 - ts0, ros_msg.twist.twist.linear.x))

    return mcap, magn_data, front_wheel_data, back_wheel_speed, targ_curv, rear_wheel_data

def sync_mcap_poses(gt_poses, mcap_poses):
    mcap_index = 0
    sync_mcap = []
    for gt_ts, _, _, _, _ in gt_poses:
        while mcap_index < len(mcap_poses):
            mcap_ts, mcap_x, mcap_y, orientation = mcap_poses[mcap_index]
            if mcap_ts > gt_ts:
                sync_mcap.append((mcap_ts, mcap_x, mcap_y, orientation))
                break
            mcap_index += 1

    return sync_mcap

def sync_wheel_speed(front_wheels_speed, back_wheels_speed):
    back_index = 0
    sync_back_wheels_speed = []
    for front_ts, _, _, _, _ in front_wheels_speed:
        while back_index < len(back_wheels_speed):
            back_ts, speed = back_wheels_speed[back_index]
            if back_ts > front_ts:
                sync_back_wheels_speed.append((back_ts, speed))
                break
            back_index += 1

    return sync_back_wheels_speed

def compute_curvature_from_orientation(gt_poses, smooth_sigma=2):
    x = gaussian_filter1d(gt_poses[:,1], sigma=smooth_sigma)
    y = gaussian_filter1d(gt_poses[:,2], sigma=smooth_sigma)
    orient = gaussian_filter1d(np.unwrap(gt_poses[:,4]), sigma=smooth_sigma)

    ds = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    s = np.concatenate([[0], np.cumsum(ds)])

    dtheta = np.gradient(orient)
    ds_path = np.gradient(s)
    curvature = dtheta / ds_path

    return curvature

def calculate_trajectory(times, curvatures, speeds, x0=-(WHEEL_BASE_LENGTH/2), y0=0.0, theta0=0.0):
    n = len(times)

    # x = np.zeros(n)
    # y = np.zeros(n)
    # theta = np.zeros(n)

    # theta[0] = theta0

    # dt = np.diff(times)

    # k_mid = (curvatures[:-1] + curvatures[1:]) * 0.5
    # v_mid = (speeds[:-1] + speeds[1:]) * 0.5

    # dtheta = k_mid * v_mid * dt

    # theta[1:] = np.cumsum(dtheta) + theta0

    # theta_mid = (theta[:-1] + theta[1:]) * 0.5

    # dx = v_mid * np.cos(theta_mid) * dt
    # dy = v_mid * np.sin(theta_mid) * dt

    # x[0] = x0 + np.cos(theta[0]) * (WHEEL_BASE_LENGTH/2)
    # y[0] = y0 + np.sin(theta[0]) * (WHEEL_BASE_LENGTH/2)
    # x[1:] = x0 + np.cumsum(dx) + np.cos(theta[1:]) * (WHEEL_BASE_LENGTH/2)
    # y[1:] = y0 + np.cumsum(dy) + np.sin(theta[1:]) * (WHEEL_BASE_LENGTH/2)

    # return times, x, y, theta

    dt = np.diff(times)

    # Инициализация массивов
    x_rear = np.zeros(n)
    y_rear = np.zeros(n)
    theta_abs = np.zeros(n)  # Теперь хранит абсолютные углы

    # Начальные условия
    x_rear[0] = x0
    y_rear[0] = y0
    theta_abs[0] = theta0

    for i in range(n-1):
        # Средние значения параметров на интервале
        k_mid = (curvatures[i] + curvatures[i+1]) / 2
        v_mid = (speeds[i] + speeds[i+1]) / 2

        # Изменение угла (относительное)
        dtheta = k_mid * v_mid * dt[i]

        # Абсолютный угол (с нормализацией в [-π, π])
        theta_abs[i+1] = (theta_abs[i] + dtheta + np.pi) % (2*np.pi) - np.pi

        # Средний угол на интервале для расчета перемещения
        theta_avg = (theta_abs[i] + theta_abs[i+1]) / 2

        # Приращение координат задней оси
        dx = v_mid * np.cos(theta_avg) * dt[i]
        dy = v_mid * np.sin(theta_avg) * dt[i]

        x_rear[i+1] = x_rear[i] + dx
        y_rear[i+1] = y_rear[i] + dy

    # Расчет координат центра автомобиля
    center_x = x_rear + (WHEEL_BASE_LENGTH/2) * np.cos(theta_abs)
    center_y = y_rear + (WHEEL_BASE_LENGTH/2) * np.sin(theta_abs)

    return times, center_x, center_y, theta_abs

def apply_new_filter(rear_wheel_speed, original_alpha=0.1, new_alpha=0.5):
    # 1. Находим моменты изменений (уникальные точки)
    left_diff = np.diff(rear_wheel_speed[:, 1], prepend=rear_wheel_speed[0, 1] - 1)
    right_diff = np.diff(rear_wheel_speed[:, 2], prepend=rear_wheel_speed[0, 2] - 1)
    mask = (left_diff != 0) | (right_diff != 0)

    ts_unique = rear_wheel_speed[:, 0][mask]
    left_unique = rear_wheel_speed[:, 1][mask]
    right_unique = rear_wheel_speed[:, 2][mask]

    # 2. Восстанавливаем исходные значения (до старого ФНЧ)
    original_left = np.zeros_like(left_unique)
    original_left[0] = left_unique[0]
    original_left[1:] = (left_unique[1:] - (1 - original_alpha) * left_unique[:-1]) / original_alpha

    original_right = np.zeros_like(right_unique)
    original_right[0] = right_unique[0]
    original_right[1:] = (right_unique[1:] - (1 - original_alpha) * right_unique[:-1]) / original_alpha

    # 3. Применяем новый ФНЧ только к уникальным точкам
    new_left = np.zeros_like(left_unique)
    new_right = np.zeros_like(right_unique)

    new_left[0] = original_left[0]
    new_right[0] = original_right[0]

    for i in range(1, len(ts_unique)):
        new_left[i] = (1 - new_alpha) * new_left[i-1] + new_alpha * original_left[i]
        new_right[i] = (1 - new_alpha) * new_right[i-1] + new_alpha * original_right[i]

    # 4. Интерполируем дублированные значения (заполняем последним предсказанным)
    new_left_full = np.interp(rear_wheel_speed[:, 0], ts_unique, new_left)
    new_right_full = np.interp(rear_wheel_speed[:, 0], ts_unique, new_right)

    # Возвращаем в том же формате, что и входные данные
    return np.column_stack((rear_wheel_speed[:, 0], new_left_full, new_right_full))

gt_poses = read_json_poses(args.path + "/odom_grid.json", blink_frame_num)
mcap_poses, magn_data, front_wheels_speed, back_wheels_speed, targ_curv, rear_wheel_speed = read_mcap_poses(args.path + "/odom_live.mcap")
mcap_poses = sync_mcap_poses(gt_poses, mcap_poses)
back_wheels_speed = sync_wheel_speed(front_wheels_speed, back_wheels_speed)

gt_poses = np.asarray(gt_poses)
gt_curv = compute_curvature_from_orientation(gt_poses)
gt_curv = savgol_filter(gt_curv, window_length=10, polyorder=3)
# gt_curv_rad[gt_curv_rad > 5] = 100000

gt_poses = np.column_stack((gt_poses, gt_curv)) # GT Trajectory [ts, x, y, speed, orientation, curvate]

front_wheels_speed = np.asarray(front_wheels_speed)  # Front wheel speed [ts, left, right]
back_wheels_speed = np.asarray(back_wheels_speed) # Motor speed [ts, speed]
# rear_wheel_speed = np.asarray(rear_wheel_speed)
rear_wheel_speed = apply_new_filter(np.asarray(rear_wheel_speed))

front_wheels_speed[:, 1:3] *= (2 * np.pi / ENCODERS_RPS * WHEEL_RADIUS_M)
rear_wheel_speed[:, 1:3] *= (2 * np.pi / ENCODERS_RPS * WHEEL_RADIUS_M)  # Rear wheel speed [ts, left, right]

V_l = front_wheels_speed[:, 1]
V_r = front_wheels_speed[:, 2]

Vr_l = rear_wheel_speed[:, 1]
Vr_r = rear_wheel_speed[:, 2]

V_x_l = np.empty_like(V_l) # расчет скорости по левому колесу

R = np.column_stack((
    HALF_WHEEL_BASE_WIDTH * ((V_r**2 + V_l**2)/(V_r**2 + V_l**2) + np.sqrt(4 * (V_r**2 * V_l**2) / (V_l**2 - V_r**2)**2 - (WHEEL_BASE_LENGTH/(2.0*HALF_WHEEL_BASE_WIDTH))**2)), #Left turn
    HALF_WHEEL_BASE_WIDTH * ((V_r**2 + V_l**2)/(V_r**2 + V_l**2) + np.sqrt(4 * (V_r**2 * V_l**2) / (V_r**2 - V_l**2)**2 - (WHEEL_BASE_LENGTH/(2.0*HALF_WHEEL_BASE_WIDTH))**2))  #Right turn
    ))

velocity_diff = np.abs(V_l-V_r)
is_turning = velocity_diff > 0.02

curvate_odom = np.zeros_like(V_l)
curvate_odom[is_turning] = (V_r[is_turning]**2 - V_l[is_turning]**2) / (2.0 * HALF_WHEEL_BASE_WIDTH * 2.0 * back_wheels_speed[:,1][is_turning]**2) # Curvate from front wheels [ts, curv]

# curvate_odom = savgol_filter(curvate_odom, window_length=13, polyorder=3)

vr_diff = np.abs(Vr_l-Vr_r)
is_moving = (Vr_l > 0.05) & (Vr_l > 0.05)
curv_back_wheels = np.zeros_like(Vr_l)
curv_back_wheels[is_moving] = ((Vr_l[is_moving] - Vr_r[is_moving]) / ((Vr_l[is_moving] + Vr_r[is_moving]) * HALF_WHEEL_BASE_WIDTH))

traj = np.asarray(calculate_trajectory(rear_wheel_speed[:, 0], -0.945 * curv_back_wheels, 1.025*(Vr_l+Vr_r)/2)).T # Trajectory rear wheels odometry [ts, x, y, orientation]

V_x_l[~is_turning] = (V_l[~is_turning] + V_r[~is_turning]) / 2.0
V_x_r = V_x_l.copy() # расчет скорости по правому колесу

# Расчет скорости при повороте налево по левому колесу
V_x_l[is_turning & (V_l < V_r)] = V_l[is_turning & (V_l < V_r)] * R[:, 0][is_turning & (V_l < V_r)] / np.sqrt((R[:, 0][is_turning & (V_l < V_r)] - HALF_WHEEL_BASE_WIDTH) ** 2 + WHEEL_BASE_LENGTH**2)
# Расчет скорости при повороте направо по левому колесу
V_x_l[is_turning & (V_l > V_r)] = V_l[is_turning & (V_l > V_r)] * R[:, 1][is_turning & (V_l > V_r)] / np.sqrt((R[:, 1][is_turning & (V_l > V_r)] + HALF_WHEEL_BASE_WIDTH) ** 2 + WHEEL_BASE_LENGTH**2)

# Расчет скорости при повороте налево по правому колесу
V_x_r[is_turning & (V_l < V_r)] = V_r[is_turning & (V_l < V_r)] * R[:, 0][is_turning & (V_l < V_r)] / np.sqrt((R[:, 0][is_turning & (V_l < V_r)] + HALF_WHEEL_BASE_WIDTH) ** 2 + WHEEL_BASE_LENGTH**2)
# Расчет скорости при повороте направо по правому колесу
V_x_r[is_turning & (V_l > V_r)] = V_r[is_turning & (V_l > V_r)] * R[:, 1][is_turning & (V_l > V_r)] / np.sqrt((R[:, 0][is_turning & (V_l > V_r)] - HALF_WHEEL_BASE_WIDTH) ** 2 + WHEEL_BASE_LENGTH**2)

V_x = np.column_stack((front_wheels_speed[:, 0], V_x_l, V_x_r)) # Linear speed calculated from front wheel (ts, speed_frm_left, spped_from_right)

def plot_all():
    def update(val):
        mask_1_ = (gt_poses[:, 0] > slider_start.val) & (gt_poses[:, 0] < slider_end.val)
        mask_2_ = (traj[:, 0] > slider_start.val) & (traj[:, 0] < slider_end.val)
        mask_3_ = (mcap_poses_np[:, 0] > slider_start.val) & (mcap_poses_np[:, 0] < slider_end.val)

        traj_new = np.asarray(calculate_trajectory(rear_wheel_speed[:, 0], -slider_curvate.val*curv_back_wheels, slider_velocity.val*(Vr_l+Vr_r)/2)).T
        # traj_new = np.asarray(calculate_trajectory(rear_wheel_speed[:, 0], slider_curvate.val*curvate_odom, slider_velocity.val*back_wheels_speed[:,1])).T

        delta_angle = gt_poses[mask_1_][0, 4] - traj_new[mask_2_][0, 3] + np.pi/2.0
        delta_angle = (delta_angle + np.pi) % (2 * np.pi) - np.pi

        cos_theta = np.cos(delta_angle)
        sin_theta = np.sin(delta_angle)
        rotation_matrix = np.array([
            [cos_theta, -sin_theta],
            [sin_theta,  cos_theta]
        ])

        print(f'angles {gt_poses[mask_1_][0, 4]} - {traj_new[mask_2_][0, 3]}')

        traj_masked = traj_new[mask_2_].copy()
        traj_masked[:,1:3] -= traj_masked[0,1:3]
        traj_masked[:,1:3] = traj_masked[:,1:3] @ rotation_matrix.T
        traj_masked[:,1:3] += gt_poses[mask_1_][0,1:3]

        line_odom.set_data(traj_masked[:, 1], traj_masked[:, 2])
        line_gt.set_data(gt_poses[mask_1_][:, 1], gt_poses[mask_1_][:, 2])
        line_ekf.set_data(mcap_poses_np[mask_3_][:, 1], mcap_poses_np[mask_3_][:, 2])

        fig.canvas.draw_idle()

    start_sec = 0
    stop_sec = 300

    fig, ax = plt.subplots()
    fig.subplots_adjust(bottom=0.12)
    ax.xaxis.set_major_locator(plticker.MultipleLocator(base=grid_size))
    ax.yaxis.set_major_locator(plticker.MultipleLocator(base=grid_size))
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)

    ax_curvate = fig.add_axes([0.2, 0.01, 0.6, 0.03])  # [left, bottom, width, height]
    slider_curvate = Slider(ax_curvate, 'Кривизна', 0.9, 1.1, valinit=0.947)

    ax_velocity = fig.add_axes([0.2, 0.03, 0.6, 0.03])
    slider_velocity = Slider(ax_velocity, 'Скорость', 0.9, 1.1, valinit=1.025)

    ax_start = fig.add_axes([0.2, 0.05, 0.6, 0.03])
    slider_start = Slider(ax_start, 'Начало', 0, 300, valinit=0)

    ax_end = fig.add_axes([0.2, 0.07, 0.6, 0.03])
    slider_end = Slider(ax_end, 'Конец', 0, 300, valinit=300)

    fig.add_axes([0.2, -0.1, 0.6, 0.001])

    mcap_poses_np = np.asarray(mcap_poses)

    # norm = Normalize(vmin=np.min(gt_poses[:, 0]), vmax=np.max(gt_poses[:, 0]))
    # cmap = cm.viridis

    mask_1 = (gt_poses[:, 0] > start_sec) & (gt_poses[:, 0] < stop_sec)
    mask_2 = (traj[:, 0] > start_sec) & (traj[:, 0] < stop_sec)
    mask_3 = (mcap_poses_np[:, 0] > start_sec) & (mcap_poses_np[:, 0] < stop_sec)
    # sc = ax.scatter(gt_poses_np[:, 1], gt_poses_np[:, 2], c=gt_poses_np[:, 0], cmap=cmap, norm=norm, s=5)
    # ax.plot(gt_poses_np[:, 1][(gt_poses[:, 0] > 17) & (gt_poses[:, 0] < 25)], gt_poses_np[:, 2][(gt_poses[:, 0] > 17) & (gt_poses[:, 0] < 25)], "-", linewidth=2, label='GT')
    line_gt, = ax.plot(gt_poses[mask_1][:, 1], gt_poses[mask_1][:, 2], "-", linewidth=2, label='GT')
    line_ekf, = ax.plot(mcap_poses_np[mask_3][:, 1], mcap_poses_np[mask_3][:, 2], "-", linewidth=1, label='EKF')
    line_odom, = ax.plot(traj[mask_2][:, 1], traj[mask_2][:, 2], "-", linewidth=1, label='Odom')
    # ax.plot(traj2[0], traj2[1], "-", linewidth=1, label='Odom')
    # cbar = plt.colorbar(sc, label="Время (с)")

    slider_curvate.on_changed(update)
    slider_velocity.on_changed(update)
    slider_start.on_changed(update)
    slider_end.on_changed(update)
    # plt.axis("equal")
    plt.grid()
    plt.show()


def plot_sequential():
    step = 30 * fps
    offset = 400

    while offset < len(gt_poses):
        fig, ax = plt.subplots()
        ax.xaxis.set_major_locator(plticker.MultipleLocator(base=grid_size))
        ax.yaxis.set_major_locator(plticker.MultipleLocator(base=grid_size))

        RANGE = slice(offset, offset + step)
        gt_poses_np = np.asarray(gt_poses)
        mcap_poses_np = np.asarray(mcap_poses)
        ax.plot(gt_poses_np[RANGE, 1], gt_poses_np[RANGE, 2], "-", linewidth=1)
        ax.plot(mcap_poses_np[RANGE, 1], mcap_poses_np[RANGE, 2], "-", linewidth=1)

        plt.axis("equal")
        plt.grid()
        plt.show()

        offset += step


def plot_animation():
    print(gt_poses.shape, traj.shape)
    fig, ax = plt.subplots(1, 1)
    ax.set_title("trajectory")
    # ax2.set_title("position drift")
    ax.xaxis.set_major_locator(plticker.MultipleLocator(base=grid_size))
    ax.yaxis.set_major_locator(plticker.MultipleLocator(base=grid_size))

    [gt_line] = ax.plot([], [], lw=1, label="gt")
    [mcap_line] = ax.plot([], [], lw=1, label="odom")
    # [diff_line] = ax2.plot([], [], lw=1)
    mcap_poses_np = np.asarray(traj, dtype=np.float32)
    # pose_diff = gt_poses_np[:,:3] - traj

    start_sec = 0
    stop_sec = 180
    def animate(n):
        mask_1 = (gt_poses[:, 0] > start_sec) & (gt_poses[:, 0] < n/10.0)
        mask_2 = (traj[:, 0] > start_sec) & (traj[:, 0] < n/10.0)
        gt_line.set_xdata(gt_poses[mask_1][:, 1])
        gt_line.set_ydata(gt_poses[mask_1][:, 2])
        mcap_line.set_xdata(traj[mask_2][:, 1])
        mcap_line.set_ydata(traj[mask_2][:, 2])
        # diff_line.set_xdata(pose_diff[cut, 1])
        # diff_line.set_ydata(pose_diff[cut, 2])
        return [gt_line, mcap_line]

    frames = range(0, (stop_sec - start_sec)*10, 1)
    anim = FuncAnimation(fig, animate, frames=frames, interval=10)  # noqa

    ax.set_xlim(np.min(traj[:, 1]) - 1, np.max(traj[:, 1]) + 1)
    ax.set_ylim(np.min(traj[:, 2]) - 1, np.max(traj[:, 2]) + 1)
    ax.set_aspect("equal", adjustable="box")
    ax.grid()
    ax.legend()

    fig.set_size_inches(20, 10)
    fig.tight_layout()
    # anim.save("anim.mp4")
    plt.show()


def plot_animation_with_orientation():
    fig, [ax, ax2] = plt.subplots(1, 2, figsize=(20, 10))
    ax.set_title("Trajectory")
    ax2 = fig.add_subplot(122, projection='polar')
    ax2.set_title("Orientation Comparison (MCAP vs Magnetometer)")

    ax.xaxis.set_major_locator(plticker.MultipleLocator(base=grid_size))
    ax.yaxis.set_major_locator(plticker.MultipleLocator(base=grid_size))

    [gt_line] = ax.plot([], [], lw=1, label="GT")
    [mcap_line] = ax.plot([], [], lw=1, label="Odometry")

    gt_poses_np = np.asarray(gt_poses[70*30:88*30], dtype=np.float32)
    mcap_poses_np = np.asarray(mcap_poses[70*30:88*30], dtype=np.float32)

    mcap_orientation = mcap_poses_np[:, 3]

    def animate(n):
        cut = slice(0, n)
        gt_line.set_xdata(gt_poses_np[cut, 1])
        gt_line.set_ydata(gt_poses_np[cut, 2])
        mcap_line.set_xdata(mcap_poses_np[cut, 1])
        mcap_line.set_ydata(mcap_poses_np[cut, 2])

        ax2.clear()
        ax2.set_title("Orientation Comparison (MCAP vs Magnetometer)")

        ax2.set_theta_offset(0)
        ax2.set_theta_direction(-1)
        ax2.plot([0, mcap_orientation[n]], [0, 1], label="MCAP Orientation", color="r", linewidth=2)
        ax2.plot([0, magn_data[n][1]], [0, 1], label="Magnetometer Orientation", color="b", linewidth=1)

        ax2.set_ylim(0, 1)
        ax2.legend(loc='upper right')
        ax2.set_yticklabels([])
        ax2.set_xticks(np.radians(np.linspace(0, 360, 9)))  # Отображаем углы
        ax2.set_xticklabels([f"{i}°" for i in range(0, 361, 45)])

        return [gt_line, mcap_line]

    frames = range(0, len(gt_poses_np), 10)
    anim = FuncAnimation(fig, animate, frames=frames, interval=250)

    ax.set_xlim(np.min(mcap_poses_np[:, 1]) - 1, np.max(mcap_poses_np[:, 1]) + 1)
    ax.set_ylim(np.min(mcap_poses_np[:, 2]) - 1, np.max(mcap_poses_np[:, 2]) + 1)
    ax.set_aspect("equal", adjustable="box")
    ax.grid()
    ax.legend()

    ax2.set_xlim(-1, 1)
    ax2.set_ylim(0, 1)
    ax2.set_aspect("equal", adjustable="box")
    ax2.grid()

    fig.tight_layout()
    # anim.save("./orientation_comparison_anim.mp4")
    plt.show()


def plot_magn():
    global sync_magn_data
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    sync_magn_data = np.asarray(sync_magn_data)
    norm = Normalize(vmin=np.min(sync_magn_data[70*30:88*30, 0]), vmax=np.max(sync_magn_data[70*30:88*30, 0]))
    cmap = cm.viridis
    sc = ax.scatter(sync_magn_data[70*30:88*30, 1], sync_magn_data[70*30:88*30, 2], sync_magn_data[70*30:88*30, 3], c=sync_magn_data[70*30:88*30, 0], cmap=cmap, norm=norm, marker='o')

    plt.colorbar(sc, label="ts")
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

    plt.show()


def compare_speed_curvate():
    targ_curv_np = np.asarray(targ_curv)
    mcap_poses_np = np.asarray(mcap_poses)

    plt.figure(figsize=(8, 5))

    R_disp = np.zeros(shape=(len(R)))
    R_disp[is_turning & (V_l > V_r)] -= R[:, 1][is_turning & (V_l > V_r)]
    R_disp[is_turning & (V_r > V_l)] += R[:, 0][is_turning & (V_r > V_l)]

    plt.plot(gt_poses[:,0], gt_poses[:,3], label='Cкорость GT')
    # plt.plot(V_x[:,0], (V_x[:,1] + V_x[:,2])/2.0, label='Скорость одометрия')
    plt.plot(back_wheels_speed[:, 0], back_wheels_speed[:, 1], label='Скорость по двигателю')
    # plt.plot(rear_wheel_speed[:, 0], (rear_wheel_speed[:, 1] + rear_wheel_speed[:, 2])/2.0, label='Скорость задние колеса')
    # new_rear_wheel_speed = apply_new_filter(rear_wheel_speed)
    new_rear_wheel_speed = rear_wheel_speed
    plt.plot(new_rear_wheel_speed[:, 0], (new_rear_wheel_speed[:, 1] + new_rear_wheel_speed[:, 2])/2.0, label='Скорость задние колеса')

    plt.plot(new_rear_wheel_speed[:, 0], new_rear_wheel_speed[:, 1], label='Нов Левое колесо')
    plt.plot(new_rear_wheel_speed[:, 0], new_rear_wheel_speed[:, 2], label='Нов Правое колесо')
    # plt.plot(rear_wheel_speed[:, 0], rear_wheel_speed[:, 1], label='Левое колесо')
    # plt.plot(rear_wheel_speed[:, 0], rear_wheel_speed[:, 2], label='Правое колесо')
    plt.plot(V_x[:, 0], curvate_odom, label='Кривизна одометрия')
    # plt.plot(V_x[:, 0], 1 / R_disp, label='Кривизна одометрия 2')

    plt.plot(targ_curv_np[:, 0], targ_curv_np[:, 1], label='Кривизна целевая')
    plt.plot(gt_poses[:, 0], gt_poses[:, 5], label='Кривизна GT')
    # plt.plot(mcap_poses_np[:, 0], mcap_poses_np[:, 3], label='Кривизна EKF')
    plt.plot(rear_wheel_speed[:, 0], -0.945*(curv_back_wheels), label='Кривизна задние колеса')

    # plt.plot(rear_wheel_speed[:, 0], rear_wheel_speed[:, 1], label='Скрость лев зад колеса')
    # plt.plot(rear_wheel_speed[:, 0], rear_wheel_speed[:, 2], label='Скрость прав зад колеса')

    plt.title('Сравнение рассчитанных скоростей и кривизны',fontsize=15)
    plt.xlabel('Время (с)',fontsize=15)
    plt.ylabel('Скорость (м/с) / Кривизна (м^-1)',fontsize=15)
    plt.ylim(-2, 2)
    plt.grid(True)
    plt.rc('legend',fontsize=15)
    plt.legend()
    plt.show()

# plot_animation_with_orientation()
plot_all()
# plot_magn()
# plot_sequential()
# plot_animation()
# compare_speed_curvate()
