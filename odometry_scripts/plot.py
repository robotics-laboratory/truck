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
            curv = ros_msg.twist.twist.angular.z / ros_msg.twist.twist.linear.x if ros_msg.twist.twist.linear.x > 0.1 else 0

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

        for _, _, message, ros_msg in reader.iter_decoded_messages(
            topics=["/hardware/wheel/odometry"]
        ):
            back_wheel_speed.append((message.publish_time / 10**9 - ts0, ros_msg.twist.twist.linear.x))

    return mcap, magn_data, front_wheel_data, back_wheel_speed, targ_curv

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

def compute_curvature_from_orientation(gt_poses, window_size=13, smooth_sigma=2):
    x = gaussian_filter1d(gt_poses[:,1], sigma=smooth_sigma)
    y = gaussian_filter1d(gt_poses[:,2], sigma=smooth_sigma)
    orient = gaussian_filter1d(np.unwrap(gt_poses[:,4]), sigma=smooth_sigma)

    ds = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    s = np.concatenate([[0], np.cumsum(ds)])

    dtheta = np.gradient(orient)
    ds_path = np.gradient(s)
    curvature = dtheta / ds_path

    return curvature

WHEEL_RADIUS_M = 0.0525
HALF_WHEEL_BASE_WIDTH = 0.265 / 2.0
WHEEL_BASE_LENGTH = 0.405
ENCODERS_RPS = 50

gt_poses = read_json_poses(args.path + "/odom_grid.json", blink_frame_num)
mcap_poses, magn_data, front_wheels_speed, back_wheels_speed, targ_curv = read_mcap_poses(args.path + "/odom_live.mcap")
mcap_poses = sync_mcap_poses(gt_poses, mcap_poses)
back_wheels_speed = sync_wheel_speed(front_wheels_speed, back_wheels_speed)

gt_poses = np.asarray(gt_poses)
gt_curv = compute_curvature_from_orientation(gt_poses)
gt_poses = np.column_stack((gt_poses, gt_curv))

front_wheels_speed = np.asarray(front_wheels_speed)
back_wheels_speed = np.asarray(back_wheels_speed)

front_wheels_speed[:, 1:3] *= (2 * np.pi / ENCODERS_RPS * WHEEL_RADIUS_M)

V_l = front_wheels_speed[:, 1]
V_r = front_wheels_speed[:, 2]

V_x_l = np.empty_like(V_l) # расчет скорости по левому колесу

R = np.column_stack((
    HALF_WHEEL_BASE_WIDTH * ((V_r**2 + V_l**2)/(V_r**2 + V_l**2) + np.sqrt(4 * (V_r**2 * V_l**2) / (V_l**2 - V_r**2)**2 - (WHEEL_BASE_LENGTH/(2.0*HALF_WHEEL_BASE_WIDTH))**2)), #Left turn
    HALF_WHEEL_BASE_WIDTH * ((V_r**2 + V_l**2)/(V_r**2 + V_l**2) + np.sqrt(4 * (V_r**2 * V_l**2) / (V_r**2 - V_l**2)**2 - (WHEEL_BASE_LENGTH/(2.0*HALF_WHEEL_BASE_WIDTH))**2))  #Right turn
    ))

velocity_diff = np.abs(V_l-V_r)
is_turning = velocity_diff > 0.01

curvate_odom = np.zeros_like(V_l)
curvate_odom[is_turning] = (V_r[is_turning]**2 - V_l[is_turning]**2) / (2.0 * HALF_WHEEL_BASE_WIDTH * 2.0 * back_wheels_speed[:,1][is_turning]**2)
curvate_odom = savgol_filter(curvate_odom, window_length=13, polyorder=2)

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

V_x = np.column_stack((front_wheels_speed[:, 0], V_x_l, V_x_r))


def plot_all():
    start_ts = 0
    end_ts = 1000

    fig, ax = plt.subplots()
    ax.xaxis.set_major_locator(plticker.MultipleLocator(base=grid_size))
    ax.yaxis.set_major_locator(plticker.MultipleLocator(base=grid_size))



    gt_poses_np = np.asarray(gt_poses)
    mcap_poses_np = np.asarray(mcap_poses)
    ax.plot(gt_poses_np[:, 1][(gt_poses_np[:, 0] > start_ts) & (gt_poses_np[:, 0] < end_ts)], gt_poses_np[:, 2][(gt_poses_np[:, 0] > start_ts) & (gt_poses_np[:, 0] < end_ts)], "-", linewidth=1)
    ax.plot(mcap_poses_np[:, 1][(mcap_poses_np[:, 0] > start_ts) & (mcap_poses_np[:, 0] < end_ts)], mcap_poses_np[:, 2][(mcap_poses_np[:, 0] > start_ts) & (mcap_poses_np[:, 0] < end_ts)], "-", linewidth=1)

    plt.axis("equal")
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
    fig, [ax, ax2] = plt.subplots(1, 2)
    ax.set_title("trajectory")
    ax2.set_title("position drift")
    ax.xaxis.set_major_locator(plticker.MultipleLocator(base=grid_size))
    ax.yaxis.set_major_locator(plticker.MultipleLocator(base=grid_size))

    [gt_line] = ax.plot([], [], lw=1, label="gt")
    [mcap_line] = ax.plot([], [], lw=1, label="odom")
    [diff_line] = ax2.plot([], [], lw=1)
    gt_poses_np = np.asarray(gt_poses, dtype=np.float32)
    mcap_poses_np = np.asarray(mcap_poses, dtype=np.float32)
    pose_diff = gt_poses_np - mcap_poses_np[:,:3]

    def animate(n):
        # cut = slice(n - fps * 10, n)
        cut = slice(0, n)
        gt_line.set_xdata(gt_poses_np[cut, 1])
        gt_line.set_ydata(gt_poses_np[cut, 2])
        mcap_line.set_xdata(mcap_poses_np[cut, 1])
        mcap_line.set_ydata(mcap_poses_np[cut, 2])
        diff_line.set_xdata(pose_diff[cut, 1])
        diff_line.set_ydata(pose_diff[cut, 2])
        return [gt_line, mcap_line, diff_line]

    frames = range(400, len(gt_poses), 10)
    anim = FuncAnimation(fig, animate, frames=frames, interval=50)  # noqa

    ax.set_xlim(np.min(mcap_poses_np[:, 1]) - 1, np.max(mcap_poses_np[:, 1]) + 1)
    ax.set_ylim(np.min(mcap_poses_np[:, 2]) - 1, np.max(mcap_poses_np[:, 2]) + 1)
    ax.set_aspect("equal", adjustable="box")
    ax.grid()
    ax.legend()

    diff_max = np.nanmax(np.abs(pose_diff[:, 1:].flatten()))
    ax2.set_xlim(-diff_max, diff_max)
    ax2.set_ylim(-diff_max, diff_max)
    ax2.set_aspect("equal", adjustable="box")
    ax2.grid()

    fig.set_size_inches(20, 10)
    fig.tight_layout()
    # anim.save(ROOT / "anim.mp4")
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

    gt_poses_np = np.asarray(gt_poses)
    plt.plot(gt_poses_np[:,0], gt_poses_np[:,3], label='Cкорость GT')
    plt.plot(V_x[:,0], (V_x[:,1] + V_x[:,2])/2.0, label='Скорость одометрия')
    plt.plot(back_wheels_speed[:, 0], back_wheels_speed[:, 1], label='Скорость по двигателю')

    plt.plot(V_x[:, 0], curvate_odom, label='Кривизна одометрия')
    plt.plot(targ_curv_np[:, 0], targ_curv_np[:, 1], label='Кривизна целевая')
    plt.plot(gt_poses_np[:, 0], gt_poses_np[:, 5], label='Кривизна GT')
    plt.plot(mcap_poses_np[:, 0], mcap_poses_np[:, 3], label='Кривизна EKF')

    plt.title('Сравнение скоростей по времени')
    plt.xlabel('Время (с)')
    plt.ylabel('Скорость (м/с) / Кривизна (м^-1)')
    plt.ylim(-2, 2)
    plt.grid(True)
    plt.legend()
    plt.show()

# plot_animation_with_orientation()
# plot_all()
# plot_magn()
# plot_magn_data()
# plot_all()
# plot_sequential()
# plot_animation()
compare_speed_curvate()
