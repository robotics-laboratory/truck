import argparse
import json

import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
import matplotlib.transforms as transf
import numpy as np
from matplotlib.animation import FuncAnimation
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

parser = argparse.ArgumentParser(description="Plot car odometry")
parser.add_argument(
    "-p", "--path", default="./", help="Path to folder with experiment data."
)

args = parser.parse_args()

with open(args.path + "/params.json", "r") as params_file:
    params = json.load(params_file)
    fps = params["fps"]
    grid_size = params["grid_size"]
    distortion_coeffs = np.asarray(params["grid_camera_dist"], dtype=np.float32)
    camera_matrix = np.asarray(params["grid_camera_matrix"], dtype=np.float32)

TF = transf.Affine2D()
TF.rotate_deg(45)
TF.translate(-7 * grid_size, -7 * grid_size)


def read_json_poses(path):
    data = json.load(open(path))
    result = []
    for item in data:
        x, y = None, None
        if item["x_pos"] is not None:
            x, y = item["x_pos"], item["y_pos"]
        ts = 1 / fps * int(item["frame_num"])
        result.append((ts, x, y))
    return result


def read_mcap_poses(path):
    result = []
    ts0 = None
    with open(path, "rb") as mcap_file:
        reader = make_reader(mcap_file, decoder_factories=[DecoderFactory()])
        for _, _, message, ros_msg in reader.iter_decoded_messages(
            topics=["/ekf/odometry/filtered"]
        ):
            x = ros_msg.pose.pose.position.x
            y = ros_msg.pose.pose.position.y
            x, y = TF.transform((x, y))
            ts = message.publish_time / 10**9
            if ts0 is None:
                ts0 = ts
            result.append((ts - ts0, x, y))
    return result


def get_time_offset(gt_poses, mcap_poses):
    ts0, x0, y0 = next(item for item in gt_poses if item[1] is not None)
    min_dist, min_ts = float("+inf"), None
    deadline = 30

    for ts, x, y in mcap_poses:
        if x is None or y is None:
            continue
        dist = np.sqrt((x0 - x) ** 2 + (y0 - y) ** 2)
        if dist < min_dist:
            min_dist = dist
            min_ts = ts
        if ts > deadline:
            break

    print(f"MIN DIST: {min_dist:.3f}m")
    print(f"GT TIME: {ts0:.3f}s")
    print(f"MCAP TIME: {min_ts:.3f}s")
    print(f"TIME OFFSET: {ts0 - min_ts:.3f}s")
    return ts0 - min_ts


def sync_mcap_poses(gt_poses, mcap_poses):
    time_offset = get_time_offset(gt_poses, mcap_poses)
    mcap_index = 0
    result = []
    for gt_ts, _, _ in gt_poses:
        while mcap_index < len(mcap_poses):
            mcap_ts, mcap_x, mcap_y = mcap_poses[mcap_index]
            mcap_ts += time_offset
            if mcap_ts > gt_ts:
                result.append((mcap_ts, mcap_x, mcap_y))
                break
            mcap_index += 1
    return result


gt_poses = read_json_poses(args.path + "/odom.json")
mcap_poses = read_mcap_poses(args.path + "/odom_live.mcap")
mcap_poses = sync_mcap_poses(gt_poses, mcap_poses)
dropped = len(gt_poses) - len(mcap_poses)
gt_poses = gt_poses[-len(mcap_poses) :]
print(f"DROPPED {dropped / fps:.3f}s OF FRAMES")


def plot_all():
    fig, ax = plt.subplots()
    loc = plticker.MultipleLocator(base=grid_size)
    ax.xaxis.set_major_locator(loc)
    ax.yaxis.set_major_locator(loc)

    gt_poses_np = np.asarray(gt_poses)
    mcap_poses_np = np.asarray(mcap_poses)
    ax.plot(gt_poses_np[:, 1], gt_poses_np[:, 2], "-", linewidth=1)
    ax.plot(mcap_poses_np[:, 1], mcap_poses_np[:, 2], "-", linewidth=1)

    plt.axis("equal")
    plt.grid()
    plt.show()


def plot_sequential():
    step = 30 * fps
    offset = 400

    while offset < len(gt_poses):
        fig, ax = plt.subplots()
        loc = plticker.MultipleLocator(base=grid_size)
        ax.xaxis.set_major_locator(loc)
        ax.yaxis.set_major_locator(loc)

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
    loc = plticker.MultipleLocator(base=grid_size)
    ax.xaxis.set_major_locator(loc)
    ax.yaxis.set_major_locator(loc)

    [gt_line] = ax.plot([], [], lw=1, label="gt")
    [mcap_line] = ax.plot([], [], lw=1, label="odom")
    [diff_line] = ax2.plot([], [], lw=1)
    gt_poses_np = np.asarray(gt_poses, dtype=np.float32)
    mcap_poses_np = np.asarray(mcap_poses, dtype=np.float32)
    pose_diff = gt_poses_np - mcap_poses_np

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


# plot_all()
# plot_sequential()
plot_animation()
