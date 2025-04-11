import json

import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
import matplotlib.transforms as transf
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

mcap_pose = []

with open("/Users/zhora/Downloads/Aruco_Odometry/run-01.mcap", "rb") as mcap_file:
    reader = make_reader(mcap_file, decoder_factories=[DecoderFactory()])
    for schema, channel, message, ros_msg in reader.iter_decoded_messages(
        topics=["/ekf/odometry/filtered"]
    ):
        mcap_pose.append((ros_msg.pose.pose.position.x, ros_msg.pose.pose.position.y))

mcap_pose = np.array(mcap_pose)
tr = transf.Affine2D()
tr.rotate_deg(45)
tr.translate(-4.2, -4.2)

mcap_pose = tr.transform(mcap_pose)

with open("odom.json", "r") as odom_file:
    odom = json.load(odom_file)
    poses = []
    for pt in odom:
        if pt["x_pos"] != "None":
            poses.append([float(pt["x_pos"]), float(pt["y_pos"])])
    poses = np.asarray(poses)

    fig, ax = plt.subplots()

    ax.plot(poses[:, 0], poses[:, 1], "-", linewidth=1)

    loc = plticker.MultipleLocator(
        base=0.6
    )  # this locator puts ticks at regular intervals
    ax.xaxis.set_major_locator(loc)
    ax.yaxis.set_major_locator(loc)

    ax.plot(mcap_pose[:, 0], mcap_pose[:, 1], "-", linewidth=1)

    plt.grid()
    plt.show()
