import os
import socket
from pathlib import Path

from ament_index_python import get_package_share_directory as get_pkg_share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction

LAUNCH_PARAMS = {}  # TODO


def launch_setup(context):
    ext_hostname = os.environ.get("MEDIAMTX_WEBRTC_HOSTNAME", "localhost")
    ext_port = os.environ.get("MEDIAMTX_WEBRTC_PORT", "8189")
    ext_ips = list(set(socket.gethostbyname_ex(ext_hostname)[2]))
    config_path = Path(get_pkg_share("video_streaming")) / "mediamtx.yaml"

    print(f"[video_streaming launch] External hostname: {ext_hostname}")
    print(f"[video_streaming launch] External IPs: {', '.join(ext_ips)}")
    print(f"[video_streaming launch] WebRTC port: {ext_port}")
    print(f"[video_streaming launch] Config path: {config_path}")

    env = {
        "MTX_WEBRTCLOCALUDPADDRESS": f":{ext_port}",
        "MTX_WEBRTCADDITIONALHOSTS": ",".join(ext_ips + [ext_hostname]),
    }
    node = ExecuteProcess(
        cmd=["mediamtx", str(config_path)],
        name="video_streaming",
        output="screen",
        additional_env=env,
    )
    return [node]


def generate_launch_description():
    launch_params = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in LAUNCH_PARAMS.items()
    ]
    description = LaunchDescription(launch_params)
    description.add_action(OpaqueFunction(function=launch_setup))
    return description
