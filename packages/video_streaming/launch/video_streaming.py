import socket
from pathlib import Path

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction

LAUNCH_PARAMS = {}


def get_external_ips(hostname: str):
    subnets = ["192.168.", "100.64."]
    _, _, addrs = socket.gethostbyname_ex(hostname)
    results = [addr for addr in addrs if any(addr.startswith(x) for x in subnets)]
    return results


def launch_setup(context):
    share_path = Path(get_package_share_directory("video_streaming"))
    config_path = share_path / "mediamtx.yaml"
    external_dns_name = "jetson-nx-3.local"
    external_ips = get_external_ips(external_dns_name)
    print(f"[video_streaming launch] External DNS name: {external_dns_name}")
    print(f"[video_streaming launch] Detected external ips: {external_ips}")

    node = ExecuteProcess(
        cmd=["mediamtx", str(config_path)],
        name="video_streaming",
        output="screen",
        additional_env={"MTX_WEBRTCADDITIONALHOSTS": ",".join(external_ips)},
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
