#!/usr/bin/env python3

import argparse
import math
import os
from typing import Any, Dict

import yaml


DEFAULT_CONFIG_PATH = (
    "/truck/packages/istk_lib/config/laser_transforms.yaml"
)


def load_yaml(path: str) -> Dict[str, Any]:
    if not os.path.exists(path):
        raise FileNotFoundError(f"YAML file not found: {path}")

    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if data is None:
        data = {}

    if "left_to_right" not in data:
        data["left_to_right"] = {}

    if "translation" not in data["left_to_right"]:
        data["left_to_right"]["translation"] = {}

    if "rotation_rpy" not in data["left_to_right"]:
        data["left_to_right"]["rotation_rpy"] = {}

    if "parent_frame" not in data["left_to_right"]:
        data["left_to_right"]["parent_frame"] = "laser_left"

    if "child_frame" not in data["left_to_right"]:
        data["left_to_right"]["child_frame"] = "laser_right"

    if "base_frame" not in data:
        data["base_frame"] = "base"

    return data


def save_yaml(path: str, data: Dict[str, Any]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(
            data,
            f,
            sort_keys=False,
            allow_unicode=True,
            default_flow_style=False,
        )


def maybe_update(container: Dict[str, Any], key: str, value, relative: bool) -> None:
    if value is None:
        return

    value = float(value)

    if relative:
        old_value = float(container.get(key, 0.0))
        container[key] = old_value + value
    else:
        container[key] = value


def print_current(data: Dict[str, Any]) -> None:
    tf = data["left_to_right"]
    t = tf.get("translation", {})
    r = tf.get("rotation_rpy", {})

    print()
    print("Current laser_left -> laser_right:")
    print(f"  parent_frame: {tf.get('parent_frame', 'laser_left')}")
    print(f"  child_frame:  {tf.get('child_frame', 'laser_right')}")
    print(f"  x:     {float(t.get('x', 0.0)):.9f}")
    print(f"  y:     {float(t.get('y', 0.0)):.9f}")
    print(f"  z:     {float(t.get('z', 0.0)):.9f}")
    print(f"  roll:  {float(r.get('roll', 0.0)):.9f} rad")
    print(f"  pitch: {float(r.get('pitch', 0.0)):.9f} rad")
    print(f"  yaw:   {float(r.get('yaw', 0.0)):.9f} rad")
    print(f"  yaw:   {math.degrees(float(r.get('yaw', 0.0))):.6f} deg")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="Update laser_left -> laser_right transform in laser_transforms.yaml"
    )

    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG_PATH,
        help=f"Path to YAML config. Default: {DEFAULT_CONFIG_PATH}",
    )

    parser.add_argument("--x", type=float, help="Translation x")
    parser.add_argument("--y", type=float, help="Translation y")
    parser.add_argument("--z", type=float, help="Translation z")

    parser.add_argument("--roll", type=float, help="Roll")
    parser.add_argument("--pitch", type=float, help="Pitch")
    parser.add_argument("--yaw", type=float, help="Yaw")

    parser.add_argument(
        "--degrees",
        action="store_true",
        help="Interpret roll/pitch/yaw arguments as degrees instead of radians",
    )

    parser.add_argument(
        "--relative",
        action="store_true",
        help="Add given values to current values instead of replacing them",
    )

    parser.add_argument(
        "--parent-frame",
        help="Parent frame, usually laser_left",
    )

    parser.add_argument(
        "--child-frame",
        help="Child frame, usually laser_right",
    )

    parser.add_argument(
        "--base-frame",
        help="Base frame, usually base",
    )

    parser.add_argument(
        "--show",
        action="store_true",
        help="Only show current transform, do not modify YAML",
    )

    args = parser.parse_args()

    data = load_yaml(args.config)

    if args.show:
        print_current(data)
        return

    if args.degrees:
        if args.roll is not None:
            args.roll = math.radians(args.roll)
        if args.pitch is not None:
            args.pitch = math.radians(args.pitch)
        if args.yaw is not None:
            args.yaw = math.radians(args.yaw)

    tf = data["left_to_right"]
    translation = tf["translation"]
    rotation = tf["rotation_rpy"]

    maybe_update(translation, "x", args.x, args.relative)
    maybe_update(translation, "y", args.y, args.relative)
    maybe_update(translation, "z", args.z, args.relative)

    maybe_update(rotation, "roll", args.roll, args.relative)
    maybe_update(rotation, "pitch", args.pitch, args.relative)
    maybe_update(rotation, "yaw", args.yaw, args.relative)

    if args.parent_frame is not None:
        tf["parent_frame"] = args.parent_frame

    if args.child_frame is not None:
        tf["child_frame"] = args.child_frame

    if args.base_frame is not None:
        data["base_frame"] = args.base_frame

    save_yaml(args.config, data)

    print(f"Updated YAML: {args.config}")
    print_current(data)


if __name__ == "__main__":
    main()
