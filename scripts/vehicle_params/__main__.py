#!/usr/bin/env python3
import argparse
import json
import math


def add_arguments(parser: argparse.ArgumentParser):
    parser.add_argument(
        "--config-path", default="../../packages/planning_node/config.json"
    )
    parser.add_argument(
        "--new-config-path", default="../../packages/planning_node/new_config.json"
    )


def main(args: argparse.Namespace):
    with open(args.config_path) as file:
        config = json.load(file)

    vehicle_params = config["vehicle"]
    shape = vehicle_params["shape"]
    margin = 1.0 + vehicle_params["circles_approximation"]["margin"]
    circle_spacing = vehicle_params["circles_approximation"]["circle_spacing"]

    min_side = shape["height"]
    max_side = shape["width"]
    assert min_side <= max_side

    radius = margin * math.sqrt(2) * min_side / 2.0
    print("Radius with margin: {}".format(radius))

    circle_count = math.ceil((max_side - min_side) / (circle_spacing * min_side)) + 1
    print("Circle count: {}".format(circle_count))

    if circle_count > 1:
        actual_spacing = (max_side - min_side) / (circle_count - 1) / min_side
        print(
            "Given spacing: {:.3f}, Actual spacing: {:.3f}".format(
                circle_spacing, actual_spacing
            )
        )

        alpha = math.sqrt(2 * margin**2 - actual_spacing**2)
        if alpha < margin:
            print("WARNING: min distance margin is less than target margin")
        print("Min distance margin delta: {:.5f}".format(alpha - margin))

        circles = []
        for i in range(circle_count):
            circles.append(
                {
                    "center": {
                        "x": min_side / 2
                        + (max_side - min_side) * i / (circle_count - 1),
                        "y": min_side / 2,
                    },
                    "radius": radius,
                }
            )

    else:
        # min_side == max_side
        print("Using single circle")
        circles = [
            {
                "center": {
                    "x": max_side / 2,
                    "y": min_side / 2,
                }
            }
        ]

    config["vehicle"]["circles_approximation"]["circles"] = circles
    with open(args.new_config_path, "w") as file:
        json.dump(config, file, indent=4)
        file.write("\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    add_arguments(parser)
    args = parser.parse_args()
    main(args)
