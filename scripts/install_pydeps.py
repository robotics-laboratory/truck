#!/usr/bin/env python3
"""
Helper script to sync pipenv requirements with custom rosdep dependencies
Usage: python3 scriprs/sync_pydeps.py <ros package name>
"""
import argparse
import subprocess
import sys
from pathlib import PosixPath as Path

import yaml

# We need to skip these, otherwise pip will complain. TODO: Fix docker image?
APT_INSTALLED_PACKAGES = ["pyserial", "pyyaml", "numpy"]
ROSDEP_SOURCES_PATH = Path("/etc/ros/rosdep/sources.list.d")


def parse_requirements(package_path: Path):
    assert (package_path / "Pipfile").exists(), "Pipenv file not found"
    print("Parsing pipenv requirements...")
    stdout = subprocess.check_output(
        args="pipenv requirements",
        cwd=str(package_path),
        shell=True,
    )
    packages = []
    for line in stdout.decode().split():
        if line.startswith("-") or line.startswith("#") or "pypi.org" in line:
            print(f"Skipping line: {line!r} (not a package)")
            continue
        package = line.split(";")[0]
        if package.split("=")[0].lower() in APT_INSTALLED_PACKAGES:
            print(f"Skipping package: {package!r} (installed via apt)")
            continue
        packages.append(package)
    print(f"Found {len(packages)} python dependencies")
    return packages


def install_via_pip(packages: list):
    print("installing via pip...")
    cmd = f"{sys.executable} -m pip install {' '.join(packages)}"
    subprocess.check_call(cmd, shell=True)


def install_via_rosdep(packages: list, package_name: str, package_path: Path):
    # FIXME: This fails with "cannot verify successfull installation for pip"
    # FIXME: Because rosdep does not allow to specify versions for pip packages
    # FIXME: (but actually packages are installed, it's just rosdep freaking out)
    # FIXME: PR is still open: https://github.com/ros-infrastructure/rosdep/pull/694
    print("Installing via rosdep...")
    pydeps_yaml_path = package_path / "pydeps.yaml"
    dep_key = f"{package_name}_pydeps"  # This is our custom dependency name
    data = {dep_key: {"ubuntu": {"pip": {"packages": packages}}}}
    print(f"Writing dependency file: {pydeps_yaml_path}")
    with open(pydeps_yaml_path, "w") as file:
        yaml.dump(data, file, yaml.Dumper, default_flow_style=False)
    rosdep_source_path = ROSDEP_SOURCES_PATH / f"{dep_key}.list"
    print(f"Writing rosdep source file: {rosdep_source_path}")
    with open(rosdep_source_path, "w") as file:
        file.write(f"yaml file://{pydeps_yaml_path.absolute()}")
    print("Updating rosdep index...")
    subprocess.check_call("rosdep update", shell=True)
    print("Installing dependencies...")
    cmd = f"rosdep install -y --ignore-src --from-paths {package_path}"
    subprocess.check_call(cmd, shell=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("package", type=str)
    parser.add_argument("--via-pip", action="store_true", default=False)
    parser.add_argument("--via-rosdep", action="store_true", default=False)
    args = parser.parse_args()
    if not (args.via_pip ^ args.via_rosdep):
        print("Select one: --via-pip or --via-rosdep")
        exit(1)
    package_path = Path("packages") / args.package
    assert package_path.exists(), f"Package not found: {package_path}"
    packages = parse_requirements(package_path)
    if args.via_pip:
        install_via_pip(packages)
    if args.via_rosdep:
        install_via_rosdep(packages, args.package, package_path)
