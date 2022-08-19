#!/usr/bin/env python3
"""
Helper script to sync pipenv requirements with custom rosdep dependencies
Usage: python3 scriprs/sync_pydeps.py <ros package name>
"""
import argparse
import subprocess
from typing import Any
import yaml
from pathlib import PosixPath as Path


PYDEPS_YAML_PATH = Path("setup/pydeps.yaml")
assert PYDEPS_YAML_PATH.exists(), "Pydeps YAML not found. Are you in project root?"

# We need to skip these, otherwise pip will complain. TODO: Fix docker image?
APT_INSTALLED_PACKAGES = ["pyserial", "pyyaml", "numpy"]


def main(package_name: str):
    package_path = Path("packages") / package_name
    assert package_path.exists(), f"Package not found: {package_path}"
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
    with open(PYDEPS_YAML_PATH) as file:
        data = yaml.load(file, yaml.Loader)
    data[f"{package_name}_pydeps"] = {"ubuntu": {"pip": {"packages": packages}}}
    with open(PYDEPS_YAML_PATH, "w") as file:
        yaml.dump(data, file, yaml.Dumper, default_flow_style=False)
    print("Updating rosdep index...")
    subprocess.check_call("rosdep update", shell=True)
    print("Installing dependencies...")
    cmd = f"rosdep install -y --ignore-src --from-paths {package_path}"
    subprocess.check_call(cmd, shell=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("package", type=str)
    args = parser.parse_args()
    main(package_name=args.package)
