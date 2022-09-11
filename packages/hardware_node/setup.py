from glob import glob

from setuptools import setup

package_name = "hardware_node"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*")),
        (f"share/{package_name}/resource", ["resource/steering.csv"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "main = hardware_node.node:main",
        ],
    },
)
