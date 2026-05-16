from setuptools import setup
import os
from glob import glob

package_name = "uav_vlm"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.py")),
        (os.path.join("share", package_name, "config"),
            glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "vlm_spatial_grounding = uav_vlm.vlm_spatial_grounding:main",
            "user_instruction_node = uav_vlm.user_instruction_node:main",
        ],
    },
)