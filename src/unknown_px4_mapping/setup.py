from setuptools import find_packages, setup
import os
from glob import glob

package_name = "unknown_px4_mapping"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
        (os.path.join("share", package_name, "behavior_trees"), glob("behavior_trees/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ghosts",
    maintainer_email="ghosts@example.com",
    description="PX4 Gazebo SLAM Toolbox mapping package",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
    "console_scripts": [
        "odom_tf_publisher = unknown_px4_mapping.odom_tf_publisher:main",
        "dynamic_pose_to_odom = unknown_px4_mapping.dynamic_pose_to_odom:main",
        "cmd_vel_to_px4 = unknown_px4_mapping.cmd_vel_to_px4:main",
     ],
   },
)
