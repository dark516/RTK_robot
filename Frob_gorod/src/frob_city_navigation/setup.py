from glob import glob
import os

from setuptools import find_packages, setup

package_name = "frob_city_navigation"

setup(
    name=package_name,
    version="0.6.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "PyYAML", "shapely>=2.0"],
    zip_safe=True,
    maintainer="Frob City Team",
    maintainer_email="maintainer@example.com",
    description="Frob City world model, directed planner, and map_planner ROS wrapper.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "map_planner = frob_city_navigation.map_planner_node:main",
        "detection_adapter = frob_city_navigation.detection_adapter_node:main",
        "motion_guard = frob_city_navigation.motion_guard_node:main",

        "navigator = frob_city_navigation.navigator_node:main",
        "city_readiness = frob_city_navigation.readiness_node:main",]},
)
