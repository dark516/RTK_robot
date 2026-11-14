from setuptools import find_packages, setup

package_name = "fleet_text_bridge_ros2"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", ["launch/bridge.launch.py", "launch/rover_agent_stack.launch.py"]),
    ],
    install_requires=["setuptools", "paho-mqtt>=1.6"],
    zip_safe=True,
    maintainer="Project Maintainer",
    maintainer_email="maintainer@example.com",
    description="MQTT to ROS 2 String topic bridge for robot text commands.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "bridge_node = fleet_text_bridge_ros2.bridge_node:main",
        ],
    },
)
