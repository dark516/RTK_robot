from glob import glob
from setuptools import find_packages, setup


package_name = "traffic_sign_detector"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/models", glob("models/*.pt")),
        ("share/" + package_name + "/models", glob("models/*.md")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Maintainer",
    maintainer_email="maintainer@example.com",
    description="YOLO traffic-sign detection and distance estimation for ROS 2.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "traffic_sign_detector = traffic_sign_detector.detector_node:main",
            "traffic_sign_camera_streamer = traffic_sign_detector.camera_streamer_node:main",
            "traffic_sign_detection_tcp_server = traffic_sign_detector.detection_tcp_server_node:main",
            "traffic_sign_detection_tcp_receiver = traffic_sign_detector.detection_tcp_receiver_node:main",
        ],
    },
)
