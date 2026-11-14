from glob import glob
import os

from setuptools import find_packages, setup


package_name = "frob_city_web"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    package_data={"frob_city_web.static": ["*.html", "*.css", "*.js"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Frob City Team",
    maintainer_email="maintainer@example.com",
    description="Frob City web operator dashboard.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["frob_city_web = frob_city_web.web_node:main"]},
)
