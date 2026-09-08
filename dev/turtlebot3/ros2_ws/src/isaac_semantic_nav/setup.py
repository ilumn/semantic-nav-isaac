from glob import glob
import os

from setuptools import find_packages, setup


package_name = "isaac_semantic_nav"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Semantic Navigation Maintainers",
    maintainer_email="user@todo.todo",
    description=(
        "Isaac Sim ROS 2 bringup and simulator contract diagnostics for the "
        "TurtleBot3 semantic-navigation stack."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "contract_checker = isaac_semantic_nav.contract_checker_node:main",
        ],
    },
)
