from setuptools import find_packages, setup
import os
from glob import glob

package_name = "underwater_target_detection"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="UUVTracking Maintainer",
    maintainer_email="maintainer@example.com",
    description="Monocular vision package for underwater target detection.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "detection_node = underwater_target_detection.detection_node:main",
        ],
    },
)
