from setuptools import find_packages, setup
import os
from glob import glob

package_name = "underwater_target_control"

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
    description="Motion-control package for underwater target following.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "control_node = underwater_target_control.control_node:main",
        ],
    },
)
