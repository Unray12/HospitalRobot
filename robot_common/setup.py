from setuptools import find_packages, setup

package_name = "robot_common"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    package_data={
        "robot_common": [
            "config/*.yaml",
            "config/*.yml",
            "config.yaml",        # monolithic fallback
            "config.json",        # legacy back-compat
            "plans/*.yaml",
            "plans/*.yml",
            "plans/*.json",       # legacy back-compat
        ],
    },
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="aclab",
    maintainer_email="aclab@example.com",
    description="Common utilities for robot packages",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
)
