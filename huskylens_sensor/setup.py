from setuptools import find_packages, setup

package_name = "huskylens_sensor"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "pyserial",
    ],
    zip_safe=True,
    maintainer="aclab",
    maintainer_email="aclab@todo.todo",
    description="HuskyLens serial sensor bridge node",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "huskylens_sensor = huskylens_sensor.huskylens_sensor_node:main",
        ],
    },
)
