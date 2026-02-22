from setuptools import find_packages, setup

package_name = "line_follower"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    package_data={
        "line_follower": ["config.json"],
    },
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aclab",
    maintainer_email="aclab@todo.todo",
    description="Line follower FSM and PID controller node",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "line_follower = line_follower.line_follower_node:main",
        ],
    },
)
