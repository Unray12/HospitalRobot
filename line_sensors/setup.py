from setuptools import find_packages, setup

package_name = 'line_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        'line_sensors': ['lineData.json'],
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
    ],
    zip_safe=True,
    maintainer='aclab',
    maintainer_email='aclab@todo.todo',
    description='Line sensor driver node',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'line_sensor_driver = line_sensors.line_sensor_driver_node:main',
        ],
    },
)
