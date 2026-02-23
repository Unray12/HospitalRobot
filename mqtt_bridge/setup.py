from setuptools import find_packages, setup

package_name = 'mqtt_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'paho-mqtt',
    ],
    zip_safe=True,
    maintainer='aclab',
    maintainer_email='aclab@todo.todo',
    description='MQTT bridge between operator inputs and ROS2 robot topics',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mqtt_bridge = mqtt_bridge.MQTTBridgeROS:main',
        ],
    },
)
