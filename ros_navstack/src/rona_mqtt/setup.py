from setuptools import setup

package_name = 'rona_mqtt'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    install_requires=[
        'setuptools',
        'tf-transformations',
        'paho-mqtt',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@domain.com',
    description='A ROS2 package to receive waypoints via MQTT and navigate through poses',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mqtt_waypoint_receiver = rona_mqtt.mqtt_waypoint_receiver:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mqtt_waypoint_receiver_launch.py']),
    ],
)
