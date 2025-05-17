from setuptools import setup

package_name = 'serial_distance'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shreya',
    maintainer_email='shreya@example.com',
    description='Publishes Arduino HC-SR04 distance over ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'distance_publisher = serial_distance.distance_publisher:main',
        ],
    },
)

