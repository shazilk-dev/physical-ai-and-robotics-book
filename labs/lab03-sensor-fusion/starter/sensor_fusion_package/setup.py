from setuptools import setup
import os
from glob import glob

package_name = 'sensor_fusion_package'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@robotics.edu',
    description='Lab 3: Sensor Fusion (IMU + RealSense Camera)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_fusion_node = sensor_fusion_package.sensor_fusion_node:main',
        ],
    },
)
