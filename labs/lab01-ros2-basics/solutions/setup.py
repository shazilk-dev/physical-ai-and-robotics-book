from setuptools import setup

package_name = 'heartbeat_package'

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
    maintainer='Student Name',
    maintainer_email='student@example.com',
    description='Lab 1: Heartbeat Publisher - ROS 2 Basics (Solution)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'heartbeat_node = heartbeat_package.heartbeat_node:main'
        ],
    },
)
