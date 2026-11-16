from setuptools import find_packages, setup
from glob import glob

package_name = 'adas_vehicle'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='dev@example.com',
    description='ROS 2 ADAS vehicle simulation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'obstacle_detection_node = adas_vehicle.perception.obstacle_detection_node:main',
            'acc_node = adas_vehicle.control.acc_node:main',
            'obstacle_avoidance_node = adas_vehicle.control.obstacle_avoidance_node:main',
            'vehicle_controller_node = adas_vehicle.control.vehicle_controller_node:main',
        ],
    },
)
