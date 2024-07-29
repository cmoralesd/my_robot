import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share/' + package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share/' + package_name, 'description'), glob(os.path.join('description', '*.*'))),
        (os.path.join('share/' + package_name, 'description/urdf'), glob(os.path.join('description/urdf', '*.*'))),
        (os.path.join('share/' + package_name, 'config'), glob(os.path.join('config', '*.*'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='todo@todo.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_robot_controller = my_robot.my_robot_controller:main',
            'joy_velocity_publisher = my_robot.joy_velocity_publisher:main',
        ],
    },
)
