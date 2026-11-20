from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'uav_ugv_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'missions'), glob('missions/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cahit Ikbal Er',
    maintainer_email='er.c@northeastern.edu',
    description='Coordinate Crazyflie UAVs and TurtleBot 4 UGVs in a motion-capture arena.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uav_node = uav_ugv_nav.uav.uav_node:main',
            'ugv_node = uav_ugv_nav.ugv.ugv_node:main',
            'check_mission = uav_ugv_nav.check_mission:main',
            'cf_check = uav_ugv_nav.uav.cf_check:main',
            'scan_check = uav_ugv_nav.ugv.scan_check:main',
        ],
    },
)
