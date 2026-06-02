from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dcvg_robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thana',
    maintainer_email='66011601@kmitl.ac.th',
    description='DCVG diff-drive robot controller nodes',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'odom = dcvg_robot_bringup.odom:main',
            'forward_kinematics = dcvg_robot_bringup.forward_kinematics:main',
            'inverse_kinematics = dcvg_robot_bringup.inverse_kinematics:main',
            'dcvg_controller = dcvg_robot_bringup.dcvg_controller:main',
        ],
    },
)
