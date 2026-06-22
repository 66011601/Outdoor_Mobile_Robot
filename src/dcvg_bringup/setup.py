from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dcvg_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thana',
    maintainer_email='66011601@kmitl.ac.th',
    description='Real-hardware bringup for the DCVG autonomous rover.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            # Real-hardware helper nodes go here (e.g. cmd_vel → MAVROS bridge).
            'vo_plotter = dcvg_bringup.vo_plotter:main',
            'stereo_feature_overlay = dcvg_bringup.stereo_feature_overlay:main',
        ],
    },
)
