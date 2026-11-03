from glob import glob

from setuptools import find_packages, setup

package_name = 'dcvg_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/behavior_trees', glob('behavior_trees/*.xml')),
        ('share/' + package_name + '/ocr', glob('ocr/*.jpg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thana',
    maintainer_email='66011601@kmitl.ac.th',
    description='Real-hardware bringup for the DCVG autonomous rover',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'low_level_command = dcvg_bringup.low_level_command:main',
            'ntrip_bridge = dcvg_bringup.ntrip_bridge:main',
            'gps_heading_to_imu = dcvg_bringup.gps_heading_to_imu:main',
            'imu_throttle = dcvg_bringup.imu_throttle:main',
            'imu_tilt = dcvg_bringup.imu_tilt:main',
            'record_bag = dcvg_bringup.record_bag:main',
            'terrain_filter = dcvg_bringup.terrain_filter:main',
            'teensy_feedback_server = dcvg_bringup.teensy_feedback_server:main',
        ],
    },
)
