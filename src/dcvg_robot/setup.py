from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dcvg_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
        (os.path.join('share', package_name, 'models', 'dcvg_robot'), glob('models/dcvg_robot/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thana',
    maintainer_email='66011601@kmitl.ac.th',
    description='DCVG diff-drive robot controller nodes',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'route_waypoint_follower = dcvg_robot.route_waypoint_follower:main',
            'osm_route_generator = dcvg_robot.osm_route_generator:main',
            'osm_route_app = dcvg_robot.osm_route_app:main',
            'odom_accuracy_plot = dcvg_robot.odom_accuracy_plot:main',
            'feature_overlay = dcvg_robot.feature_overlay:main',
            'route_recorder = dcvg_robot.route_recorder:main',
            'recorded_route_runner = dcvg_robot.recorded_route_runner:main',
            'stereo_feature_overlay = dcvg_robot.stereo_feature_overlay:main',
        ],
    },
)
