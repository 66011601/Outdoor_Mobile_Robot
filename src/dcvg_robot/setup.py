from glob import glob

from setuptools import find_packages, setup

package_name = 'dcvg_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thana',
    maintainer_email='66011601@kmitl.ac.th',
    description='DCVG rover robot description, route planning web app and waypoint follower',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'osm_route_app = dcvg_robot.osm_route_app:main',
            'route_waypoint_follower = dcvg_robot.route_waypoint_follower:main',
        ],
    },
)
