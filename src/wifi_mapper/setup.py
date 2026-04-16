from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wifi_mapper'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='robot@example.com',
    description='Cartographie WiFi avec iRobot Create 3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explorer_node    = wifi_mapper.explorer_node:main',
            'wifi_scanner_node = wifi_mapper.wifi_scanner_node:main',
            'heatmap_node     = wifi_mapper.heatmap_node:main',
        ],
    },
)
