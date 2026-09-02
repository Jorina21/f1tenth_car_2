from setuptools import setup
import os
from glob import glob

package_name = 'ttc_brake_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'),
            glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arc',
    maintainer_email='arc@example.com',
    description='Simple TTC emergency brake for F1TENTH',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ttc_brake_manager = ttc_brake_manager.ttc_brake_manager_node:main',
            'simple_ttc_brake = ttc_brake_manager.simple_ttc_brake_node:main',
        ],
    },
)