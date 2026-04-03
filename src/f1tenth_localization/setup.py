from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'f1tenth_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arc',
    maintainer_email='arc@todo.todo',
    description='Localization bringup for F1TENTH using Nav2 AMCL',
    license='PURDUE INDY ARC',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)