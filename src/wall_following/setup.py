from setuptools import setup

package_name = 'wall_following'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/wall_following.launch.py']),
        ('share/' + package_name + '/params', ['params/wall_following.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@todo.todo',
    description='Wall following controller for F1TENTH (ROS 2 Humble)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_following_node = wall_following.wall_following_node:main',
        ],
    },
)
