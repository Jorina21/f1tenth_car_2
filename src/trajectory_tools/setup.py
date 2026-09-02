from setuptools import setup

package_name = 'trajectory_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arc',
    maintainer_email='jorina21@gmail.com',
    description='Offline trajectory processing tools for F1TENTH',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'overlay_waypoints_on_map = trajectory_tools.overlay_waypoints_on_map:main',
            "generate_trajectory = trajectory_tools.generate_trajectory:main",
            'map_to_xy = trajectory_tools.map_to_xy:main',
            'pixel_anchors_to_xy = trajectory_tools.pixel_anchors_to_xy:main',
        ],
    },
)