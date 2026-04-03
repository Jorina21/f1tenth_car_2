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
            'build_trajectory = trajectory_tools.build_trajectory:main',
            'plot_trajectory = trajectory_tools.plot_trajectory:main',
            'plot_waypoints = trajectory_tools.plot_waypoints:main',
            'overlay_waypoints_on_map = trajectory_tools.overlay_waypoints_on_map:main',
            'smooth_waypoints = trajectory_tools.smooth_waypoints:main',
        ],
    },
)