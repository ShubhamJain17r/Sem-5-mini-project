import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sorting_arm'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shubham',
    maintainer_email='shubham@todo.todo',
    description='Single xArm 1s serial driver and motion control package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'direct_test = sorting_arm.direct_test:main',
            'driver_node = sorting_arm.driver_node:main',
            'polar_teleop = sorting_arm.polar_teleop_node:main',
            'pick_and_place = sorting_arm.pick_and_place_test:main',
            'motion_test = sorting_arm.motion_test_suite:main',
            'vision_node = sorting_arm.vision_node:main',
            'autonomous_sorter = sorting_arm.autonomous_sorter_node:main',
            'mock_vision_node = sorting_arm.mock_vision_node:main',
            'arena_visualizer = sorting_arm.arena_visualizer_node:main',
            'run_autonomous_system = sorting_arm.run_autonomous_system:main',
            'autonomous_system = sorting_arm.run_autonomous_system:main',
        ],
    },
)
