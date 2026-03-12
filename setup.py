import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'turtlebot4_reactive_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files so ros2 can use them
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zachary Lilley, Javier Zertuche, Leslie Kelih',
    maintainer_email='zacharydlilley@ou.edu',
    description='Subsumption-architecture reactive controller for TurtleBot 4',
    license='Apache-2.0',
    tests_require=['pytest'],
    # Maps reactive_controller to the main() func in the controller py script
    entry_points={
        'console_scripts': [
            'reactive_controller = turtlebot4_reactive_controller.reactive_controller:main',
        ],
    },
)