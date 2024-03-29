from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gps_nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'models/turtlebot_waffle_gps'),
         glob('models/turtlebot_waffle_gps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='pedro.gonzalez@eia.edu.co,rahafy34@gmail.com',
    description='The gps_nav2 package enhances ROS2-based robotic navigation by integrating GPS waypoint navigation with object detection capabilities.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interactive_waypoint_follower = gps_nav2.interactive_waypoint_follower:main',
            'vision_node = gps_nav2.vision_node:main'
        ],
    },
)
